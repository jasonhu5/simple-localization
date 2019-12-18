#!/usr/bin/env python

import math
import numpy as np
import os
import rospy
import tf
from duckietown import DTROS
from duckietown_msgs.msg import (
    BoolStamped, Twist2DStamped, FSMState, AprilTagExtended, WheelsCmdStamped
)
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

class TowerOffset():
    """
    Helper class for watchtower offset measurement.
    Before user confirms, only candidate is updated.
    Once user confirms, the candidate replaces the offset value.
    
    What is offset?
    - In following LocalizationNode, apriltag processor output is used for
    faster localization. The local position and orientation is transformed
    from watchtower frame to map world frame.
    - Offset refers to the fixed position and orientation difference between
    the transformed (x, y, rotation) to the real (x, y, rotation).
    """

    def __init__(self):
        self.candidate = (0.0, 0.0, 0.0)
        self.value = (0.0, 0.0, 0.0)

    def update(self, offset):
        """
        Update the offset from watchtower frames

        @args:
            offset -> (float, float, float): x, y and rotation angle
        """
        self.candidate = offset

    def useIfGood(self):
        """
        Elect the candidate
        """
        self.value = self.candidate


class LocalizationNode(DTROS):
    """
    The LocalizationNode uses input from cslam localization, which is accurate
    but slow, and apriltag processors (fast but not integrated), to generate
    a local fast and accurate estimate of a Duckiebot's position and orientation.
    """

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)

        # Info of current watchtowers 
        self.tower_list = dict()
        # Offset of watchtowers
        self.tower_offset = dict()

        ## parameters
        # threshold for tag validity in seconds, older tag info is not used
        self.parameters['~tag_ttl'] = None
        # trigger offset measurement procedure on/off
        self.parameters['~measuring_offset'] = None
        # print offset measurement related debug prints
        self.parameters['~offset_debug_print'] = None
        # the tower that is currently being measured offset for
        self.parameters['~offset_tower_id'] = None
        # the bot used for offset measurement
        self.parameters['~offset_bot_id'] = None
        self.updateParameters()

        # # SUBSCRIBE
        # Subscribe to topics online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cb_markers, queue_size=30)
        # Subscribe to processed apriltag information
        self.sub_tags = rospy.Subscriber(
            "/poses_acquisition/poses",
            AprilTagExtended,
            self.cb_tags,
            queue_size=1,
        )

        # # PUBLISH
        # publish the bots and towers' locations as markers for visualization
        self.pub_marker = rospy.Publisher("/simple_loc", Marker, queue_size=10)
        # publish bot related data in numbers
        self.pub_bot_loc = rospy.Publisher(
            "/simple_loc_bots",
            Float64MultiArray,
            queue_size=10,
        )

        # matches tag_id => bot_id
        tag_bot_mapping_file = os.path.join(
            "/code/catkin_ws/src",
            "simple-localization/packages/localization/src",
            "bot_tag_mapping"
        )
        self.tag2bot = self.read_bot_tag_mapping(tag_bot_mapping_file)

        # localization from existing cslam
        self.measure_bot_loc = None 
        # localization from this repo
        self.measure_bot_simple_loc = None
        # the bot used for measuring offset
        self.current_measure_bot = self.parameters["~offset_bot_id"]
        # the tower currently being measured for
        self.current_measure_tower = self.parameters["~offset_tower_id"]
        # a set of all watchtowers that are offset adjusted
        self.measured_towers = set([])


    def read_bot_tag_mapping(self, file_path):
        """
        Reads the mapping file and return a dictionary for lookup

        @args:
            file_path -> str: the path in container where the mapping file is
        
        @return:
            the dictionary from tag_id to bot_id
        """

        with open(file_path) as f:
            lines = f.read().split("\n")
            data = [l.split(':') for l in lines if any(
                char.isdigit() for char in l)]
            mapping = {}
            for d in data:
                mapping[int(d[0])] = int(d[1])
            return mapping


    def qv_mult(self, q1, v1):
        """
        rotate a vector by a quaternion

        @args:
            q1 -> Quaternion: rotation;
            v1 -> np.array(3): the vector
        
        @return:
            the rotated vector
        """
        q2 = list(tf.transformations.unit_vector(v1)) + [0.0]
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2), 
            tf.transformations.quaternion_conjugate(q1)
        )[:3] * np.linalg.norm(v1)


    def prep_marker(self, ns, x, y, q, idx, tst=False):
        """
        prepare marker for publish, based on namespace, and a flag for special
        cases, e.g. during offset measurement
        
        Currently, it's a red arrow for duckiebots, and green cube for 
        watchtowers that are offset adjusted. And it's yellow cube for a
        watchtower undergoing offset measurement.

        @args:
            ns -> str: namespace, "duckiebot" or "watchtower";
            x -> float: x position, in meters;
            y -> float: y position, in meters;
            q -> Quaternion: the heading;
            idx -> int: the ID;
            tst -> bool: in special case or not

        @return:
            the prepared marker.
        """

        marker = Marker()
        # set basic information
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.id = idx 
        marker.ns = "simple_loc"
        # set shape and color
        marker.type = Marker.ARROW if ns == "duckiebot" else Marker.CUBE
        marker.action = marker.ADD
        marker.scale = Vector3(0.15, 0.06, 0.06) if ns == "duckiebot" else Vector3(0.1, 0.1, 2)
        marker.color = ColorRGBA(1, 0, 0, 1) if ns == "duckiebot" else ColorRGBA(1, 1, 0, 1)
        # if special cases, set special color/scale
        if tst:
            marker.color = ColorRGBA(0, 1, 0, 1)
            if ns != "duckiebot":
                marker.scale = Vector3(0.08, 0.08, 0.8)
        # position and orientation
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation = q

        return marker


    def cb_tags(self, msg):
        """
        Callback when a message from apriltag processors is received.
        Extract the bot's location, apply transformations to make the
        local-to-watchtower position and orientation global.
        Decide to discard, use for localization, use for offset measurement.

        @args:
            msg -> AprilTagExtended: detected apriltag in each watchtower frame
        """

        # watchtower ID
        frame_id = int(msg.header.frame_id[-3:])
        # bot ID
        idx = self.tag2bot.get(msg.tag_id)

        # if a bot_id can be found
        if idx:
            # if do not have watchtower position, skip
            if not self.tower_list.get(frame_id):
                return
            # don't use too old data
            since_created = rospy.Time.now().to_sec() - msg.header.stamp.secs
            if since_created > self.parameters['~tag_ttl']:
                return

            # extract from message
            pos = msg.transform.translation
            rot = msg.transform.rotation
            # reference frame
            ref_pos = self.tower_list[frame_id].pose.position
            ref_rot = self.tower_list[frame_id].pose.orientation
            # transform local position to global
            vec = [pos.x, pos.y, pos.z]
            qua = [ref_rot.x, ref_rot.y, ref_rot.z, -ref_rot.w]
            tf_pos = self.qv_mult(qua, vec)
            # global position
            x = tf_pos[0] + ref_pos.x
            y = -tf_pos[1] + ref_pos.y

            # transform local orientation to global
            ref_qua = [ref_rot.x, ref_rot.y, ref_rot.z, ref_rot.w]
            self_qua = [rot.x, rot.y, rot.z, -rot.w]
            rot = Quaternion()
            result = tf.transformations.quaternion_multiply(self_qua, ref_qua)
            # compensate a fixed heading difference
            q = tf.transformations.quaternion_from_euler(0, 0, 1.5707)
            result = tf.transformations.quaternion_multiply(result, q)
            # global orientation
            rot.x, rot.y, rot.z, rot.w = result

            # prepare data for offset measurement use
            if (idx == self.current_measure_bot and 
                frame_id == self.current_measure_tower):
                self.measure_bot_simple_loc = (
                    x, y, self.quat2angle(rot))

            # only publish stuff if the watchtower is offset adjusted 
            if frame_id in self.measured_towers:
                # adjust offset
                x -= self.tower_offset[frame_id].value[0]
                y -= self.tower_offset[frame_id].value[1]
                # create markers for rviz
                self.pub_marker.publish(self.prep_marker(
                    "duckiebot", x, y, rot, idx))
                # publish bot info details
                bot_loc = Float64MultiArray()
                stmp = rospy.get_rostime()
                bot_loc.data = [
                    float(idx),  # bot_id
                    x, y, self.quat2angle(rot), # position & orientation
                    stmp.secs, stmp.nsecs,  # timestamp
                ]
                self.pub_bot_loc.publish(bot_loc)


    def simple_vs_loc_after_correction(self):
        """
        Debug print of the offset, after compensated by candidate offset
        """
        if (not self.measure_bot_loc or not self.measure_bot_simple_loc):
            return
        if self.parameters['~offset_debug_print']:
            offset = self.offset_simple_vs_loc()
            print("[Measuring offset for tower {}] \nx: {}\ny: {}\nz: {}".format(
                self.current_measure_tower,
                offset[0] - self.tower_offset[self.current_measure_tower].candidate[0],
                offset[1] - self.tower_offset[self.current_measure_tower].candidate[1],
                offset[2] - self.tower_offset[self.current_measure_tower].candidate[2]
            ))
    

    def offset_simple_vs_loc(self):
        """
        Calcuate the offset between cslam localization and this simple
        localization

        @return:
            (float, float, float): the current offset
        """
        if (not self.measure_bot_loc or not self.measure_bot_simple_loc):
            return (100.0, 100.0, 100.0)
        return (
            self.measure_bot_simple_loc[0] - self.measure_bot_loc[0],
            self.measure_bot_simple_loc[1] - self.measure_bot_loc[1],
            self.measure_bot_simple_loc[2] - self.measure_bot_loc[2],
        )


    def quat2angle(self, q):
        """
        Convert a quaternion rotation to a rotation angle

        @args:
            q -> Quaternion: the rotation in quaternion format

        @return:
            float: the rotation angle in degrees
        """

        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return (180.0/math.pi * math.atan2(siny_cosp, cosy_cosp))


    def cb_markers(self, msg):
        """
        Callback when receiving cslam localization marker results
        Register watchtower most updated frames.
        Do offset measurement procedure if turned on.

        @args:
            msg -> MarkerArray: cslam online localization markers, containing
                                position, orientation for bots and towers
        """

        # update which bot to use to measure
        if self.current_measure_bot != self.parameters["~offset_bot_id"]:
            self.measure_bot_loc = None
            self.measure_bot_simple_loc = None
            # self.measure_bot_simple_loc = {}
            self.current_measure_bot = self.parameters["~offset_bot_id"]

        for m in msg.markers:
            # care only about duckiebots
            if m.ns == "watchtowers":
                idx = m.id
                if not idx in self.tower_offset:
                    self.tower_offset[idx] = TowerOffset()
                
                self.tower_list[idx] = m
                p = m.pose.position
                r = m.pose.orientation

                # highlight ones being measured
                if self.parameters['~measuring_offset'] and idx == self.current_measure_tower:
                    self.pub_marker.publish(self.prep_marker(
                        "watchtower", p.x, p.y, r, idx
                    ))
                else:
                    # if measured, mark in rviz
                    if idx in self.measured_towers:
                        self.pub_marker.publish(self.prep_marker(
                            "watchtower", p.x, p.y, r, idx, True
                        ))
                    else:
                        # set marker to somewhere not relevent
                        self.pub_marker.publish(self.prep_marker(
                            "watchtower", -10, -10, r, idx
                        ))

            if m.ns == "duckiebots":
                idx = m.id
                # if bot is used for offset measurement
                if idx == self.current_measure_bot:
                    self.measure_bot_loc = (
                        m.pose.position.x,
                        m.pose.position.y,
                        self.quat2angle(m.pose.orientation)
                    )
                    if self.parameters['~measuring_offset']:
                        if self.current_measure_tower in self.tower_offset:
                            self.simple_vs_loc_after_correction()

        # update the offset or commit the offset and move to a next watchtower
        if self.parameters['~measuring_offset']:
            if self.current_measure_tower == self.parameters["~offset_tower_id"]:
                if self.current_measure_tower in self.tower_offset:
                    # show which towers are going through the measurement
                    self.tower_offset[self.current_measure_tower].update(
                        self.offset_simple_vs_loc(),
                    )
            else:
                if self.current_measure_tower in self.tower_offset:
                    # if target tower changed, commit previous offset measurement
                    self.tower_offset[self.current_measure_tower].useIfGood()
                    self.measured_towers.add(self.current_measure_tower)
                    self.measure_bot_simple_loc = None
                self.current_measure_tower = self.parameters["~offset_tower_id"]
        else:
            # if turning off measurement procedure
            if self.current_measure_tower in self.tower_offset:
                # commit last measured offset if tower ID param exists
                self.tower_offset[self.current_measure_tower].useIfGood()
                self.measured_towers.add(self.current_measure_tower)
                self.measure_bot_simple_loc = None
            self.current_measure_tower = -1
            # reset the tower ID param
            rospy.set_param("/simple_localization/loc_node/offset_tower_id", -1)


if __name__ == '__main__':
    # create the node
    node = LocalizationNode(node_name='loc_node')
    # keep spinning
    rospy.spin()