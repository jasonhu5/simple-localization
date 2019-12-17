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
    def __init__(self):
        self.candidate = (0.0, 0.0, 0.0)
        self.value = (0.0, 0.0, 0.0)

    def update(self, offset):
        self.candidate = offset

    def useIfGood(self):
        self.value = self.candidate

class LocalizationNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)

        # List of current watchtowers 
        self.tower_list = dict()
        self.tower_offset = dict()
        # self.tower_offset_measuring_set = set([])

        ## parameters
        self.parameters['~tag_ttl'] = None
        self.parameters['~measuring_offset'] = None
        self.parameters['~offset_debug_print'] = None
        self.parameters['~offset_tower_id'] = None
        self.parameters['~offset_bot_id'] = None
        self.updateParameters()

        # # SUBSCRIBE
        # Subscribe to topics online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cb_markers, queue_size=30)
        self.sub_tags = rospy.Subscriber(
            "/poses_acquisition/poses",
            AprilTagExtended,
            self.cb_tags,
            queue_size=1,
        )

        # # PUBLISH
        self.pub_marker = rospy.Publisher("/simple_loc", Marker, queue_size=10)
        self.pub_bot_loc = rospy.Publisher(
            "/simple_loc_bots",
            Float64MultiArray,
            queue_size=10,
        )

        # tag <=> bot
        tag_bot_mapping_file = os.path.join(
            "/code/catkin_ws/src",
            "simple-localization/packages/localization/src",
            "bot_tag_mapping"
        )
        self.tag2bot = self.read_bot_tag_mapping(tag_bot_mapping_file)

        self.measure_bot_loc = None 
        self.measure_bot_simple_loc = None
        self.current_measure_bot = self.parameters["~offset_bot_id"]
        self.current_measure_tower = self.parameters["~offset_tower_id"]
        self.measured_towers = set([])

        # which tag info to use
        self.latest_tag_stamp = rospy.Time.now().to_sec()


    def read_bot_tag_mapping(self, file_path):
        with open(file_path) as f:
            lines = f.read().split("\n")
            data = [l.split(':') for l in lines if any(
                char.isdigit() for char in l)]
            mapping = {}
            for d in data:
                mapping[int(d[0])] = int(d[1])
            return mapping


    # rotate vector v1 by quaternion q1 
    def qv_mult(self, q1, v1):
        nm = np.linalg.norm(v1)
        v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2), 
            tf.transformations.quaternion_conjugate(q1)
        )[:3] * nm


    def prep_marker(self, ns, x, y, q, idx, tst=False):
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.id = idx 
        marker.ns = "simple_loc"

        marker.type = Marker.ARROW if ns == "duckiebot" else Marker.CUBE
        marker.action = marker.ADD

        marker.scale = Vector3(0.15, 0.06, 0.06) if ns == "duckiebot" else Vector3(0.1, 0.1, 2)
        marker.color = ColorRGBA(1, 0, 0, 1) if ns == "duckiebot" else ColorRGBA(1, 1, 0, 1)

        if tst:
            marker.color = ColorRGBA(0, 1, 0, 1)
            if ns != "duckiebot":
                marker.scale = Vector3(0.08, 0.08, 0.8)

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        marker.pose.orientation = q

        return marker


    def cb_tags(self, msg):
        """
        """
        # watchtower ID
        frame_id = int(msg.header.frame_id[-3:])
        # bot ID
        idx = self.tag2bot.get(msg.tag_id)

        if idx:
            # TIME TO LIVE: don't use too old data
            tag_time_since_created = rospy.Time.now().to_sec() - msg.header.stamp.secs
            # print(tag_time_since_created, self.parameters['~tag_ttl'])
            if tag_time_since_created > self.parameters['~tag_ttl']:
                return

            new_timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 10 ** -9
            time_diff = new_timestamp - self.latest_tag_stamp
            should_use = frame_id in self.measured_towers

            pos = msg.transform.translation
            rot = msg.transform.rotation

            # if do not have watchtower position, skip
            if not self.tower_list.get(frame_id):
                return

            ref_pos = self.tower_list[frame_id].pose.position
            ref_rot = self.tower_list[frame_id].pose.orientation

            # vec = Vector3(pos.x, pos.y, pos.z)
            vec = [pos.x, pos.y, pos.z]
            qua = [ref_rot.x, ref_rot.y, ref_rot.z, -ref_rot.w]
            tf_pos = self.qv_mult(qua, vec)

            ref_qua = [ref_rot.x, ref_rot.y, ref_rot.z, ref_rot.w]
            self_qua = [rot.x, rot.y, rot.z, -rot.w]
            rot = Quaternion()

            result = tf.transformations.quaternion_multiply(self_qua, ref_qua)
            q = tf.transformations.quaternion_from_euler(0, 0, 1.5707)
            result = tf.transformations.quaternion_multiply(result, q)
            rot.x, rot.y, rot.z, rot.w = result

            x = tf_pos[0] + ref_pos.x
            y = -tf_pos[1] + ref_pos.y

            if (idx == self.current_measure_bot and 
                frame_id == self.current_measure_tower):
                self.measure_bot_simple_loc = (
                    x, y, self.quat2angle(rot))

            x -= self.tower_offset[frame_id].value[0]
            y -= self.tower_offset[frame_id].value[1]
            
            if should_use:
                self.latest_tag_stamp = new_timestamp

                self.pub_marker.publish(self.prep_marker(
                    "duckiebot", x, y, rot, idx))
                bot_loc = Float64MultiArray()
                stmp = rospy.get_rostime()
                bot_loc.data = [float(idx), x, y, self.quat2angle(rot), stmp.secs, stmp.nsecs]
                self.pub_bot_loc.publish(bot_loc)


    def simple_vs_loc_after_correction(self):
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
        if (not self.measure_bot_loc or not self.measure_bot_simple_loc):
            return (100.0, 100.0, 100.0)
        return (
            self.measure_bot_simple_loc[0] - self.measure_bot_loc[0],
            self.measure_bot_simple_loc[1] - self.measure_bot_loc[1],
            self.measure_bot_simple_loc[2] - self.measure_bot_loc[2],
        )

    def quat2angle(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return (180.0/math.pi * math.atan2(siny_cosp, cosy_cosp))


    def cb_markers(self, msg):
        """
        Callback when receiving localization marker results
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
                # print(
                #     "TOWER[{}] \n\tx: {}\n\ty: {}\n\tz: {}\n\tangle: {}".format(
                #         idx, p.x, p.y, p.z, r # self.quat2angle(r)
                #     )
                # )
                # if self.parameters['~measuring_offset'] and idx in self.tower_offset_measuring_set:
                #     self.pub_marker.publish(self.prep_marker(
                #         "watchtower", p.x, p.y, r, idx
                #     ))
                # else:
                #     self.pub_marker.publish(self.prep_marker(
                #         "watchtower", -10, -10, r, idx
                #     ))

                # highlight ones being measured
                if self.parameters['~measuring_offset'] and idx == self.current_measure_tower:
                    self.pub_marker.publish(self.prep_marker(
                        "watchtower", p.x, p.y, r, idx
                    ))
                else:
                    if idx in self.measured_towers:
                        self.pub_marker.publish(self.prep_marker(
                            "watchtower", p.x, p.y, r, idx, True
                        ))
                    else:
                        self.pub_marker.publish(self.prep_marker(
                            "watchtower", -10, -10, r, idx
                        ))

            if m.ns == "duckiebots":
                idx = m.id
                if idx == self.current_measure_bot:
                    self.measure_bot_loc = (
                        m.pose.position.x,
                        m.pose.position.y,
                        self.quat2angle(m.pose.orientation)
                    )
                    if self.parameters['~measuring_offset']:
                        if self.current_measure_tower in self.tower_offset:
                            self.simple_vs_loc_after_correction()

        if self.parameters['~measuring_offset']:
            if self.current_measure_tower == self.parameters["~offset_tower_id"]:
                if self.current_measure_tower in self.tower_offset:
                    # show which towers are going through the measurement
                    self.tower_offset[self.current_measure_tower].update(
                        self.offset_simple_vs_loc(),
                    )
            else:
                if self.current_measure_tower in self.tower_offset:
                    self.tower_offset[self.current_measure_tower].useIfGood()
                    self.measured_towers.add(self.current_measure_tower)
                    self.measure_bot_simple_loc = None
                self.current_measure_tower = self.parameters["~offset_tower_id"]
        else:
            if self.current_measure_tower in self.tower_offset:
                self.tower_offset[self.current_measure_tower].useIfGood()
                self.measured_towers.add(self.current_measure_tower)
                self.measure_bot_simple_loc = None
            self.current_measure_tower = -1
            rospy.set_param("/simple_localization/loc_node/offset_tower_id", -1)


if __name__ == '__main__':
    # create the node
    node = LocalizationNode(node_name='loc_node')
    # keep spinning
    rospy.spin()