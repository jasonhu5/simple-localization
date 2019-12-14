#!/usr/bin/env python

import cv2
import os
import rospy
import tf
import math
import subprocess
from duckietown import DTROS
from std_msgs.msg import String, ColorRGBA, Float64MultiArray
from duckietown_msgs.msg import (
    BoolStamped, Twist2DStamped, FSMState, AprilTagExtended, WheelsCmdStamped
)
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from simple_map import SimpleMap

class TowerOffset():
    def __init__(self):
        self.counter = 0 
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
        self.parameters['~offset_delay_counter_thres'] = None
        self.parameters['~measuring_offset'] = None
        self.parameters['~offset_debug_print'] = None
        # self.parameters['~offset_debug_check_tower'] = None
        self.parameters['~offset_tower_id'] = None
        self.parameters['~offset_bot_id'] = None
        # self.parameters['~measure_exclude_tower'] = None
        self.parameters['~onroad_debug_print'] = None
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
        # # TODO: other bots
        # self.sub_car_cmd = rospy.Subscriber(
        #     "/autobot26/wheels_driver_node/wheels_cmd_decalibrated",
        #     WheelsCmdStamped,
        #     self.cb_car_cmd,
        #     queue_size=5
        # )

        # # PUBLISH
        self.pub_marker = rospy.Publisher("/simple_loc", Marker, queue_size=10)
        self.pub_bot_loc = rospy.Publisher(
            "/simple_loc_bots",
            Float64MultiArray,
            queue_size=10,
        )

        # build simpleMap
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "simple-localization/packages/localization/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)
        self.map.display_raw_debug()

        # tag <=> bot
        tag_bot_mapping_file = os.path.join(
            "/code/catkin_ws/src",
            "simple-localization/packages/localization/src",
            "bot_tag_mapping"
        )
        self.tag2bot = self.read_bot_tag_mapping(tag_bot_mapping_file)

        self.measure_bot_loc = None 
        # self.measure_bot_simple_loc = {}
        self.measure_bot_simple_loc = None
        self.current_measure_bot = self.parameters["~offset_bot_id"]
        self.current_measure_tower = self.parameters["~offset_tower_id"]
        # self.measure_tower_black_list = set([])
        self.measured_towers = set([])

        # which tag info to use
        self.latest_tag_stamp = rospy.Time.now().to_sec()

        # self.bot_last_seen = None
        # self.bot_past_cmds = []
        # self.last_cmd = None
        # self.last_cmd_time = None


    def read_bot_tag_mapping(self, file_path):
        with open(file_path) as f:
            lines = f.read().split("\n")
            data = [l.split(':') for l in lines if any(char.isdigit() for char in l)]
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
        # marker.type = Marker.ARROW if ns == "duckiebot" else Marker.ARROW
        # marker.type = Marker.CUBE if ns == "duckiebot" else Marker.ARROW
        marker.action = marker.ADD

        # marker.scale = Vector3(0.15, 0.06, 0.06) if ns == "duckiebot" else Vector3(0.03, 0.03, 0.3)
        marker.scale = Vector3(0.15, 0.06, 0.06) if ns == "duckiebot" else Vector3(0.1, 0.1, 2)
        # marker.scale = Vector3(0.1, 0.03, 0.03) if ns == "duckiebot" else Vector3(0.1, 0.03, 0.03)
        # marker.scale = Vector3(0.1, 0.1, 0.1) if ns == "duckiebot" else Vector3(0.1, 0.03, 0.03)
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
        header:
            seq: 157382
            stamp:
                secs: 1575978474
                nsecs: 262840986
            frame_id: "watchtower107"
        transform:
            translation:
                x: 0.279585103715
                y: -0.474503871831
                z: 0.86165734889
            rotation:
                x: -0.186639306675
                y: 0.049473187544
                z: 0.2191318536
                w: 0.956399186353
        tag_id: 336
        tag_family: "tag36h11"
        hamming: 0
        decision_margin: 67.1385879517
        homography: [9.550640106201172, -14.088746070861816, 765.119140625, 2.9025955200195312, 6.868178367614746, 261.6824035644531, -0.006694934796541929, -0.01182014774531126, 1.0]
        center: [765.119140625, 261.6824035644531]
        corners: [745.299560546875, 267.0165100097656, 774.9288940429688, 276.573974609375, 784.736572265625, 256.4027099609375, 755.666015625, 247.33226013183594]
        pose_error: 2.89966184397e-09
        """
        frame_id = int(msg.header.frame_id[-3:])

        idx = self.tag2bot.get(msg.tag_id)

        if idx:

            # if tower see's a bot in measuring mode, note down
            # if idx == self.current_measure_bot:
            #     if self.parameters['~measuring_offset']:
            #         if frame_id not in self.measure_tower_black_list:
            #             self.tower_offset_measuring_set.add(frame_id)

            # TIME TO LIVE
            # # don't use too old data
            tag_time_since_created = rospy.Time.now().to_sec() - msg.header.stamp.secs
            # print(tag_time_since_created, self.parameters['~tag_ttl'])
            if tag_time_since_created > self.parameters['~tag_ttl']:
                return
            # # print("using")

            # TODO: this should be bot wise
            new_timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 10 ** -9
            time_diff = new_timestamp - self.latest_tag_stamp
            # should_use = frame_id in self.measured_towers and (
            #     time_diff > 0 or abs(time_diff) < self.parameters['~tag_ttl'])
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
            # x = pos.x
            # y = pos.y

            ref_qua = [ref_rot.x, ref_rot.y, ref_rot.z, ref_rot.w]
            self_qua = [rot.x, rot.y, rot.z, -rot.w]
            rot = Quaternion()
            # result = tf.transformations.quaternion_multiply(ref_qua, self_qua)
            result = tf.transformations.quaternion_multiply(self_qua, ref_qua)
            q = tf.transformations.quaternion_from_euler(0, 0, 1.5707)
            result = tf.transformations.quaternion_multiply(result, q)
            rot.x, rot.y, rot.z, rot.w = result

            x = tf_pos[0] + ref_pos.x
            y = -tf_pos[1] + ref_pos.y

            # print("tag [{}]: \n\tx: {}\n\ty: {}\n\tq: {}".format(
            #     idx,
            #     pos.x,
            #     pos.y,
            #     # self.quat2angle(rot)
            #     rot
            # ))
            onRoad = self.map.position_on_map((x, y), True)
            if self.parameters['~onroad_debug_print']:
                print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> [{}] On road: {}".format(
                    idx, onRoad))
            if (idx == self.current_measure_bot and 
                frame_id == self.current_measure_tower):
                self.measure_bot_simple_loc = (
                    x, y, self.quat2angle(rot))
                # self.measure_bot_simple_loc[frame_id] = (
                #     x, y, self.quat2angle(rot))
                # self.simple_vs_loc_after_correction()

            x -= self.tower_offset[frame_id].value[0]
            y -= self.tower_offset[frame_id].value[1]
            
            if should_use:
                # cut_index = math.ceil(time_diff * 20)
                # self.bot_past_cmds = self.bot_past_cmds[cut_index:]
                self.latest_tag_stamp = new_timestamp

                self.pub_marker.publish(self.prep_marker(
                    "duckiebot", x, y, rot, idx))
                bot_loc = Float64MultiArray()
                stmp = rospy.get_rostime()
                bot_loc.data = [float(idx), x, y, self.quat2angle(rot), stmp.secs, stmp.nsecs]
                self.pub_bot_loc.publish(bot_loc)

                # self.bot_last_seen = (x, y, rot)
                # self.bot_integration = 0.0

                # print('using info from', frame_id)
            # else:
            #     print('discarding from', frame_id)


    def simple_vs_loc_after_correction(self):
        if (not self.measure_bot_loc or not self.measure_bot_simple_loc):
            return
        # if (not self.measure_bot_loc or not len(self.measure_bot_simple_loc)):
        #     return
        # print("[S] {}".format(self.measure_bot_simple_loc))
        # print("[L] {}".format(self.measure_bot_loc))
        if self.parameters['~offset_debug_print']:
            # show_tower = self.parameters["~offset_debug_check_tower"]
            # tower_checking_set = set([show_tower]) if (
            #     show_tower in self.tower_offset_measuring_set
            # ) else self.tower_offset_measuring_set
            # for tower in tower_checking_set:
            #     offset = self.offset_simple_vs_loc(tower)
            #     print("[Diff {}] \nx: {}\ny: {}\nz: {}".format(
            #         tower, offset[0], offset[1], offset[2]))
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
        # if (not self.measure_bot_loc or not len(self.measure_bot_simple_loc)):
        #     return None
        return (
            # self.measure_bot_simple_loc[tower][0] - self.measure_bot_loc[0],
            # self.measure_bot_simple_loc[tower][1] - self.measure_bot_loc[1],
            # self.measure_bot_simple_loc[tower][2] - self.measure_bot_loc[2],
            self.measure_bot_simple_loc[0] - self.measure_bot_loc[0],
            self.measure_bot_simple_loc[1] - self.measure_bot_loc[1],
            self.measure_bot_simple_loc[2] - self.measure_bot_loc[2],
        )

    def quat2angle(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return (180.0/math.pi * math.atan2(siny_cosp, cosy_cosp))


    # def cb_car_cmd(self, msg):
    #     # self.bot_integration += msg.vel_right * 0.04

    #     # cmd_stamp = msg.header.stamp.

    #     # lag = msg.header.stamp.secs - self.latest_tag_stamp
    #     # if lag > 1.0:
    #     # publish lastseen + integration
    #     if self.bot_last_seen:
    #         x = self.bot_last_seen[0]
    #         y = self.bot_last_seen[1]
    #         rot = self.bot_last_seen[2]
    #         # y += self.bot_integration
    #         self.pub_marker.publish(self.prep_marker(
    #             "duckiebot", x, y, rot, 26, True))
    #     # else:
    #     #     # clear integration
    #     #     self.bot_integration = 0 

    #     # if msg.vel_left != 0 or msg.vel_right != 0:
    #     #     lag = msg.header.stamp.secs - self.latest_tag_stamp
    #     #     if lag > 1.0:
    #     #         pass
    #     #     else:
    #     #         self.bot_predict = self.bot_last_seen
        
    #     # self.last_cmd = (msg.vel_left, msg.vel_right)
    #     # self.last_cmd_time = (msg.header.stamp.secs, msg.header.stamp.nsecs)


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

        # # exclude unwanted towers
        # self.measure_tower_black_list.add(
        #     self.parameters["~measure_exclude_tower"])
        # for excluded_tower in self.measure_tower_black_list:
        #     self.tower_offset_measuring_set.discard(excluded_tower)

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

        # if self.parameters['~measuring_offset']:
        #     # show which towers are going through the measurement
        #     print("Measuring offset for {}".format(
        #         self.tower_offset_measuring_set))
        #     for tower in self.tower_offset_measuring_set:
        #         self.tower_offset[tower].update(
        #             self.offset_simple_vs_loc(tower),
        #             self.parameters['~offset_delay_counter_thres'],
        #         )
        # else:
        #     for tower in self.tower_offset_measuring_set:
        #         self.tower_offset[tower].useIfGood()
        #     self.measure_tower_black_list = set([])
        #     self.tower_offset_measuring_set = set([])
        if self.parameters['~measuring_offset']:
            if self.current_measure_tower == self.parameters["~offset_tower_id"]:
                if self.current_measure_tower in self.tower_offset:
                    # show which towers are going through the measurement
                    # print("Measuring offset for {}".format(self.current_measure_tower))
                    self.tower_offset[self.current_measure_tower].update(
                        self.offset_simple_vs_loc(),
                    )
                    # print(self.tower_offset[self.current_measure_tower].candidate)
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
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()