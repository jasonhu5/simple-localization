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
    BoolStamped, Twist2DStamped, FSMState, AprilTagExtended
)
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
# from simple_map import SimpleMap


class LocalizationNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)

        # List of current watchtowers 
        self.tower_list = dict()

        ## parameters
        self.parameters['~tag_ttl'] = None
        self.updateParameters()

        # Subscribe to topics online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cb_markers, queue_size=30)


        self.sub_tags = rospy.Subscriber(
            "/poses_acquisition/poses",
            AprilTagExtended,
            self.cb_tags,
            queue_size=1,
        )

        self.pub_marker = rospy.Publisher("/simple_loc", Marker, queue_size=10)
        self.pub_bot_loc = rospy.Publisher(
            "/simple_loc_bots",
            Float64MultiArray,
            queue_size=10,
        )

        # build simpleMap
        # map_file_path = os.path.join(
        #     "/code/catkin_ws/src",
        #     "simple-localization/packages/localization/src",
        #     "test_map.yaml"
        # )
        # self.map = SimpleMap(map_file_path)
        # self.map.display_raw_debug()


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


    def prep_marker(self, ns, x, y, q, idx):
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.id = idx 
        marker.ns = "simple_loc"

        marker.type = Marker.ARROW if ns == "duckiebot" else Marker.CUBE
        # marker.type = Marker.ARROW if ns == "duckiebot" else Marker.ARROW
        # marker.type = Marker.CUBE if ns == "duckiebot" else Marker.ARROW
        marker.action = marker.ADD

        marker.scale = Vector3(0.15, 0.06, 0.06) if ns == "duckiebot" else Vector3(0.03, 0.03, 0.3)
        # marker.scale = Vector3(0.1, 0.03, 0.03) if ns == "duckiebot" else Vector3(0.1, 0.03, 0.03)
        # marker.scale = Vector3(0.1, 0.1, 0.1) if ns == "duckiebot" else Vector3(0.1, 0.03, 0.03)
        marker.color = ColorRGBA(1, 0, 0, 1) if ns == "duckiebot" else ColorRGBA(0, 1, 0, 0.7)

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        marker.pose.orientation = q

        return marker


    def updateSubscriberPublisher(self, tower_id):
        '''
        Updates Subscriber and Publisher dictionaries, if new watchtower is added
        Input: tower_id (int)
        '''
        # TODO
        pass


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

        # TODO: add other bots
        # testing, autobot27
        if msg.tag_id == 426:
            # don't use too old data
            tag_time_since_created = rospy.Time.now().to_sec() - msg.header.stamp.secs
            # print(tag_time_since_created, self.parameters['~tag_ttl'])
            if tag_time_since_created > self.parameters['~tag_ttl']:
                return
            # print("using")

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
            #     msg.tag_id,
            #     pos.x,
            #     pos.y,
            #     # self.quat2angle(rot)
            #     rot
            # ))
            # onRoad = self.map.position_on_map((x, y), True)
            self.pub_marker.publish(self.prep_marker(
                "duckiebot", x, y, rot, msg.tag_id))
            bot_loc = Float64MultiArray()
            bot_loc.data = [float(msg.tag_id), x, y, self.quat2angle(rot)]
            self.pub_bot_loc.publish(bot_loc)


    def quat2angle(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return (180.0/math.pi * math.atan2(siny_cosp, cosy_cosp))


    def cb_markers(self, msg):
        """
        Callback when receiving localization marker results
        """

        for m in msg.markers:
            # care only about duckiebots
            if m.ns != "watchtowers":
                continue

            idx = m.id
            if not idx in self.tower_list:
                self.tower_list[idx] = m
                p = m.pose.position
                r = m.pose.orientation
                print(
                    "TOWER[{}] \n\tx: {}\n\ty: {}\n\tz: {}\n\tangle: {}".format(
                        idx, p.x, p.y, p.z, r # self.quat2angle(r)
                    )
                )
                self.pub_marker.publish(self.prep_marker(
                    "watchtower", p.x, p.y, r, idx
                ))
            # else:
            #     self.tower_list[idx] = m
            #     p = m.pose.position
            #     r = m.pose.orientation
            #     self.pub_marker.publish(self.prep_marker(
            #         "watchtower", p.x, p.y, r, idx
            #     ))


if __name__ == '__main__':
    # create the node
    node = LocalizationNode(node_name='loc_node')
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()