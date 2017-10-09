#!/usr/bin/env python2
from __future__ import division
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import rospy
import tf2_ros # had to use tf2 because tf1 broke on me
import tf2_geometry_msgs
import tf.transformations as tf_utils

# Date:   September 2017
# Author: Matthew Wilson 

class ArucoPatchNode(object):
    """
    ROS node to take in aruco camera detections (as tfs), average them,
    and broadcast the tf from map to odom
    """
    def __init__(self):
        rospy.init_node("aruco_patch", log_level=rospy.DEBUG)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.two_d_mode = rospy.get_param("~aruco_patch/two_d_mode", True)
        self.marker_id_list = rospy.get_param("/aruco_mapping/marker_id_list")

        roll = rospy.get_param("/aruco_mapping/roll")
        pitch = rospy.get_param("/aruco_mapping/pitch")
        yaw = rospy.get_param("/aruco_mapping/yaw")
        self.rot_quat = (3.1415, pitch, yaw) # for some reason this is 0

    def ros_msg_to_list(self, msg):
        return [msg.__getattribute__(a) for a in msg.__slots__]

    def rotate_quat(self, quat):
        rot_quat = tf_utils.quaternion_from_euler(*self.rot_quat)
        rotated_quat = tf_utils.quaternion_multiply(quat, rot_quat)
        # normalize the quaternion
        rotated_quat /= tf_utils.vector_norm(rotated_quat) 
        return rotated_quat

    def run(self):
        """Run the main ros loop"""
        last_map_odom_tf = None
        #last_camera_tfs = []
        rate = rospy.Rate(10) #10 Hz looping
        while not rospy.is_shutdown():
            live_camera_tfs = []
            # list of camera_i frames that we have tf data on
            for i in range(len(self.marker_id_list)):
                frame_name = "camera_{}".format(i)
                rospy.logdebug("frame_name: "+frame_name)
                # if marker_i has been detected, check if we tf data for
                # the corresponding camera
                if self.tf_buffer._frameExists(frame_name): 
                    lookup_time = rospy.Time.now() - rospy.Duration(2.0)
                    if self.tf_buffer.can_transform('map', frame_name, lookup_time):
                        geomsg_tfstamped = self.tf_buffer.lookup_transform('map', \
                                frame_name, lookup_time)
                        live_camera_tfs.append(geomsg_tfstamped)
                        rospy.logdebug("tfstamped: {}".format(geomsg_tfstamped))
                        quat = self.ros_msg_to_list(geomsg_tfstamped.transform.rotation)
                        #rospy.logwarn("quat: {}".format(tf_utils.euler_from_quaternion(quat)))
                        fixed_quat = self.rotate_quat(quat)
                        #rospy.logwarn("fixed quat: {}".format(tf_utils.euler_from_quaternion(fixed_quat)))
                        geomsg_tfstamped.transform.rotation = Quaternion(*fixed_quat)
                        geomsg_tfstamped.child_frame_id = "camera_patch_{}".format(i)
                        # Publish out the rotated frame
                        self.tf_broadcaster.sendTransform(geomsg_tfstamped)
                        #last_camera_tfs = live_camera_tfs
                    else:
                        rospy.logdebug("No markers tfs")

            lookup_time = rospy.Time.now() - rospy.Duration(0.1)
            if self.tf_buffer.can_transform('odom', 'ZED_left_camera', lookup_time):
                odom_zed_tf = self.tf_buffer.lookup_transform('odom', 'ZED_left_camera', lookup_time)
            else:
                rospy.logdebug("Can't lookup odom --> ZED_left_camera")
                if last_map_odom_tf is not None:
                    self.tf_broadcaster.sendTransform(last_map_odom_tf)
                rate.sleep()
                continue

            if len(live_camera_tfs) > 1:
                # Create 2d list of all positions 
                # (rows are data from each camera, cols are xyz)
                positions = [self.ros_msg_to_list(camera_tf.transform.translation) \
                        for camera_tf in live_camera_tfs]
                quaternions = [self.ros_msg_to_list(camera_tf.transform.rotation) \
                        for camera_tf in live_camera_tfs]
 
                pos_arr = np.array(positions)
                quat_arr = np.array(quaternions)

                pos_mean = np.mean(pos_arr, axis=0).tolist()
                #pos_cov = np.cov(pos_arr.T)
                quat_mean = np.mean(quat_arr, axis=0).tolist()
                #quat_cov = np.cov(quat_arr.T)

                odom_pos = self.ros_msg_to_list(odom_zed_tf.transform.translation)
                odom_quat = self.ros_msg_to_list(odom_zed_tf.transform.rotation)

                #translation = (pos_mean - np.array(odom_pos)).tolist()

                map_tf = tf_utils.concatenate_matrices(tf_utils.translation_matrix(pos_mean), tf_utils.quaternion_matrix(quat_mean))
                odom_tf = tf_utils.concatenate_matrices(tf_utils.translation_matrix(odom_pos), tf_utils.quaternion_matrix(odom_quat))
                inversed_transform = tf_utils.inverse_matrix(odom_tf)

                out_tf = map_tf.dot(inversed_transform)

                translation = tf_utils.translation_from_matrix(out_tf)
                quaternion = tf_utils.quaternion_from_matrix(out_tf)

                #odom_quat_mat = tf_utils.quaternion_matrix(odom_quat)
                #map_quat_mat = tf_utils.quaternion_matrix(quat_mean)
                #inv_oqm = tf_utils.inverse_matrix(odom_quat_mat)
                #inv_mqm = tf_utils.inverse_matrix(map_quat_mat)

                #out_quat_mat = map_quat_mat.dot(inv_oqm)
                #out_quat_mat = odom_quat_mat.dot(inv_mqm)
                #quaternion = tf_utils.quaternion_from_matrix(out_quat_mat)

                pack_header = Header(0, rospy.Time.now(), 'map')
                pack_vec3 = Vector3(*translation)
                if self.two_d_mode:
                    pack_vec3.z = 0.0  # set z tf to 0
                    # Set roll and pitch to 0, only leaving yaw
                    euler = tf_utils.euler_from_quaternion(quaternion)
                    quaternion = tf_utils.quaternion_from_euler(0, 0, euler[2])
                    
                pack_quat = Quaternion(*quaternion)
                pack_tf = Transform(pack_vec3, pack_quat)
                rospy.logwarn("tf changed")
                last_map_odom_tf = TransformStamped(pack_header, 'odom', pack_tf)
            else:
                rospy.logdebug("No markers detected")

            if last_map_odom_tf is not None:
                self.tf_broadcaster.sendTransform(last_map_odom_tf)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = ArucoPatchNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting aruco_patch node")
