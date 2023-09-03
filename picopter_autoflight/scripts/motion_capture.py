#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

vicon_msg = TransformStamped()

def callback(sub_msg):
    global vicon_msg
    vicon_msg = sub_msg

if __name__ == "__main__":

    rospy.init_node("motion_capture")

    sub_pose  = rospy.Subscriber('vicon/picopter/picopter', TransformStamped, callback = callback)

    pub_pose  = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size = 10)

    mocap_msg = PoseStamped()

    rate      = rospy.Rate(50)

    while (not rospy.is_shutdown()):

        mocap_msg.header.stamp.secs  = vicon_msg.header.stamp.secs
        mocap_msg.header.stamp.nsecs = vicon_msg.header.stamp.nsecs
        mocap_msg.header.frame_id    = vicon_msg.header.frame_id
        mocap_msg.pose.position.x    = vicon_msg.transform.translation.x
        mocap_msg.pose.position.y    = vicon_msg.transform.translation.y
        mocap_msg.pose.position.z    = vicon_msg.transform.translation.z
        mocap_msg.pose.orientation.x = vicon_msg.transform.rotation.x
        mocap_msg.pose.orientation.y = vicon_msg.transform.rotation.y
        mocap_msg.pose.orientation.z = vicon_msg.transform.rotation.z
        mocap_msg.pose.orientation.w = vicon_msg.transform.rotation.w

        pub_pose.publish(mocap_msg)

        rate.sleep()
