#!/usr/bin/env python

import rospy
import tf_conversions
import tf2_ros

from openvr_ros.msg import TrackedDevicePose
from openvr_ros.msg import TrackedDeviceClass, TrackedDeviceRole, TrackedDeviceResult

from geometry_msgs.msg import TransformStamped


class Tracked2TF:
    def __init__(self, pose_topic):
        rospy.Subscriber(pose_topic, TrackedDevicePose, self.callback, queue_size=300)
        self.br = tf2_ros.TransformBroadcaster()

        self.device_classes = ["invalid", "hmd", "controller", "tracker", "reference", "display"]
        # Keeps track of a persistent name for a device_id
        self.registered_devices = {}
        # The "key" of this dict is the device_class.
        # This dict keeps track of the number of devices in each device_class.
        self.count = {key: 0 for key in self.device_classes}

    def register(self, ID, device_class):
        if not ID in self.registered_devices:
            self.count[device_class] += 1
            device_name = "vr_" + device_class + "_" + str(self.count[device_class])
            self.registered_devices[ID] = device_name
            rospy.logwarn("Just registered a device of class '{}' with ID {}. Naming it: '{}'".format(device_class, ID, device_name))
        return self.registered_devices[ID]

    def callback(self, data):
        # 0. Register the role and class of the object
        # 1. Make the name of the frame from its class and its role.
        # 2. If TrackingResult, PoseIsValid, and DeviceIsConnected are in good status then pblish the device position.

        # "data" type: TrackedDevicePose()
        tf_msg = TransformStamped()

        device_class = self.device_classes[data.device_header.Class]
        frame_child = self.register(data.device_header.ID, device_class)

        tf_msg.header.stamp = data.header.stamp
        tf_msg.header.frame_id = "vr_link"
        tf_msg.child_frame_id = frame_child
        #data.pose.position.z = -data.pose.position.z
        tf_msg.transform.translation = data.pose.position
        tf_msg.transform.rotation = data.pose.orientation
        self.br.sendTransform(tf_msg)


if __name__=="__main__":
    rospy.init_node('tracked2tf', anonymous=True)
    pose_topic = rospy.get_param("~pose_topic")
    Tracked2TF(pose_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS VR tracker module")
