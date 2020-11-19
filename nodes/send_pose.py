#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose

if __name__ == '__main__':
    rospy.init_node('pose_array')
    r = rospy.Rate(60.0)
    pub = rospy.Publisher("/poseArrayTopic", PoseArray)
    while not rospy.is_shutdown():
        poseArray = PoseArray()
        poseArray.header.stamp = rospy.Time.now()
        poseArray.header.frame_id = "/camera_rgb_optical_frame"
        for i in range(1, 5):
            somePose = Pose()
            somePose.position.x = 0.0
            somePose.position.y = 0.0
            somePose.position.z = i

            somePose.orientation.x = 0.0
            somePose.orientation.y = 0.0
            somePose.orientation.z = 0.0
            somePose.orientation.w = 1.0

            poseArray.poses.append(somePose)

        pub.publish(poseArray)
        r.sleep()