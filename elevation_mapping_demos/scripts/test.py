#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException

class TfToPosePublisher:
    def __init__(self):
        rospy.init_node('tf_to_pose_publisher')

        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        self.source_frame = rospy.get_param('~source_frame', 'odom')
        pose_name = self.target_frame + '_pose'

        self.tf_listener = TransformListener()
        self.publisher = rospy.Publisher(pose_name, PoseWithCovarianceStamped, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.callback)

    def callback(self, event):
        try:
            # 요청 시간 변경: 최신 데이터를 요청
            now = rospy.Time(0)  # 요청 시간을 최신으로 설정
            self.tf_listener.waitForTransform(self.source_frame, self.target_frame, now, rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform(self.source_frame, self.target_frame, now)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logwarn(f"Transform not available: {str(e)}. Source: {self.source_frame}, Target: {self.target_frame}")
            return

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.source_frame
        pose.pose.pose.position.x = trans[0]
        pose.pose.pose.position.y = trans[1]
        pose.pose.pose.position.z = trans[2]
        pose.pose.pose.orientation.x = rot[0]
        pose.pose.pose.orientation.y = rot[1]
        pose.pose.pose.orientation.z = rot[2]
        pose.pose.pose.orientation.w = rot[3]

        pose.pose.covariance = [0.0] * 36

        self.publisher.publish(pose)


def main():
    node = TfToPosePublisher()
    rospy.spin()

if __name__ == '__main__':
    main()
