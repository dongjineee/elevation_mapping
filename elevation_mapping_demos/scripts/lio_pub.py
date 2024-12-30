#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def odom_callback(odom_msg):
    """Callback function to convert Odometry to PoseWithCovarianceStamped."""
    global pose_publisher

    # Convert Odometry to PoseWithCovarianceStamped
    pose_msg = PoseWithCovarianceStamped()

    # Copy header information
    pose_msg.header.stamp = odom_msg.header.stamp
    # pose_msg.header.stamp = rospy.Time.now()

    pose_msg.header.frame_id = "odom"

    # Copy pose and covariance
    pose_msg.pose.pose = odom_msg.pose.pose
    pose_msg.pose.covariance = odom_msg.pose.covariance

    # Publish the PoseWithCovarianceStamped message
    pose_publisher.publish(pose_msg)

def main():
    """Main function to initialize the node and set up subscribers and publishers."""
    global pose_publisher

    # Initialize ROS node
    rospy.init_node('lio_odom_to_pose_converter', anonymous=True)

    # Set up subscriber to LIO odometry topic
    lio_odom_topic = rospy.get_param("~lio_odom_topic", "/Odometry")
    rospy.Subscriber(lio_odom_topic, Odometry, odom_callback)

    # Set up publisher for PoseWithCovarianceStamped
    pose_topic = rospy.get_param("~pose_topic", "/lio/pose")
    pose_publisher = rospy.Publisher(pose_topic, PoseWithCovarianceStamped, queue_size=10)

    # Spin to process callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass