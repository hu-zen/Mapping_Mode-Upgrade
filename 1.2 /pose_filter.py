#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

alpha = 0.7  # smoothing strength

last_pose = None

def callback(msg):
    global last_pose
    if last_pose is None:
        last_pose = msg
    else:
        # smooth position
        msg.pose.pose.position.x = alpha * last_pose.pose.pose.position.x + (1-alpha) * msg.pose.pose.position.x
        msg.pose.pose.position.y = alpha * last_pose.pose.pose.position.y + (1-alpha) * msg.pose.pose.position.y
        msg.pose.pose.orientation = last_pose.pose.pose.orientation
        last_pose = msg
    pub.publish(msg)

rospy.init_node('pose_filter')
pub = rospy.Publisher('/amcl_pose_filtered', PoseWithCovarianceStamped, queue_size=1)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
rospy.spin()
