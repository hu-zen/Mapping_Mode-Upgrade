# FILE: depth_median_filter.py (reuse, included for completeness) autonomus_mobile_robot/scripts/
if len(self.buf) > self.bufsize:
self.buf.pop(0)
if len(self.buf) == self.bufsize:
m = np.median(np.stack(self.buf, axis=0), axis=0).astype(np.float32)
out = self.bridge.cv2_to_imgmsg(m, encoding='32FC1')
out.header = msg.header
self.pub.publish(out)


if __name__ == '__main__':
try:
MedianDepth()
except rospy.ROSInterruptException:
pass




# FILE: odom_bias_correction.py
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import time


"""
Simple odom spike suppressor.
Monitors /odom (wheel odom) and /vo_filtered (visual odom). If during
high angular velocity the wheel odom reports a sudden backward spike
(below lin_spike_thresh), the node will hold (re-publish previous odom)
for a short duration to avoid feeding spike into ekf.
Publishes corrected odometry to /odom_data_quat (used by ekf input).
"""


class OdomBiasCorrector:
def __init__(self):
rospy.init_node('odom_bias_correction')
self.ang_thresh = rospy.get_param('~ang_thresh', 0.6)
self.lin_spike_thresh = rospy.get_param('~lin_spike_thresh', -0.05)
self.hold_duration = rospy.get_param('~hold_duration', 0.4)


self.pub = rospy.Publisher('/odom_data_quat', Odometry, queue_size=5)
self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb)
self.last_odom = None
self.hold_until = 0


def odom_cb(self, msg):
now = time.time()
lw = msg.twist.twist.angular.z
lv = msg.twist.twist.linear.x
if self.last_odom is None:
self.last_odom = msg
self.pub.publish(msg)
return


# detect backward spike during turning
if abs(lw) > self.ang_thresh and lv < self.lin_spike_thresh and now < self.hold_until + self.hold_duration:
# If we're already holding, continue to publish last safe odom
rospy.logwarn_throttle(2, 'odom_bias_correction: holding odom to avoid backward spike')
out = self.last_odom
out.header.stamp = rospy.Time.now()
self.pub.publish(out)
self.hold_until = now
return


if abs(lw) > self.ang_thresh and lv < self.lin_spike_thresh:
# first detection, set hold
rospy.logwarn('odom_bias_correction: detected backward spike during turn, holding odom for %.2fs', self.hold_duration)
out = self.last_odom
out.header.stamp = rospy.Time.now()
self.pub.publish(out)
self.hold_until = now
return


# normal case
self.last_odom = msg
msg.header.stamp = rospy.Time.now()
self.pub.publish(msg)


if __name__ == '__main__':
try:
OdomBiasCorrector()
rospy.spin()
except rospy.ROSInterruptException:
pass
