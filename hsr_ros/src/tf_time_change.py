#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf2_msgs.msg import TFMessage

class TF_Changer():

	def __init__(self):
		self.pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
		self.sub = rospy.Subscriber("/tf_sigverse", TFMessage, self.callback)
		rospy.spin()

	def callback(self, msg):
		#print msg
		send_msg = TFMessage()
		for i in range(len(msg.transforms)):
			temp = TransformStamped()
			temp.header.stamp = rospy.Time.now()
			temp.child_frame_id = msg.transforms[i].child_frame_id
			temp.transform = msg.transforms[i].transform
			send_msg.transforms.append(temp)

		self.pub.publish(send_msg)

if __name__ == '__main__':
	rospy.init_node('sigverse_tf_time_changer')
	rospy.loginfo('sigverse_tf_time_changer start')
	TF_Changer()
