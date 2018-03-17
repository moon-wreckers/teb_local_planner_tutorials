#!/usr/bin/env python 
import roslib
import rospy
import math
import tf

if __name__ == '__main__':

	rospy.init_node('fixed_tf_broadcaster')
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)

	t = 0.0
	while not rospy.is_shutdown():
		br.sendTransform((0.0, math.sin(t), 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "map", "LHB_892315F9_AM")
		t = t + 0.1
		rate.sleep();