#!/usr/bin/env python

import rospy, math
from teb_local_planner.msg import ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32

import tf

def publish_obstacle_msg():

	#pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleMsg, queue_size=1)
	pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleMsg, queue_size=1)

	rospy.init_node("publish_obstacles")

	obstacle_msg = ObstacleMsg()
	obstacle_msg.header.stamp = rospy.Time.now()
	obstacle_msg.header.frame_id = "map"

	# obstacle_msg.obstacles.append(PolygonStamped())
	# obstacle_msg.obstacles[0].polygon.points = [Point32()]
	# obstacle_msg.obstacles[0].polygon.points[0].x = 0
	# obstacle_msg.obstacles[0].polygon.points[0].y = 0
	# obstacle_msg.obstacles[0].polygon.points[0].z = 0


	v1_x = -0.25
	v1_y = -0.25
	v2_x = 0.25
	v2_y = -0.25
	v3_x = 0.25
	v3_y = 0.25
	v4_x = -0.25
	v4_y = 0.25

	print "I am INNNNN!"

	obstacle_msg.obstacles.append(PolygonStamped())
	# v1 = Point32()
	# v1.x = -0.25
	# v1.y = -0.25
	#
	# v2 = Point32()
	# v2.x = 0.25
	# v2.y = -0.25
	#
	# v3 = Point32()
	# v3.x = 0.25
	# v3.y = 0.25
	#
	# v4 = Point32()
	# v4.x = -0.25
	# v4.y = 0.25
	obstacle_msg.obstacles[0].polygon.points = [Point32()]
	obstacle_msg.obstacles[0].polygon.points[0].x = 0
	obstacle_msg.obstacles[0].polygon.points[0].y = 0
	obstacle_msg.obstacles[0].polygon.points[0].z = 0
	# obstacle_msg.obstacles[0].polygon.points = [v1, v2, v3, v4]



	r = rospy.Rate(10) # 10hz
	t = 0.0

	listener = tf.TransformListener()

	while not rospy.is_shutdown():
	# Vary y component of the point obstacle
		try:
			rospy.loginfo('tf reached')
			(trans,rot) = listener.lookupTransform('map', 'ak1_base_link', rospy.Time(0))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		# obstacle_msg.obstacles[0].polygon.points[0].y = 1*math.sin(t)
		obstacle_msg.obstacles[0].polygon.points[0].x = trans[0]
		obstacle_msg.obstacles[0].polygon.points[0].y = trans[1]
		obstacle_msg.obstacles[0].polygon.points[0].z = trans[2]

		# obstacle_msg.obstacles[0].polygon.points[1].x = v2_x + trans[0]
		# obstacle_msg.obstacles[0].polygon.points[1].y = v2_y + trans[1]
		# obstacle_msg.obstacles[0].polygon.points[1].z = trans[2]
		#
		# obstacle_msg.obstacles[0].polygon.points[2].x = v3_x + trans[0]
		# obstacle_msg.obstacles[0].polygon.points[2].y = v3_y + trans[1]
		# obstacle_msg.obstacles[0].polygon.points[2].z = trans[2]
		#
		# obstacle_msg.obstacles[0].polygon.points[3].x = v4_x + trans[0]
		# obstacle_msg.obstacles[0].polygon.points[3].y = v4_y + trans[1]
		# obstacle_msg.obstacles[0].polygon.points[3].z = trans[2]


		t = t + 0.1

		rospy.loginfo('publishing')
		pub.publish(obstacle_msg)
		r.sleep()


if __name__ == '__main__':
	try:
		publish_obstacle_msg()
	except rospy.ROSInterruptException:
		pass
