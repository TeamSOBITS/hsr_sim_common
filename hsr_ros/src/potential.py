#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool,String,Float32
from sensor_msgs.msg import LaserScan
import sys

import numpy as np
import cv2
import math
#from odom_base_controller.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist



class potential_direction:
	def __init__(self):

		self.turn_speed = 20 #degree
		self.stlight_speed = 0.3 #cm

		self.turtlebot_r = 0.19#[m]

		self.angle_from_potential = Bool()
		self.order_angle = 0.0#[rad]
		self.potential_angle =0.0#[rad]
		self.potential_scara_result = 0.0#[%]
		self.DISTANCE_MAX = 1.0#[m]
		self.INRYOKU_KEISUU = 700.0#700.0 
		self.SEKIRYOKU_KEISUU = 1300.0
		self.PI = 3.1415926535
		self.m2pix = 250#pixel
		self.urg_scan_using = False#ポテンシャル法でデータを使用中かどうか

		self.img_size = self.DISTANCE_MAX * self.m2pix * 2 + 1,self.DISTANCE_MAX * self.m2pix * 2 + 1,3#ちょっと大きめに設定
		self.urg_scan = LaserScan()
		self.do_flag = False

		#self.pub = rospy.Publisher('angle_from_potential',Bool,queue_size = 1)
		self.sub_scan = rospy.Subscriber('scan', LaserScan, self.laser_callback)
		self.sub_order_angle = rospy.Subscriber('potential_order_angle', Float32, self.order_angle_callback)
		self.sub_do_flag = rospy.Subscriber('potential_do_flag', Bool, self.do_flag_callback)
		self.angle_from_potential.data = False
		#self.pub.publish(self.angle_from_potential)

		self.pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)#turtlebotを動かす


		#rospy.sleep(20)



	def laser_callback(self,data):


		self.urg_scan = data

		if self.do_flag == False:
			return

		radian = 0.0
		distance = 0.0

		object_x = 0.0
		object_y = 0.0
		order_x = math.sin(self.order_angle)* self.INRYOKU_KEISUU
		order_y = math.cos(self.order_angle)* self.INRYOKU_KEISUU
		potential_x = 0.0
		potential_y = 0.0

		rgb_color = (0,255,0)
		map_img = np.zeros(self.img_size,dtype=np.uint8)
		map_img[self.DISTANCE_MAX * self.m2pix][self.DISTANCE_MAX * self.m2pix] = tuple(reversed(rgb_color))

		count = 0
		object_count = 0
		self.urg_scan_using = True
		for i in range(0, len(self.urg_scan.ranges)):
			if(self.urg_scan.ranges[i] < self.DISTANCE_MAX):
				#if i < len(data.ranges)/2:
				count += 1

				radian = float(i - len(self.urg_scan.ranges)*0.5) / float(len(self.urg_scan.ranges)) * self.PI
				#rospy.loginfo("radian= %f",radian)
				distance = (self.DISTANCE_MAX - self.urg_scan.ranges[i]) * self.SEKIRYOKU_KEISUU # 障害物の距離を比例させる
				object_x = distance * math.sin(radian)
				object_y = distance * math.cos(radian)
				object_x_4map = 0
				object_y_4map = 0 
				object_x_4map = (self.DISTANCE_MAX - self.urg_scan.ranges[i] *math.sin(radian)) * self.m2pix
				object_y_4map = (self.DISTANCE_MAX - self.urg_scan.ranges[i] *math.cos(radian)) * self.m2pix
				#rospy.loginfo("y= %d x= %d ",object_y_4map,object_x_4map)
				map_img[object_y_4map][object_x_4map] = tuple(reversed(rgb_color))

				potential_x += (order_x - object_x)#ベクトルの合成
				potential_y += (order_y - object_y)

				#std::cout << "radian " << RAD2DEG(radian) << " radian_vbn " << RAD2DEG(radian_vbn) << " err " << RAD2DEG(abs(radian - radian_vbn)) << std::endl;
				object_count += 1

		self.urg_scan_using = False
		#rospy.loginfo("count= %d",count)

		potential_result_x = 0.0
		potential_result_y = 0.0
		potential_angle = 0.0
		if  object_count == 0:
			potential_result_x = math.sin(self.order_angle)
			potential_result_y = math.cos(self.order_angle)
			potential_angle = self.order_angle;
		else:
			potential_result_x = float(potential_x) / object_count
			potential_result_y = float(potential_y) / object_count
			potential_angle = math.atan2(potential_result_x, potential_result_y)#単位はrad


		rospy.loginfo("potential_angle= %f",float(potential_angle)/self.PI*180)

		scara = math.hypot(potential_result_x, potential_result_y)#回避ベクトルの長さ
		
		if object_count == 0:#障害物なし
			scara = self.INRYOKU_KEISUU
		
		if scara > self.INRYOKU_KEISUU:
			scara = self.INRYOKU_KEISUU

		self.potential_scara_result = scara / self.INRYOKU_KEISUU * 100#normarize		max100　//引力ベクトルの長さに対する回避ベクトルの長さの割合　＝＞速度に加味
		#rospy.loginfo("scara= %f",scara)
		rospy.loginfo("self.potential_scara_result= %f",self.potential_scara_result)

		potential_x_4map = self.DISTANCE_MAX * self.m2pix - self.potential_scara_result  * math.sin(potential_angle) * 5
		potential_y_4map = self.DISTANCE_MAX * self.m2pix - self.potential_scara_result  * math.cos(potential_angle) * 5
		cv2.line(map_img,(int(self.DISTANCE_MAX * self.m2pix),int(self.DISTANCE_MAX * self.m2pix)),(int(potential_x_4map),int(potential_y_4map)),(0,0,255),3)

		order_x_4map = (self.DISTANCE_MAX - math.sin(self.order_angle) ) * self.m2pix
		order_y_4map = (self.DISTANCE_MAX - math.cos(self.order_angle) ) * self.m2pix
		cv2.line(map_img,(int(self.DISTANCE_MAX * self.m2pix),int(self.DISTANCE_MAX * self.m2pix)),(int(order_x_4map),int(order_y_4map)),(255,0,0),2)

		cv2.circle(map_img,(int(self.DISTANCE_MAX * self.m2pix),int(self.DISTANCE_MAX * self.m2pix) + int(self.turtlebot_r * self.m2pix)),int(self.turtlebot_r * self.m2pix) ,(0,100,0),1)

		cv2.namedWindow("map_img",cv2.WINDOW_AUTOSIZE)
		cv2.imshow("map_img",map_img)
		cv2.moveWindow("map_img" ,100 ,100 )
		cv2.waitKey(23)


		send_cmd = Twist()#メッセージ変数の宣言
		send_cmd.angular.z = potential_angle#角度の指定
		send_cmd.linear.x = self.stlight_speed * self.potential_scara_result*0.01#cm 速度の指定
		self.pub_twist.publish(send_cmd)




	def order_angle_callback(self,data):
		self.order_angle = data.data#[rad]

		rospy.loginfo("################# order_angle is %f ! ",self.order_angle)



	def do_flag_callback(self,data):
		self.do_flag = data.data
		if self.do_flag == True:
			rospy.loginfo("OK, START ! ")
		else:
			rospy.loginfo("Oh, I'm waiting . ")




if __name__ == '__main__':

	rospy.init_node('potential')
	pd = potential_direction()

	rospy.loginfo("\n potential_node_started")

	rospy.spin()
		
		
