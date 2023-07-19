#! /usr/bin/env python3
# -*- coding: utf-8 -*-
from time import sleep
import rospy
import cv2
#import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16, Int32
#from playsound import playsound

class ucar_nav:
    def __init__(self):
        rospy.init_node('MultiNav', anonymous=True)  # 创建了一个节点

        # 取得菜品标志位
        self.flag = -1

        #是否到达
        self.arrive = 0

        # 订阅二维码识别的结果
        rospy.Subscriber("/qr_res", Int16, self.qrcallback)
        #初始化坐标点
        self.locations = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]

        # 任务点
        self.locations[0][0] = rospy.get_param("~point_1_x", 4.991)
        self.locations[0][1] = rospy.get_param("~point_1_y", -0.230)
        self.locations[0][2] = rospy.get_param("~point_1_z", 0.99999077)
        self.locations[0][3] = rospy.get_param("~point_1_w", 0.00429631)

        # 1点
        self.locations[1][0] = rospy.get_param("~point_2_x", 2.033)
        self.locations[1][1] = rospy.get_param("~point_2_y", -0.262)
        self.locations[1][2] = rospy.get_param("~point_2_z", 1.000)
        self.locations[1][3] = rospy.get_param("~point_2_w", -0.005)

        self.locations[1][0] = rospy.get_param("~point_3_x", 2.344)
        self.locations[1][1] = rospy.get_param("~point_3_y", 4.944)
        self.locations[1][2] = rospy.get_param("~point_3_z", 0.704)
        self.locations[1][3] = rospy.get_param("~point_3_w", 0.710)


        self.new_process = mp.Process(target=self.nav(), name="move")
        self.new_process.start()


    # 发布目标点函数
    def move(self):
        # 设置目标点
        self.send_goal(0)
        if self.arrive == 1:
            self.send_goal(1)


    def send_goal(self,num):
        # 调用发布目标点函数
   # 订阅move_base服务
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # 等待服务
        while self.move_base.wait_for_server(rospy.Duration(5)) == 0:
            rospy.loginfo("Connected to move base server")
        rospy.loginfo("准备发布目标点")
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = self.locations[num][0]
        self.goal.target_pose.pose.position.y = self.locations[num][1]
        self.goal.target_pose.pose.position.z = 0
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = self.locations[num][2]
        self.goal.target_pose.pose.orientation.w = self.locations[num][3]
        # 前往下一个目标
        self.move_base.send_goal(self.goal)

        # 设置一个运行限制时间
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(100))

        # 查看是否成功到达
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                self.arrive = 1
            else:
                rospy.loginfo("Goal failed！")

    def qrcallback(self,ids):      
        self.flag = 0
        self.flag = ids.data
        if self.flag == 0:
            playsound('res/jia.wav')
        if self.flag == 1:
            playsound('')
        elif self.flag == (''):
            playsound('')
        # 取到菜品标志位置1以后才开始处理二维码识别结果
        # 防止车子还没到领取区域就已经识别到了
        # data<3是防止误识别，项目提供的二维码是0 1 2
       
  

if __name__ == '__main__':
	try:
		# 创建节点
		rospy.init_node("ucar_nav")
		ucar_nav()
		rospy.spin()
	except rospy.is_shutdown():
		print("exit")
