#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pcms.pytorch_models import *
from pcms.openvino_models import Yolov8, HumanPoseEstimation
import numpy as np
from geometry_msgs.msg import Twist
import math
import time
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from RobotChassis import RobotChassis
import datetime
def move(forward_speed: float = 0, turn_speed: float = 0):
    global _cmd_vel
    msg = Twist()
    msg.linear.x = forward_speed
    msg.angular.z = turn_speed
    _cmd_vel.publish(msg)

def callback_imu(msg):
    global _imu
    _imu = msg
def turn_to(angle: float, speed: float):
    global _imu
    max_speed = 0.3
    limit_time = 1
    start_time = rospy.get_time()
    while True:
        q = [
            _imu.orientation.x,
            _imu.orientation.z,
            _imu.orientation.y,
            _imu.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(q)
        e = angle - yaw
        print(yaw, e)
        if yaw < 0 and angle > 0:
            cw = np.pi + yaw + np.pi - angle
            aw = -yaw + angle
            if cw < aw:
                e = -cw
        elif yaw > 0 and angle < 0:
            cw = yaw - angle
            aw = np.pi - yaw + np.pi + angle
            if aw < cw:
                e = aw
        if abs(e) < 0.01 or rospy.get_time() - start_time > limit_time:
            break
        move(0.0, max_speed * speed * e)
        rospy.Rate(20).sleep()
    move(0.0, 0.0)


def turn(angle: float):
    global _imu
    q = [
        _imu.orientation.x,
        _imu.orientation.y,
        _imu.orientation.z,
        _imu.orientation.w
    ]
    roll, pitch, yaw = euler_from_quaternion(q)
    target = yaw + angle
    if target > np.pi:
        target = target - np.pi * 2
    elif target < -np.pi:
        target = target + np.pi * 2
    turn_to(target, 0.1)
'''
def callback_image(msg):
    global _frame
    _frame = cv2.flip(CvBridge().imgmsg_to_cv2(msg, "bgr8"),0)'''

def say(a):
    global publisher_speaker
    publisher_speaker.publish(a)

'''
def callback_depth(msg):
    global _depth
    _depth = cv2.flip(CvBridge().imgmsg_to_cv2(msg, "passthrough"),0)

def callback_imu(msg):
    global _imu
    _imu = msg

def callback_image1(msg):
    global _image1
    _image1 = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_depth1(msg):
    global _depth1
    _depth1 = CvBridge().imgmsg_to_cv2(msg, "passthrough")'''
if __name__ == "__main__":    
    rospy.init_node("demo")
    rospy.loginfo("demo node start!")
    topic_imu = "/imu/data"
    _imu=None
    rospy.Subscriber(topic_imu, Imu, callback_imu)
    rospy.wait_for_message(topic_imu, Imu)
    print("waiting imu")
    is_turning = False
    ax, ay, az, bx, by, bz = 0, 0, 0, 0, 0, 0
    pre_x, pre_z = 0.0, 0.0
    t=3.0
    cnt=1
    print("arm")
    '''
    Kinda = np.loadtxt(RosPack().get_path("mr_dnn") + "/Kinda.csv")
    dnn_yolo = Yolov8("bagv4")
    dnn_follow = Yolov8("yolov8n")
    dnn_yolo.classes = ['obj']'''
    print("run")
    chassis = RobotChassis()
    clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    rospy.sleep(1)
    '''
    pos = [[-0.281,-0.793,0],[0.331,-0.262,0],[0.828,0.295,0],[1.24,0.885,0],[1.67,1.54,0],[2.24,2.31,0],[2.95,2.84,0],[2.41,4.02,0],[1.19,4.72,0],[0.41,5.16,0],[-0.477,5.53,0],[-1.36,6.01,0],[-2.06,6.48,0],[-3.3,5.02,0],[-2.46,3.3,0],[-1.55,2.86,0],[-0.452,2.32,0],[0.541,1.75,0],[1.25,1.39,0],[-0.327,-0.807,0],[-0.909,-0.394,0],[-1.64,0.0751,0],[-2.36,0.407,0],[-3.2,0.811,0]]'''
    pos = [[-2.46,3.3,0],[-1.55,2.86,0],[-0.452,2.32,0],[0.541,1.75,0],[1.25,1.39,0],[-0.327,-0.807,0],[-0.909,-0.394,0],[-1.64,0.0751,0],[-2.36,0.407,0],[-3.2,0.811,0]]
    print("start")
    while not rospy.is_shutdown():
        for [i,j,k] in pos:
            clear_costmaps
            chassis.move_to(i,j,k)
            
            #checking
            print(cnt)
            cnt+=1
            while not rospy.is_shutdown():
                # 4. Get the chassis status.
                code = chassis.status_code
                text = chassis.status_text
                if code == 3:
                    break
        break
                    
                    
                  
            
        #say("I am back")
        key = cv2.waitKey(1)
        if key in [ord('q'), 27]:
            break




