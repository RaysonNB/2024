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
    pos=[[2.4542484283447266, 1.324235439300537, 0.00226593017578125],
     [1.1773695945739746, 1.6855344772338867, 0.00010776519775390625],
     [0.1447134017944336, 1.4833669662475586, 0.0026798248291015625],
     [-1.2144417762756348, 1.562230110168457, 0.0029201507568359375],
     [-2.094003200531006, 1.9802799224853516, 0.005820274353027344],
     [-2.7831168174743652, 0.6641445159912109, 0.0009584426879882812],
     [-2.740118980407715, -0.789093017578125, 0.0042743682861328125],
     [-2.692660331726074, -2.3315939903259277, 0.0006628036499023438],
     [-2.4997949600219727, -3.9404232501983643, 0.004380226135253906],
     [-1.3480339050292969, -4.239188194274902, 0.0008668899536132812],
     [-0.43129730224609375, -4.575753211975098, 0.004513740539550781],
     [0.15888595581054688, -2.9458394050598145, 0.0028638839721679688],
     [-0.020209312438964844, -1.3719573020935059, -0.00025081634521484375],
     [-0.12377357482910156, 0.8423192501068115, 0.00299835205078125],
     [2.2681922912597656, 1.6104834079742432, 0.0028314590454101562],
     [2.797421455383301, 0.4407784938812256, 0.0015544891357421875],
        [2.4160289764404297,-0.7244741916656494,0],
        [2.3478660583496094,-2.331594467163086,0]]
    '''
    pos = [[-0.281,-0.793,0],[0.331,-0.262,0],[0.828,0.295,0],[1.24,0.885,0],[1.67,1.54,0],[2.24,2.31,0],[2.95,2.84,0],[2.41,4.02,0],[1.19,4.72,0],[0.41,5.16,0],[-0.477,5.53,0],[-1.36,6.01,0],[-2.06,6.48,0],[-3.3,5.02,0],[-2.46,3.3,0],[-1.55,2.86,0],[-0.452,2.32,0],[0.541,1.75,0],[1.25,1.39,0],[-0.327,-0.807,0],[-0.909,-0.394,0],[-1.64,0.0751,0],[-2.36,0.407,0],[-3.2,0.811,0]]'''
    #pos = [[-2.46,3.3,0],[-1.55,2.86,0],[-0.452,2.32,0],[0.541,1.75,0],[1.25,1.39,0],[-0.327,-0.807,0],[-0.909,-0.394,0],[-1.64,0.0751,0],[-2.36,0.407,0],[-3.2,0.811,0]]
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




