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
from pcms.pytorch_models import *
from pcms.openvino_models import Yolov8, HumanPoseEstimation
def move(forward_speed: float = 0, turn_speed: float = 0):
    global _cmd_vel
    msg = Twist()
    msg.linear.x = forward_speed
    msg.angular.z = turn_speed
    _cmd_vel.publish(msg)
# gemini2

def get_real_xyz(dp, x, y):
    a = 55.0 * np.pi / 180
    b = 86.0 * np.pi / 180
    d = dp[y][x]
    h, w = dp.shape[:2]
    x = int(x) - int(w // 2)
    y = int(y) - int(h // 2)
    real_y = round(y * 2 * d * np.tan(a / 2) / h)
    real_x = round(x * 2 * d * np.tan(b / 2) / w)
    return real_x, real_y, d
#gemini2
def callback_image2(msg):
    global frame2
    frame2 = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_depth2(msg):
    global depth2
    depth2 = CvBridge().imgmsg_to_cv2(msg, "passthrough")
#astra
def callback_image1(msg):
    global frame1
    frame1 = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_depth1(msg):
    global depth1
    depth1 = CvBridge().imgmsg_to_cv2(msg, "passthrough")
def callback_depth1(msg):
    global _depth1
    _depth1 = CvBridge().imgmsg_to_cv2(msg, "passthrough")
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

    frame2 = None
    rospy.Subscriber("/camera/color/image_raw", Image, callback_image2)

    depth2 = None
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth2)
    
    frame1 = None
    rospy.Subscriber("/cam2/rgb/image_raw", Image, callback_image1)

    depth1= None
    rospy.Subscriber("/cam2/depth/image_raw", Image, callback_depth1)
    
    topic_imu = "/imu/data"
    _imu=None
    rospy.Subscriber(topic_imu, Imu, callback_imu)
    rospy.wait_for_message(topic_imu, Imu)
    dnn_yolo = Yolov8("yolov8n", device_name="GPU")
    print("yolo")
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
    #pos = [[-2.46,3.3,0],[-1.55,2.86,0],[-0.452,2.32,0],[0.541,1.75,0],[1.25,1.39,0],[-0.327,-0.807,0],[-0.909,-0.394,0],[-1.64,0.0751,0],[-2.36,0.407,0],[-3.2,0.811,0]]
    print("start")
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
        if frame1 is None: 
            print("frame1")
            continue
        if frame2 is None: 
            print("frame2")
            continue
        if depth1 is None: 
            print("depth1")
            continue
        if depth2 is None: 
            print("depth2")
            continue
        up_image=frame2.copy()
        down_image=frame1.copy()
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
            for i in range(290):
                move(0,0.25)
                time.sleep(0.1)
            detections1 = dnn_yolo.forward(up_image)[0]["det"]
            detections2 = dnn_yolo.forward(down_image)[0]["det"]
            goal_index[0,1,2,3,4,5,6,7,8,9]
            for i, detection in enumerate(detections1):
                x1, y1, x2, y2, score, class_id = map(int, detection)
                score=detection[4]
                if class_id not in goal_index: continue
                if score<0.3: continue
                al.append([x1, y1, x2, y2, score, class_id])
                print(float(score), class_id)
                cv2.rectangle(up_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(up_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(up_image, str(class_id), (x1+5, y1+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            for i, detection in enumerate(detections2):
                x1, y1, x2, y2, score, class_id = map(int, detection)
                score=detection[4]
                if class_id not in goal_index: continue
                if score<0.3: continue
                al.append([x1, y1, x2, y2, score, class_id])
                print(float(score), class_id)
                cv2.rectangle(down_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(down_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(down_image, str(class_id), (x1+5, y1+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            bb=sorted(al, key=(lambda x:x[0]))
                    
                    
                  
        h,w,c = up_image.shape
        upout=cv2.line(up_image, (320,0), (320,500), (0,255,0), 5)
        downout=cv2.line(down_image, (320,0), (320,500), (0,255,0), 5)
        img = np.zeros((h,w*2,c),dtype=np.uint8)
        img[:h,:w,:c] = upout
        img[:h,w:,:c] = downout
        
        cv2.imshow("frame", img)   
        key = cv2.waitKey(1)
        #say("I am back")
        key = cv2.waitKey(1)
        if key in [ord('q'), 27]:
            break
