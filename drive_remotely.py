#!/usr/bin/env python3
import rospy, time
import termios, sys, tty
from std_msgs.msg import String, Int16, Int32MultiArray, Int8
import steering
import time 
import cv2
import os
import threading

encoder = Int32MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]

steer = steering.Steering()
steer.set_steering(0)
pub = rospy.Publisher('speed', String, queue_size=10)

WIDTH = 640 
HEIGHT = 480
FRAMERATE = 20
FLIP_METHOD = 0

streamer = ("nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                WIDTH,
                HEIGHT,
                FRAMERATE,
                FLIP_METHOD,
                WIDTH,
                HEIGHT,
            ))
cap = cv2.VideoCapture(streamer, cv2.CAP_GSTREAMER)

def encoders_callback(data):
    global encoder

    rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
    encoder = data
    # print("Encoder: ", encoder.data)

    
def dc_speed(speed=0):
    msg = str(speed)
    rospy.loginfo(msg)
    pub.publish(msg)
    


def listener():
    
    rospy.init_node('run1', anonymous=True)

    rospy.Subscriber("encoders", Int32MultiArray, encoders_callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    rate = rospy.Rate(30) 
    dc_speed(0)
    try:
        filedescriptors = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
        speed = 0
        current_steer = 0
        image_directory = os.path.join(os.getcwd(), "images")
        image_counter = 0
        while not rospy.is_shutdown():
            
            key = sys.stdin.read(1)[0]
            if key == 'e':
                break
            if key == 'w':
                dc_speed(30)
            if key == 's':
                dc_speed(-30)
            if key == 'q':
                dc_speed(0)
            if key == 'a':
                current_steer = max(current_steer-20, -100)
                steer.set_steering(current_steer)
                # time.sleep(0.2)
            if key == 'd':
                current_steer = min(current_steer+20, 100)
                steer.set_steering(current_steer)
                # time.sleep(0.2)
            if key == 'p':
                ret_val, cv_img = cap.read()
                last_frame_num = cap.get(cv2.CAP_PROP_FRAME_COUNT)
                print("last_frame_num:", last_frame_num)
                if ret_val:
                    
                    # cv2.waitKey(1)
                    img_file = os.path.join(image_directory,"img_" + str(image_counter) + ".png")
                    status = cv2.imwrite(img_file, cv_img)
                    print("Save image No " + str(img_file) + ", status: ", status)
                    image_counter += 1
                else:
                    print("fail to cap image")

            rate.sleep()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
        cap.release()
    finally:
        print("Exit!")
        dc_speed(0)
        steer.set_steering(0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
        cap.release()
if __name__ == '__main__':
    listener()