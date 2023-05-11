#!/usr/bin/env python3
import rospy, time
import termios, sys, tty
from std_msgs.msg import String, Int16, Int32MultiArray, Int8
import steering
import time 
import cv2
import os
from camera import Camera

# WIDTH = 640 
# HEIGHT = 480
# FRAMERATE = 60
# FLIP_METHOD = 0

encoder = Int32MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]

steer = steering.Steering()
steer.set_steering(0)
pub = rospy.Publisher('speed', String, queue_size=10)

cap = Camera()

def encoders_callback(data):
    global encoder

    # rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
    encoder = data
    # print("Encoder: ", encoder.data)

    
def dc_speed(speed=0):
    msg = str(speed)
    # rospy.loginfo(msg)
    pub.publish(msg)
    
def listener():
    
    rospy.init_node('drive_remotely', anonymous=True)

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
                try:
                    cv_img = cap.read()
                    img_file = os.path.join(image_directory,"img_" + str(image_counter) + ".png")
                    print(img_file)
                    print(cv_img.shape)
                    status = cv2.imwrite(img_file, cv_img)
                    rospy.loginfo("Save image No " + str(img_file) + ", status: " + str(status))
                    print("Save image No ", str(img_file), ", status: ", status)
                    image_counter += 1
                except Exception as e:
                    print("Camera Fail")
                    print(e)
                    # cap.reboot()
                    continue
                
            rate.sleep()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
        cap.kill_camera = True
        time.sleep(3)
    finally:
        print("Exit!")
        dc_speed(0)
        steer.set_steering(0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
        cap.kill_camera = True
        time.sleep(3)
if __name__ == '__main__':
    listener()