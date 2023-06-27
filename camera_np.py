#!/usr/bin/env python3
# license removed for brevity
import rospy, time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

WIDTH = 640 
HEIGHT = 480
FRAMERATE = 60
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
bridge = CvBridge()

def talker():
    pub = rospy.Publisher('frame', Image, queue_size=10)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(20) 

    while not rospy.is_shutdown():
        ret_val, cv_img = cap.read()
        if not ret_val:
            print("Fail to capture")
            continue
        try:
            ros_img = bridge.cv2_to_imgmsg(cv_img, "bgr8")
            rospy.loginfo("Sending image")
            pub.publish(ros_img)
        except CvBridgeError as e:
            print(e)
            continue
        
        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass