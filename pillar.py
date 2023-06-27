#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
from modules.pillar_detection import PillarDetection
import cv2

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

pillar_detection = PillarDetection(colors={"red":{"h":(165, 10), "s":(40, 200), "v":(90, 160), "num":1}, 
                                           "green":{"h":(65, 85), "s":(0, 255), "v":(0, 255), "num":-1}})
pillar_array = Int16MultiArray()

def talker():
    pub = rospy.Publisher('pillar_topic', Int16MultiArray, queue_size=1)
    rospy.init_node('pillar', anonymous=True)
    rate = rospy.Rate(30) 
    error = 0
    while not rospy.is_shutdown(): 
        ret_val, cv_img = cap.read()
        if not ret_val:
            print("Fail to capture")
            continue
        result_image, pillar_info = pillar_detection.detect_pillars(cv_img, colors=["green"], line=True, error=error)
        cv2.imshow("CSI Camera", result_image)
        # This also acts as
        keyCode = cv2.waitKey(1) & 0xFF
        # Stop the program on the ESC key
        if keyCode == 27:
            break   

        pillar_array.data = [pillar_info["color"], pillar_info["cx"], pillar_info["cy"], pillar_info["height"], pillar_info["cerror"]]
        rospy.loginfo(str(pillar_array.data))
        pub.publish(pillar_array)
        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass