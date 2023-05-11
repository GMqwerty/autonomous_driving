#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int8
from modules.color import Color

def talker():
    color = Color()
    pub = rospy.Publisher('line', Int8, queue_size=10)
    rospy.init_node('lines', anonymous=True)
    rate = rospy.Rate(400) # 10hz
    while not rospy.is_shutdown():
        line_d = color.detect_line()
        # line_d = color.read_hsv()
        if line_d != 0:
            rospy.loginfo(str(line_d))
            pub.publish(line_d)
        rate.sleep()

if __name__ == '__main__':
    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass