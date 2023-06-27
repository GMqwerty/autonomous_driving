#!/usr/bin/env python3
from simple_pid import PID
import rospy, time
import termios, sys, tty
from std_msgs.msg import String, Int16, Int16MultiArray, Int8
import steering
from modules.pid import PID

encoder = Int16MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]
left_distance = right_distance = 0
gyro_pos = 0
compass_pos = 0
first_line = 0
do_turn = False

steer = steering.Steering()
reference_angle = 0

pub = rospy.Publisher('speed', String, queue_size=10)

def line_callback(data):
    global first_line, do_turn

    rospy.loginfo(rospy.get_caller_id() + "Line %s", data.data)
    if first_line == 0:
        first_line = data.data
    if first_line == data.data and do_turn is False:
        do_turn = True
        print("$$$$$$$$$$$  TURN $$$$$$$$$$$$$$")

def reset_gyro_angle():
    global reference_angle

    reference_angle = gyro_pos

def read_angle():
    return reference_angle - gyro_pos



def distance_callback(data):
    global left_distance, right_distance

    #rospy.loginfo(rospy.get_caller_id() + "Distances %s", data.data)
    left_distance = data.data[0]
    right_distance = data.data[1]
    # print("LD: ", left_distance)
    # print("RD: ", right_distance)


def gyro_callback(data):
    global gyro_pos

    #rospy.loginfo(rospy.get_caller_id() + "Gyro angle %s", str(data.data))
    gyro_pos = data.data

def heading_callback(data):
    global compass_pos

    #rospy.loginfo(rospy.get_caller_id() + "Heading %s", data.data)
    compass_pos = data.data

def encoders_callback(data):
    global encoder

    #rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
    encoder = data
    # print("Encoder: ", encoder.data)

    
def dc_speed(speed=0):
    msg = str(speed)
    #rospy.loginfo(msg)
    pub.publish(msg)

def turn(angle):
    global do_turn

    print("start steering in demo turn function")
    time.sleep(0.5)
    print("stop steering in demo turn function")
    do_turn = False

def drive_straight(speed=50, pos=0, distance_threshold=1000, wall_side=0, pid_ks=(1, 0, 0)):
    # Initialise PID object
    pid = PID(pid_ks[0], pid_ks[1], pid_ks[2])
    pid.SetPoint = 0
    pid.setSampleTime(0.005)
    dc_speed(speed)
    start_time = time.time()
    last_turn = 0
    first_straight = True
    direction = 0
    try:
        while True:
            current_ms = time.time() * 1000
            # Calculate error
            error = read_angle() + pos
            # Calculate the correction value with the PID object
            pid.update(error)
            u = int(pid.output)
            current_steer = int(min(100, max(u, -100)))
            #print("current_steer:", current_steer)
            steer.set_steering(current_steer)
            # if time.time() - start_time > 10:
            #     dc_speed(0)
            #     break
            if current_ms - last_turn > 3000:
                print(left_distance, right_distance)
                if left_distance > 1000:
                    last_turn = current_ms
                    pos += 90
                if right_distance > 1000:
                    last_turn = current_ms
                    pos -= 90
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        dc_speed(0)
        steer.set_steering(0)
        return
# def turn(speed=50, current_pos=0, new_pos=-90):
#     new_angle =  gyro_pos + new_pos
#     if new_pos < 0:
#         steer.set_steering(-100)
#     elif new_pos > 0:
#         steer.set_steering(100)
    




def listener():
    
    rospy.init_node('run1', anonymous=True)

    rospy.Subscriber("line", Int8, line_callback)
    rospy.Subscriber("lrdistance", Int16MultiArray, distance_callback)
    rospy.Subscriber("gyro_angle", Int16, gyro_callback)
    rospy.Subscriber("heading", Int16, heading_callback)
    rospy.Subscriber("encoders", Int16MultiArray, encoders_callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    rate = rospy.Rate(100) 
    dc_speed(0)
    time.sleep(5)

    reset_gyro_angle()
    drive_straight(speed=60, pos=0, pid_ks=(1.5, 0.1, 0.005))
    dc_speed(0)
    steer.set_steering(0)

if __name__ == '__main__':
    listener()