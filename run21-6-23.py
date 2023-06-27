#!/usr/bin/env python3
from simple_pid import PID
import rospy, time
from std_msgs.msg import String, Int16, Int16MultiArray, Int32MultiArray, Int8
import steering
from modules.pid import PID

encoder = Int32MultiArray() # left_encoder, main_encoder, right_encoder
encoder.data = [0, 0, 0]
# pillar_array.data = ["color", "cx", "cy", "height", "center"]
pillar = Int16MultiArray()
pillar.data = [0, 0, 0, 0, 0]
left_distance = right_distance = 0
gyro_pos = 0
# first_line = 0
# do_turn = False
line = 0

steer = steering.Steering()
reference_angle = 0

pub = rospy.Publisher('speed', String, queue_size=10)


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

def lines_callback(data):
    global line

    #rospy.loginfo(rospy.get_caller_id() + "Gyro angle %s", str(data.data))
    line = data.data

def encoders_callback(data):
    global encoder

    #rospy.loginfo(rospy.get_caller_id() + "Encoders %s", data.data)
    encoder = data
    # print("Encoder: ", encoder.data)

def pillar_callback(data):
    global pillar

    # rospy.loginfo(rospy.get_caller_id() + "Pillar %s", data.data)
    pillar = data
    # print("Pillar: ", pillar.data)

    
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

def get_target(color, center, height, kc=1, kh=1, min_center=30):
        center = (100 + center)/2
        # print("Color:", color)
        # print("Center:", center)
        if color == 1 and center > min_center:
            t = kc * center + (kh * 100 * height)/300
            # print("t:", t)
            return min(max(-100, int(t)), 100)
        if color == -1 and center < 100 - min_center:
            t = kc * center + (kh * 100 * height)/300
            # print("t:", t)
            return -1 * min(max(-100, int(t)), 100)
        #     return -1 * min(max(-100, int(kc * (100 + center)/2 + (kh * 100 * height)/300)), 100)
        return None


def drive_gyro(speed=50, main_position=0, pid_ks=(1, 0, 0)):
    global line
    # Initialise PID object
    pid = PID(pid_ks[0], pid_ks[1], pid_ks[2])
    pid.SetPoint = 0
    pid.setSampleTime(0.005)
    dc_speed(speed)
    print("RUNNING")
    last_turn = 0
    starting_position = encoder.data
    start_distance = 0
    finish_position = 0
    turns = 0
    angle_target = main_position
    direction = 0
    while turns < 13:
        # print("Pillar.data", pillar.data)
        target = get_target(pillar.data[0], pillar.data[4], pillar.data[3], kc=0.6, kh=0.6, min_center=22)
        if target is not None:
            
            print("target:", target)
            steer.set_steering(target)
            time.sleep(0.7)
            
        else:
            current_ms = time.time() * 1000
            # Calculate error
            error = read_angle() + angle_target
            # Calculate the correction value with the PID object
            pid.update(error)
            u = int(pid.output)
            current_steer = int(min(100, max(u, -100)))
            steer.set_steering(current_steer)
        if line != 0:                    
            
            turns += 1
            angle_target += -1*line*90
            line = 0
    dc_speed(0)


def listener():
    
    rospy.init_node('run2', anonymous=True)

    rospy.Subscriber("line", Int8, lines_callback)
    rospy.Subscriber("lrdistance", Int16MultiArray, distance_callback)
    rospy.Subscriber("gyro_angle", Int16, gyro_callback)
    rospy.Subscriber("pillar_topic", Int16MultiArray, pillar_callback)
    rospy.Subscriber("encoders", Int32MultiArray, encoders_callback)

    rate = rospy.Rate(100) 
    dc_speed(0)
    steer.set_steering(0)
    time.sleep(5)

    reset_gyro_angle()
    drive_gyro(speed=40, main_position=0, pid_ks=(5, 0.1, 0.005))
    # while not rospy.is_shutdown():
    #     pass

if __name__ == '__main__':
    listener()