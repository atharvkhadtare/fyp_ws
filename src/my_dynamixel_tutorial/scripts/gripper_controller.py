#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

# rate = rospy.Rate(10) # 10hz
# rate.sleep()

load_threshold = 0.1

def open_gripper():
    # while not rospy.is_shutdown():
    gripper_pub.publish(1) # 1 radians

def state_callback(current_state):
    global current_goal_pos
    global current_load
    current_load = current_state.load
    current_goal_pos = current_state.goal_pos 
    if (current_load < 0):
        current_load = current_load * (-1)
    
    # print "current_goal_pos", current_goal_pos

def close_gripper():
    print "in close_gripper"
    global current_goal_pos
    global current_load

    angle = current_goal_pos
    
    while(current_load < load_threshold) :
        print "angle ", angle,
        print " current_load ", current_load
        angle = angle - 0.05
        # if (angle<0):
        #     angle = 0
        gripper_pub.publish(angle)
    print "close_gripper END"
    print "angle ", angle,
    print " current_load ", current_load
    exit()

def clean_exit():
    exit()

if __name__ == '__main__':
    gripper_pub = rospy.Publisher('/gripper_controller/command', Float64, queue_size=10)
    rospy.Subscriber('/gripper_controller/state', JointState, state_callback)
    rospy.init_node('gripper_controller', anonymous=True)
    # open_gripper()
    # rospy.sleep(3.)
    close_gripper()
    rospy.spin()
    rospy.on_shutdown(clean_exit)
        
