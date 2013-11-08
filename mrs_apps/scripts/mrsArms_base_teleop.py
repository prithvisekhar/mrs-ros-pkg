#!/usr/bin/env python


'''
Keyboard teleop base and the arm.
TESTING
Type help for a list of commands, and help <command> for the usage.
'''

import cmd
import rospy
import math
from std_msgs.msg import Float64 
from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry

# Global constants
PI = 3.1415926535897931

T1_J1_CMD = '/mrs/1_L1_j1_position_controller/command'
T1_J2_CMD = '/mrs/1_L1_j2_position_controller/command'
T1_J3_CMD = '/mrs/1_L1_j3_position_controller/command'
T1_J4_CMD = '/mrs/2_L1_j1_position_controller/command'
T1_J5_CMD = '/mrs/2_L1_j2_position_controller/command'
T1_J6_CMD = '/mrs/2_L1_j3_position_controller/command'

T1_J1_STT = '/mrs/1_L1_j1_position_controller/state'
T1_J2_STT = '/mrs/1_L1_j2_position_controller/state'
T1_J3_STT = '/mrs/1_L1_j3_position_controller/state'
T1_J4_STT = '/mrs/2_L1_j1_position_controller/state'
T1_J5_STT = '/mrs/2_L1_j2_position_controller/state'
T1_J6_STT = '/mrs/2_L1_j3_position_controller/state'

TOPIC_BASE_PLANAR = '/cmd_vel'
TOPIC_ODOM = '/odom'

DEFAULT_SPEED_LIN = 0.3
DEFAULT_SPEED_ANG = 0.4

# utilities functions

def is_float(string):
    '''Check if a string is a float.'''
    try:
        float(string)
        return True
    except ValueError:
        return False

# messages generation

def get_twist(speed_lin, orientation, speed_ang, clockwise):
    '''Generate a twist message.'''
    msg = Twist()
    msg.linear.x = speed_lin * math.cos(orientation)
    msg.linear.y = speed_lin * math.sin(orientation) 
    msg.angular.z = speed_ang * clockwise
    return msg

def get_duration_lin(speed_lin, length):
    '''Calculate duration for linear move.'''
    return float(math.fabs(length / speed_lin))

def get_duration_ang(speed_ang, angle):
    '''Calculate duration for angular move.'''
    return float(math.fabs(angle / speed_ang))

# messages publishers

def pub_arm_pose(pub, value):
    '''Publish message to a joint controller.'''
    pub.publish(value)

def pub_base_planar(pub, msg, duration):
    '''Publish message to planar plugin.'''
    pub.publish(msg)
    rospy.sleep(duration)
    msg_init = get_twist(0.0, 0.0, 0.0, 1.0)
    pub.publish(msg_init)

# subscribers callback functions

def callback_joint(data, (joint_description, joint_nb)):
    '''callback function for arm joints state data.'''
    joint_description[joint_nb] = data.process_value

def callback_odom(data, odom_description):
    ''' callback function for odometrie data.'''
    odom_description[0] = (data.pose.pose.position.x,
            data.pose.pose.position.y)
    odom_description[1] = data.pose.pose.orientation.z

# commands line parsers

def parse_arm_joint_cmd(line):
    '''Parse the arm_pose and arm_move command.'''
    args = str.split(line)
    joint_nb = args[0]
    value  = float(args[1])
    return (joint_nb, value)

def parse_base_move(line, rotation_info):
    '''Parse the base_move command.'''
    args = str.split(line)

    if len(args) == 0:
        return (0.1, 0.0)
    
    if len(args) == 1:
        if is_float(args[0]):
            return (float(args[0]), 0.0)
        else:
            return (0.1, rotation_info[args[0]])

    if len(args) == 2:
        if is_float(args[0]):
            return (float(args[1]), float(args[0]))
        else:
            return (float(args[1]), rotation_info[args[0]])

def parse_base_spin(line, rotation_info):
    '''Parse the base_spin command.'''
    args = str.split(line)

    if len(args) == 0:
        return 0.2

    if len(args) == 1:
        if is_float(args[0]):
            return float(args[0])
        else:
            return rotation_info[args[0]]

class CommandMrs(cmd.Cmd):
    intro = 'Welcome to the teleop shell.   Type help or ? to list commands.\n'
    prompt = '(teleop) '
    file = None
    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        
        # arm_pose = [joint_1 joint_2, joint_3, joint_4, joint_5, joint_6]
        self.arm_pose = [0.0] * 6
        # odom = [(pose_x, pose_y), orientation_z]
        self.odom = [(0.0, 0.0), 0.0]
        # intialize ROS node
        rospy.init_node('Mrs_teleop')
        
        # command publishers
        self.pubj1 = rospy.Publisher(T1_J1_CMD, Float64)
        self.pubj2 = rospy.Publisher(T1_J2_CMD, Float64)
        self.pubj3 = rospy.Publisher(T1_J3_CMD, Float64)
        self.pubj4 = rospy.Publisher(T1_J4_CMD, Float64)
        self.pubj5 = rospy.Publisher(T1_J5_CMD, Float64)
        self.pubj6 = rospy.Publisher(T1_J6_CMD, Float64)


        self.pubb = rospy.Publisher(TOPIC_BASE_PLANAR, Twist)

        # ROS topics states subscribers
        rospy.Subscriber(T1_J1_STT, JointControllerState,
                callback_joint, (self.arm_pose, 0))
        rospy.Subscriber(T1_J2_STT, JointControllerState,
                callback_joint, (self.arm_pose, 1))
        rospy.Subscriber(T1_J3_STT, JointControllerState,
                callback_joint, (self.arm_pose, 2))
        rospy.Subscriber(T1_J4_STT, JointControllerState,
                callback_joint, (self.arm_pose, 3))
        rospy.Subscriber(T1_J5_STT, JointControllerState,
                callback_joint, (self.arm_pose, 4))
        rospy.Subscriber(T1_J6_STT, JointControllerState,
                callback_joint, (self.arm_pose, 5))
        rospy.Subscriber(TOPIC_ODOM, Odometry, 
                callback_odom, self.odom)
        
        # various dicts containing default values and description
        self.joints_info = {
                'j1': (0, self.pubj1),
                'j2': (1, self.pubj2),
                'j3': (2, self.pubj3),
                'j4': (3, self.pubj4),
                'j5': (4, self.pubj5),
                'j6': (5, self.pubj6),

                }

        self.rotation_info =  {
                'front': 0.0,
                'back': PI,
                'left': (PI / 2),
                'right': (-1 * PI / 2)
                }

        # default speeds initilization
        self.speed_lin = DEFAULT_SPEED_LIN
        self.speed_ang = DEFAULT_SPEED_ANG
        
    def do_arm_pose(self, line):
        '''
        Set the arm joints value.
        Usage: arm_pose jn n.n
        '''
        (joint_nb, value) = parse_arm_joint_cmd(line)
        (rank, pub) = self.joints_info[joint_nb]
        pub_arm_pose(pub, value)
    
    def do_arm_move(self, line):
        '''
        Move the arm joints of the offset from the current position.
        Usage: arm_move j_number float  ex:arm_move j2 -0.5
        '''
        (joint_nb, offset) = parse_arm_joint_cmd(line)
        (rank, pub) = self.joints_info[joint_nb]
        value = self.arm_pose[rank] + offset
        pub_arm_pose(pub, value)


    def do_base_move(self, line):
        '''
        Move the base in the given direction of the given length.
        Usage:
            base_move 
            base_move [front/back/left/right]
            base_move [front/back/left/right] [n.n]
            base_move [m.m] [n.n] (direction and length)
        By default, move the base of 0.1 m.
        '''
        (length, orientation) = parse_base_move(line, self.rotation_info)
        msg = get_twist(self.speed_lin, orientation, 0.0, 1.0)
        duration = get_duration_lin(self.speed_lin, length)
        pub_base_planar(self.pubb, msg, duration)

    def do_base_spin(self, line):
        '''
        Spin the base of a given angle.
        Usage:
            base_spin
            base_spin [right/left]
            base_spin [n.n]
        By default, spin the base of 0.2 rad.
        '''
        rotation = parse_base_spin(line, self.rotation_info)
        msg = get_twist(0.0, 0.0, self.speed_ang, math.copysign(1.0, rotation))
        duration = get_duration_ang(self.speed_ang, rotation)
        pub_base_planar(self.pubb, msg, duration)

    def do_describe(self, line):
        '''
        Display the states of the joints and the position of the base.
        Usage: describe
        '''
        print "odom: ", self.odom
        print "arm: ", self.arm_pose

    def do_end(self, line):
        '''Override end of file'''
        print "Exit!"
        return True

def main():
    cy = CommandMrs()
    cy.cmdloop()

if __name__ == '__main__':
    main()
