#!/usr/bin/env python

'''
BASE teleop program
$rosrun package name
$ rosrun mrs_apps base_teleop.py 
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
    '''callback function for arm and gripper joints state data.'''
    joint_description[joint_nb] = data.process_value

def callback_odom(data, odom_description):
    ''' callback function for odometrie data.'''
    odom_description[0] = (data.pose.pose.position.x,
            data.pose.pose.position.y)
    odom_description[1] = data.pose.pose.orientation.z

# commands line parsers


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

class CommandYouBot(cmd.Cmd):
    
    '''Subclass of the cmd class'''

    def preloop(self):
        '''Override and used for class variable'''
        
        # odom = [(pose_x, pose_y), orientation_z]
        self.odom = [(0.0, 0.0), 0.0]
        # intialize ROS node
        rospy.init_node('mrsbase_teleop')
        
        # command publishers
   

        self.pubb = rospy.Publisher(TOPIC_BASE_PLANAR, Twist)

        # ROS topics states subscribers
       
        rospy.Subscriber(TOPIC_ODOM, Odometry, 
                callback_odom, self.odom)
        

        self.rotation_info =  {
                'front': 0.0,
                'back': PI,
                'left': (PI / 2),
                'right': (-1 * PI / 2)
                }

        # default speeds initilization
        self.speed_lin = DEFAULT_SPEED_LIN
        self.speed_ang = DEFAULT_SPEED_ANG

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
       
    def do_end(self, line):
        '''Override end of file'''
        print "EXIT!"
        return True

def main():
    cy = CommandYouBot()
    cy.cmdloop()

if __name__ == '__main__':
    main()
