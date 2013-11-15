#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from omni_msgs.msg import OmniButtonEvent, OmniFeedback, OmniState
from geometry_msgs.msg import Vector3, PoseStamped, Twist
import numpy as np
from cgkit.cgtypes import vec3, mat4
import math

topic_state = '/phantom/pose'
topic_button = '/phantom/button'
topic_cmdvel ='/mrs/cmd_vel'


master_st = PoseStamped()
button_st = OmniButtonEvent()
vel_st = Twist()
pub = rospy.Publisher(topic_cmdvel,Twist)
rospy.init_node('phantom_teleop')

def callback_button(button):
  button_st = button
  if button.grey_button == 1:
    rospy.loginfo("grey ON")
  if button.white_button == 1:
    rospy.loginfo("white ON")  

def callback_pose(msg):
  master_st = msg
  unpack_message(master_st)
  
def unpack_message(msg):
  default_pos = np.array([-2,-15,0.5])
  master_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
  # Correction by 20 for origin compensation 
  cmd_values = np.array([msg.pose.position.x, msg.pose.position.z])
  current_mag= np.linalg.norm(cmd_values) - 15 if (np.linalg.norm(cmd_values) > 15 ) else 0
  vel_st = Twist()
  if current_mag>0 :

    #convert and remap speed values, with deadzone of 15
    vel_st.linear.y = -msg.pose.position.z/20 if (math.fabs(msg.pose.position.z) > 50 and math.fabs(msg.pose.position.x) < 50) else 0
    vel_st.linear.x = msg.pose.position.x/20 if (math.fabs(msg.pose.position.x) > 50 and math.fabs(msg.pose.position.z) < 50) else 0
    vel_st.linear.z = 0
    vel_st.angular.x = 0
    vel_st.angular.y = 0
    vel_st.angular.z = math.atan2 (-msg.pose.position.z,msg.pose.position.x) if (math.fabs(msg.pose.position.x) > 50 and math.fabs(msg.pose.position.z) > 50) else 0
  pub.publish(vel_st)
  #~ rospy.loginfo("talker")


  current_angle=np.arccos(np.dot(master_pos, default_pos))
  
  v=vec3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
  v2=vec3(-2,-15,0.5)
  #~ print("length, angle" ,current_mag,current_angle, v.length(),v.angle(v2),master_pos )
  
  rospy.loginfo("pos %s", master_pos) 
  master_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
  master_header = np.array([msg.header.frame_id, msg.header.seq, msg.header.stamp])
  #~ rospy.loginfo(rospy.get_name() + ": I heard %s" % msg)
  #~ rospy.loginfo(rospy.get_name() + ": I heard %s" % master_pos )
  #~ rospy.loginfo(rospy.get_name() + ": I heard %s" % master_rot )
  #~ rospy.loginfo(rospy.get_name() + ": I heard %s" % master_header)

def listener_talker():
  rospy.Subscriber(topic_state, PoseStamped, callback_pose)
  rospy.Subscriber(topic_button, OmniButtonEvent,callback_button)
  rospy.loginfo("listennig" )




if __name__ == '__main__':
  print "To control robot with Phantom Omni"
  rospy.loginfo("init" )

  listener_talker()
  rospy.spin()
