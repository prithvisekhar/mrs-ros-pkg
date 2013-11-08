#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from mrs_control.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("mrs_control_joint_dynamic", anonymous = True)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()
