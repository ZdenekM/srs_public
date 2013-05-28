#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by dcgm-robotics@FIT group.
# 
# Author: Zdenek Materna (imaterna@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: dd/mm/2012
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.
#
import roslib; roslib.load_manifest('srs_user_tests')
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion

def main():
    
    rospy.init_node('get_robot_position')
    
    tf = TransformListener()
    
    g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    rospy.wait_for_service("/gazebo/get_model_state")
    
    try:
            
      state = g_get_state(model_name="robot")
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      return
  
    print "\n"
    print "Position from Gazebo:"
    print state.pose
    
    r_pose = PoseStamped()
    
    r_pose.header.stamp = rospy.Time(0)
    r_pose.header.frame_id = '/base_link'
    r_pose.pose.position = Vector3(0,0,0)
    r_pose.pose.orientation = Quaternion(0,0,0,1)
    
    tf.waitForTransform('/map',r_pose.header.frame_id,rospy.Time(0), rospy.Duration(3.0))
    r_pose = tf.transformPose('/map',r_pose)
    
    print "\n"
    print "Position from localization:"
    print r_pose.pose
    print "\n"
  

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass