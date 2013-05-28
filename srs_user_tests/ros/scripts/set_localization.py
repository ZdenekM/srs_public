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
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty as EmptyMsg


def main():
    
    rospy.init_node('set_localization')

    sim = rospy.get_param('/use_sim_time')    

    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,latch=True)
    
    pose = Pose()
    
    prefix = ""
    
    if sim and rospy.has_param('~localization'):
        
        rospy.loginfo('Using special localization pose')
        prefix = "localization/"
        
    else:
        
        rospy.loginfo('Using normal pose')
    
    pose.position.x = rospy.get_param('~' + prefix + 'position_x')
    pose.position.y = rospy.get_param('~' + prefix + 'position_y')
    pose.position.z = rospy.get_param('~' + prefix + 'position_z')
    
    pose.orientation.x = rospy.get_param('~' + prefix + 'orientation_x')
    pose.orientation.y = rospy.get_param('~' + prefix + 'orientation_y')
    pose.orientation.z = rospy.get_param('~' + prefix + 'orientation_z')
    pose.orientation.w = rospy.get_param('~' + prefix + 'orientation_w')
    
    loc = PoseWithCovarianceStamped()
    
    loc.pose.pose = pose
    loc.header.frame_id = "/map"
    #loc.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942] # this was commented... why???
    
    rospy.loginfo("Adjusting localization")
    pub.publish(loc)
    pub.publish(loc)
    pub.publish(loc)

    rospy.sleep(1)
          
    

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
