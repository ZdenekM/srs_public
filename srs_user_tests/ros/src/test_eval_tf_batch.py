#! /usr/bin/env python
import roslib; roslib.load_manifest('srs_user_tests')

import rospy
from math import fabs, sqrt
import numpy
import os
import glob
import sys
import csv

class TfEvalBatch(object):
    
    def __init__(self):
                
        self.path = rospy.get_param('~path')
        self.input_fn = self.path + rospy.get_param('~input_file')
        self.analysis_launch = rospy.get_param("~analysis_launch")
        
        rospy.loginfo('Reading input from %s file',self.input_fn)
        
        if not os.path.isdir(self.path):
            
            rospy.logerr("Path doesn't exist!")
            sys.exit()
        
        if not os.path.isfile(self.input_fn):
            
            rospy.logerr("Input file doesn't exist!")
            sys.exit()
            
            
        self.input_file = open(self.input_fn,"rb")
        self.reader = csv.reader(self.input_file, delimiter=';', quotechar='"')
            
        rospy.loginfo('Initialized.')
        
        
    def analyze(self):
        
        first_row = False
        
        cnt = 0
        
        for row in self.reader:
            
            cnt +=1
            
            if not first_row:
                
                first_row = True
                continue
            
            # roslaunch srs_user_tests analyze_robot_movement.launch exp:=e1 task:=nav2 id:=p06 cond:=a start:=1362417198 end:=1362417277
            
            rospy.loginfo("Launching analysis no. " + str(cnt) + ".")
            print row[4] + ', ' + row[5]
            os.system("roslaunch srs_user_tests "+ self.analysis_launch + ".launch exp:="+ row[1] + " task:="+ row[2] + " id:="+ row[0] + " cond:="+ row[3] + " start:="+ row[4] + " end:=" + row[5] + " path:=" + self.path)
            
            if rospy.is_shutdown():
                
                rospy.logwarn("Canceling analysis...")
                break
        
        
        rospy.loginfo('Finished.')
        self.input_file.close()
        
        
if __name__ == '__main__':

        rospy.init_node('tf_test_eval_batch')

        node = TfEvalBatch()
        
        node.analyze()