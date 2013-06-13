#! /usr/bin/env python
import roslib; roslib.load_manifest('srs_user_tests')

import rospy
from math import fabs, sqrt, pi
import numpy
import tf
import rosbag
import os
import glob
import sys
from geometry_msgs.msg import PoseStamped

class TfEval(object):
        
    
    def __init__(self, def_frame_1='/map', def_frame_2='/rviz_cam', def_bag_file_dir=''):
        
        self.frame_1 = rospy.get_param('~frame_1', def_frame_1)
        self.frame_2 = rospy.get_param('~frame_2', def_frame_2)
        self.debug = rospy.get_param('~debug', False)
        
        # /logs/$(arg id)/$(arg exp)/$(arg task)/$(arg cond)/
        
        self.id = rospy.get_param('~id')
        self.exp = rospy.get_param('~exp')
        self.task = rospy.get_param('~task')
        self.cond = rospy.get_param('~cond')
        self.path = rospy.get_param('~path')
        self.output_file = rospy.get_param('~output_file')
        
        self.start_time = rospy.Time(rospy.get_param('~start_time',0.0))
        self.end_time = rospy.Time(rospy.get_param('~end_time',0.0))
        
        self.bag_filename = None
        
        self.cache_dur = rospy.Duration(0.0)
        
        self.first_attempt = True
        
        self.notes = ""
        self.req_dur = self.end_time - self.start_time
        
        self.listener = tf.TransformListener()
        
        self.tfpublisher= rospy.Publisher("tf",tf.msg.tfMessage)
        
        if self.path[-1] != '/':
            
            self.path += '/'
            
        self.bag_file_dir = self.path + self.id + '/' + self.exp + '/' + self.task + '/' + self.cond + '/'
        
        if os.path.isfile(self.path + self.output_file):
        
            self.csv = open(self.path + self.output_file,'a')
            
        else:
            
            rospy.loginfo('Creating new CSV file.')
            self.csv = open(self.path + self.output_file,'w')
            self.csv.write('filename;id;experiment;task;condition;req_start;req_end;req_duration;real_start;real_end;real_dur;frame_in_motion;path_len;rotations;notes\n')
        
        
        
        if not os.path.isdir(self.bag_file_dir):
            
            msg = 'Directory ' + str(self.bag_file_dir) + 'does not exist.'
            self.notes += msg
            rospy.logerr(msg)
            self.write_csv()
            self.csv.close()
            sys.exit()
        
        fn = glob.glob(self.bag_file_dir + '*.bag')
        
        if len(fn) == 0:
            
            msg = 'There is no bag file in ' + self.bag_file_dir + '.'
            
            self.notes += msg
            rospy.logerr(msg)
            self.write_csv()
            self.csv.close()
            
            sys.exit()
            
        if len(fn) > 1:
            
            msg = "There are more bag files in the directory! Don't know which one to analyze."
            
            self.notes += msg
            rospy.logerr(msg)
            self.write_csv()
            self.csv.close()
            
            sys.exit()
            
        self.bag_filename = fn[0]
        
        try:
        
            self.bag = rosbag.Bag(self.bag_filename)
            
        except rosbag.ROSBagException:
            
            msg = 'Error on openning bag file (' + self.bag_filename + ').' 
            rospy.logerr(msg)
            
            self.notes += msg
            self.write_csv()
            self.csv.close()
            
            rospy.signal_shutdown(msg)
            sys.exit()
            
        except rosbag.ROSBagFormatException:
            
            msg = "Bag file is corrupted and can't be open."
            rospy.logerr(msg)
            
            self.notes += msg
            self.write_csv()
            self.csv.close()
            
            rospy.signal_shutdown(msg)
            sys.exit()
            
            
        if self.start_time.to_sec() > self.end_time.to_sec():
            
            msg = "Requested start time is higher then end time."
            rospy.logerr(msg)
            
            self.notes += msg
            self.write_csv()
            self.csv.close()
            
            rospy.signal_shutdown(msg)
            sys.exit()
     
        self.started = False
        
        self.pose = PoseStamped()
        self.pose.header.frame_id = self.frame_2
        self.pose.header.stamp = rospy.Time(0)
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.0
        
        self.last_pose = None
        
        self.changes = rospy.Duration(0)
        self.timer_period = rospy.Duration(0.01)
        
        
        
        self.timer_trig = False
    
        #rospy.Timer(self.timer_period, self.timer)
        
        rospy.loginfo('Initialized with %s and %s frames.',self.frame_1,self.frame_2)
        rospy.loginfo('File to analyze: %s',self.bag_filename)
        dur = self.end_time.to_sec() - self.start_time.to_sec()
        rospy.loginfo('Start time: %f, end time: %f (dur: %f)',round(self.start_time.to_sec(),2), round(self.end_time.to_sec(),2),round(dur,2))
        
        self.change_occ_time = rospy.Time(0)
        self.change_last_time = rospy.Time(0)
        self.intgr = 0
        self.change = False
        self.last_path_len = 0.0
        
        
    def write_csv(self,real_start=-1,real_end=-1,real_dur=-1,frame_in_motion=-1,path_len=-1,rotations=-1):
        
        if self.bag_filename is None:
            
            fn = self.bag_file_dir + "xxx"
            
        else:
            
            fn = self.bag_filename
        
        self.csv.write(fn + ';' + self.id + ';' + self.exp + ';' + self.task + ';' + self.cond + ';' + str(self.start_time.to_sec()) + ';' + str(self.end_time.to_sec()) + ';' + str(self.req_dur.to_sec()) + ';' + str(real_start) + ';' + str(real_end) + ';' + str(real_dur) + ';' + str(frame_in_motion) + ';' + str(path_len) + ';' + str(rotations) + ';' + self.notes + '\n')
        
        
    def readBag(self,start,end,start_pose=None):
        
        
        frames_avail = rospy.Time(0)

        frames_checked = False
        
        started = False
        last_pose = start_pose
        last = None

        
        
        np = None
        
        path_len = 0.0
        
        start_real = rospy.Time(0)
        #end_real = rospy.Time(0)
        end_real = start-self.cache_dur
        
        error = False
        error_at = rospy.Time(0)
        
        path_len = 0.0
        
        rotations = 0.0
        
        changes = rospy.Duration(0)
        
        #self.change = False
        #self.change_last_time = rospy.Time(0)
        
        
        if self.debug:
            
            rospy.loginfo('Request for analysis from: %s, to: %s.',str(round(start.to_sec(),2)), str(round(end.to_sec(),2)))
    
        try:
            
                for topic, msg, t in self.bag.read_messages(topics=['/tf'],start_time=start-self.cache_dur,end_time=end):
                    
                    end_real = t
                    
                    
                    if rospy.is_shutdown():
                        
                        msg = "Analysis canceled. "
                        self.notes += msg
                        rospy.loginfo(msg)
                        break
                        
                    try:
                        
                        self.tfpublisher.publish(msg)
                        
                    except TypeError:
                        
                        if self.debug:
                            
                            rospy.logwarn('Strange TF msg.')
                            
                        pass
                    
                    if self.first_attempt is True:
                        
                        dif = t - start
                        
                        if dif > rospy.Duration(0.01):
                            
                            msg = "Requested start time (" + str(start.to_sec()) + ") is too low! First msg available at " + str(t.to_sec()) + " (diff: " + str(dif.to_sec()) + '). ' 
                            self.notes += msg
                            rospy.logwarn(msg) 
                        
                        self.first_attempt = False
                        
                    #rospy.sleep(0.001)
                    
                    if not frames_checked:
                        
                        if self.listener.frameExists(self.frame_1) and self.listener.frameExists(self.frame_2):
                            
                            frames_avail = t
                            frames_checked = True
                            if self.debug:
                            
                                rospy.loginfo('Both TF frames are available (%s). Caching TF...',str(t.to_sec()))
                            
                        else:
                            
                            continue
                        
                    if not started: #and (t >= frames_avail + self.cache_dur):
                        last = t
                        start_real = t
                        started = True
                        
                        if self.debug:
                        
                            rospy.loginfo('Starting analysis, at %f',round(t.to_sec(),2))
                    
                    if (t - last >= self.timer_period): # we don't want to do calculations each for iteration
                        
                        last = t
                        self.pose.header.stamp = rospy.Time(0)
        
                        try:
                
                            self.listener.waitForTransform(self.frame_1,self.frame_2,self.pose.header.stamp,rospy.Duration(0.05))
                        
                            np = self.listener.transformPose(self.frame_1, self.pose)
                    
                        except Exception, e:
                            
                            if self.debug: 
                                
                                rospy.logwarn("Can't transform (%s).",str(t.to_sec()))
                                
                            continue
                        
                        if last_pose is None:
                            
                            last_pose = np
                            
                            continue
                        
                        pdist = sqrt((np.pose.position.x - last_pose.pose.position.x)**2 + (np.pose.position.y - last_pose.pose.position.y)**2 + (np.pose.position.z - last_pose.pose.position.z)**2)
                        #path_len += pdist
                        
                        (r, p, y) = tf.transformations.euler_from_quaternion([np.pose.orientation.x,
                                                                             np.pose.orientation.y,
                                                                             np.pose.orientation.z,
                                                                             np.pose.orientation.w],axes='sxyz') # is this order correct??
                        
                        (lr, lp, ly) = tf.transformations.euler_from_quaternion([last_pose.pose.orientation.x,
                                                                                 last_pose.pose.orientation.y,
                                                                                 last_pose.pose.orientation.z,
                                                                                 last_pose.pose.orientation.w],axes='sxyz')
                        
                        
                        #dor = fabs(r - lr)
                        #dop = fabs(p - lp)
                        #doy = fabs(y - ly)
                        
                        dor = min((2 * pi) - abs(r - lr), abs(r - lr))
                        dop = min((2 * pi) - abs(p - lp), abs(p - lp))
                        doy = min((2 * pi) - abs(y - ly), abs(y - ly))
                        
                        if self.debug and doy > 6.0:
                            
                            print "y: " + str(y) + ", ly: " + str(ly)
                        
                        
                        #print str(pdist) + ", " + str(dor) + ", " + str(dop) + ", " + str(doy)
                        
                        # if there is any change in position/orientation, lets do some stuff
                        if pdist > 0.01 or dor > 0.01 or dop > 0.01 or doy > 0.01:
                        
                            path_len += pdist
                        
                            if self.change is False:
                                
                                self.change_occ_time = t
                                self.change = True
                                self.intgr = 0
                                #print "ch_s " + str(t.to_sec())
                                
                            #else:
                                
                                #print "ch " + str(self.intgr)
                
                            #cnt = 0
                            self.intgr += 1
                            self.change_last_time = t
                            
                            rotations += doy
                                
                            last_pose = np
                            self.last_path_len = path_len
                            
                            #print "ch  "  + str(self.intgr) + ", " + str(self.change) + ", akt: " + str((t).to_sec()) + ", occ: " + str(self.change_occ_time.to_sec()) + ", last: " + str(self.change_last_time.to_sec())
                            
                        else:
                            
                            #print "chn "  + str(self.intgr) + ", " + str(self.change) +  ", akt: " + str((t).to_sec()) + ", occ: " + str(self.change_occ_time.to_sec()) + ", last: " + str(self.change_last_time.to_sec())
                            
                            if (self.change is True) and ((t-self.change_last_time) >= rospy.Duration(0.5)): #cnt > 50: # 20
                                
                                #print "ch_e " + str(t.to_sec())
                                
                                if self.intgr > 1:
                                
                                    dt = self.change_last_time - self.change_occ_time
                                    changes += dt
                                    
                                if self.debug: #and dt != rospy.Duration(0.0):
                                
                                    print str(dt.to_sec()) + '; ' + str(self.intgr)
                                
                                self.change = False
                            
                        
        except rosbag.ROSBagFormatException:
            
            msg = "Bag file format corrupted around " + str(round(end_real.to_sec(),2)) + ". "
            self.notes += msg
            
            if self.debug:
                
                rospy.logerr(msg)
                
            error = True
        
        except roslib.message.DeserializationError:
            
            msg = 'Bag file deserialization error around ' + str(round(end_real.to_sec(),2)) + '. '
            self.notes += msg
            
            if self.debug:
            
                rospy.logerr(msg)
                
                
            error = True
            
        except:
            
            msg = 'Some other error around ' + str(round(end_real.to_sec(),2)) + '. '
            self.notes += msg
            
            if self.debug:
            
                rospy.logerr(msg)
                
            error = True
            
        if np is None and last_pose is not None:
            
            np = last_pose
            
        return (error, start_real, end_real, changes, path_len, rotations, np)
        
        
    def analyze(self):
        
        dur = rospy.Duration(0)
        errors = 0
        spath_len = 0.0
        srotations = 0.0
        last_pose = None
        
        (error, start_real, end_real, changes, path_len, rotations, last_pose) = self.readBag(self.start_time, self.end_time, last_pose)
        
        fstart_real = start_real # first start real
        dur = end_real - start_real
        schanges = changes # sum of changes
        spath_len = path_len
        srotations = rotations
        
        while (error is True and end_real < self.end_time and not rospy.is_shutdown()):
            
            end_real += self.cache_dur + rospy.Duration(0.25)
            
            (error, start_real, end_real, changes, path_len, rotations, last_pose) = self.readBag(end_real, self.end_time, last_pose)
            
            errors += 1
            
            if start_real != rospy.Time(0):
                         
                dur += end_real - start_real
                
                schanges += changes
                spath_len += path_len
                srotations += rotations
               
        rospy.loginfo('Finished (%d errors).',errors)
        
        self.notes += 'No. of errors: ' + str(errors) + '. '
        
        tot = str(round(dur.to_sec(),2))
        ch = str(round(schanges.to_sec(),2))
        
        if fstart_real != rospy.Time(0):
        
            rospy.loginfo('Real start: %s, real end: %s.',str(fstart_real.to_sec()),str(end_real.to_sec()))
            rospy.loginfo('Total duration: ' + tot + 's')
            rospy.loginfo('Changes:' + ch + 's, path length: ' + str(round(spath_len,2)) + 'm, rotations: ' + str(round(srotations,2)) + 'rads.' )
            
            self.write_csv(fstart_real.to_sec(), end_real.to_sec(), tot, ch, round(spath_len,2), round(srotations,2))
            
        else:
            
            msg = "Can't compute anything. One of the frames was probably not available."
            self.notes += msg
            self.write_csv()
            
            rospy.logwarn(msg)
        
        
        self.csv.close()
        
        
if __name__ == '__main__':

        rospy.init_node('tf_test_eval')

        node = TfEval()
        
        node.analyze()
        
