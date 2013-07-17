#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 Miguel Sarabia del Castillo
# Imperial College London
#
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#


import threading
import roslib

roslib.load_manifest('deadlockTest')
import rospy

import actionlib
from deadlockTest.msg import CountdownAction, CountdownResult

class SAS():
    # These variables should be treated as a constant
    NODE_NAME = "simple_action_server_test"

    def __init__(self):
        # ROS initialization:
        rospy.init_node(self.NODE_NAME, log_level=rospy.DEBUG)      
        
        #Parameters
        self.preempted = False
        self.countdown = 0
        self.countdown_rate = rospy.Rate(rospy.get_param("~frequency"))
        
        #Thread stuff
        self.lock = threading.RLock()
        
        # Prepare Simple Action Server
        self.action_server = actionlib.SimpleActionServer(
            "deadlock",
            CountdownAction,
            execute_cb = self.execute_action,
            auto_start = False)
        
        self.action_server.register_preempt_callback(self.stop_action)
        self.action_server.start()

    def execute_action(self, goal):

        rospy.loginfo("──> Executing new action")

        # Create new async thread to deal with request
        with self.lock:
            self.countdown = goal.countdown
            thread = threading.Thread(target = self.run )
            thread.start()
        
        rospy.loginfo("──> Waiting for action to finish")

        #Wait for thread to finish
        thread.join()
        
        rospy.loginfo("──> Cleaning up for next iteration")

        ##Clean up for next iteration before finishing this goal
        with self.lock:
            self.preempted = False

        #Prepare results
        rospy.loginfo("──> Preparing results")
        result = CountdownResult()
        result.remaining = self.countdown
        
                    
        #Evaluate results
        if result.remaining < 0:
            rospy.logerr("──> Code should never reach this point")
            self.action_server.set_aborted(result)
        
        elif self.preempted :
            rospy.loginfo("──> Action preempted")
            self.action_server.set_preempted(result)
        
        else :
            rospy.loginfo("──> Action finished")
            self.action_server.set_succeeded(result)
    
    
    def stop_action(self):
        rospy.loginfo("──> Preempting action")
        with self.lock:
            self.preempted = True
        rospy.loginfo("──> Preempted!")
    
    
    def run(self):
        while self.countdown > 0 and not self.preempted and not rospy.is_shutdown():
            rospy.loginfo("*** Counting down from {} ***".format(self.countdown))
            self.countdown -= 1
            self.countdown_rate.sleep()

if __name__ == '__main__':

    node = SAS()
    rospy.loginfo(node.NODE_NAME + " running...")
    rospy.spin()

    rospy.loginfo(node.NODE_NAME + " stopped.")
    exit(0)
