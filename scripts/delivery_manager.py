#!/usr/bin/env python

from cob_web_delivery.srv import *
from cob_web_delivery.msg import *
from move_base_msgs.msg import *

import rospy
import actionlib

class DeliveryServer:
  # create messages that are used to publish feedback/result
  _feedback = DeliveryFeedback()
  _result   = DeliveryResult()
  
  def __init__(self):
    print "Starting delivery Server"
    self.server = actionlib.SimpleActionServer('delivery', DeliveryAction, self.execute, False)
    self.server.start()
    print "Delivery Server started"
    print "Connecting to move_base ActionServer"
    self.mbac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.mbac.wait_for_server()
    print "Connected to MoveBAse"


  def execute(self, goal):
    print "Got request for %s"%(goal.item)
    rospy.sleep(rospy.Duration.from_sec(1.0))
    self._feedback.state = 2
    self.server.publish_feedback(self._feedback)
    rospy.sleep(rospy.Duration.from_sec(1.0))
    
    bgoal = MoveBaseGoal()
    bgoal.target_pose.header.frame_id = "map"
    bgoal.target_pose.header.stamp = rospy.Time.now()
    bgoal.target_pose.pose.position = goal.target_pose1.position
    bgoal.target_pose.pose.orientation.w = 1.0
    self.mbac.send_goal(bgoal)
 
    self._result.success = True
    self._result.Error = "Arrived"
    self._result.state = 1
    self.server.set_succeeded(self._result)
    print "Action done"


if __name__ == '__main__':
  rospy.init_node('delivery_server')
  server = DeliveryServer()
  print "Ready to Serve"
  rospy.spin()
