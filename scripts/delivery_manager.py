#!/usr/bin/env python

from cob_web_delivery.srv import *
from cob_web_delivery.msg import *

import rospy
import actionlib

class DeliveryServer:
  # create messages that are used to publish feedback/result
  _feedback = DeliveryFeedback()
  _result   = DeliveryResult()
  
  def __init__(self):
    self.server = actionlib.SimpleActionServer('delivery', DeliveryAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    print "Got request for %s"%(goal.item)
    rospy.sleep(rospy.Duration.from_sec(2.0))
    self._feedback.state = 2
    self.server.publish_feedback(self._feedback)
    rospy.sleep(rospy.Duration.from_sec(2.0))
 
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
