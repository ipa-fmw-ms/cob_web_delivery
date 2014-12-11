#!/usr/bin/env python

from cob_web_delivery.srv import *
from cob_web_delivery.msg import *

import rospy
import actionlib

class QueServer:
  def __init__(self):
    s = rospy.Service('order_que', OrderQue, self.handle_order_que)
    self.ac = actionlib.SimpleActionClient('delivery', DeliveryAction)
    print "Waiting for delivery Actions Sevrer"
    self.ac.wait_for_server()
    print "Ready to Accept Orders"
    
  def handle_order_que(self, req):
      print "Appending %s with Priority %d and destination x: %F y: %F to Que  "%(req.item, req.priority, req.target_pose1.position.x, req.target_pose1.position.y)
      rospy.sleep(rospy.Duration.from_sec(1.0))
      goal = DeliveryGoal()
      goal.target_pose1 = req.target_pose1
      goal.target_pose2 = req.target_pose2
      goal.item = req.item
      self.ac.send_goal(goal)#feedbackCB in here?
      done = self.ac.wait_for_result()
      print "Action Returned Error: %s in state: %d "%(self.ac.get_result().Error, self.ac.get_result().state)
      return OrderQueResponse(done)

if __name__ == "__main__":
    rospy.init_node('order_que')
    server = QueServer()
    rospy.spin()
