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
    print "Waiting for segmentation data"
    self.mapsegs = rospy.get_param('segs') #try wait
    self.destinations = rospy.get_param('destinations')
    print "Ready to Accept Orders"
    
  def handle_order_que(self, req):
      room_id = self.get_room(pixels=req.pixels, mapsegs=self.mapsegs)
      if room_id == (0 or 255):
        accepted = False
      target = self.get_destinations(room_id=room_id)
      print target
      print "Appending %s with Priority %d and destination x: %F y: %F to Que  "%(req.item, req.priority, req.target_pose1.position.x, req.target_pose1.position.y)
      goal = DeliveryGoal()
      goal.target_pose1 = req.target_pose1
      goal.target_pose2 = req.target_pose2
      goal.item = req.item
      self.ac.send_goal(goal)#feedbackCB in here?
      accepted = self.ac.wait_for_result()
      print "Action Returned Error: %s in state: %d "%(self.ac.get_result().Error, self.ac.get_result().state)
      return OrderQueResponse(accepted, "Room 1", req.target_pose1, req.target_pose2)

  def get_room(self, pixels, mapsegs):
      pixfield = mapsegs["width"]*(pixels[1]-1)+(pixels[0]-1)
      return mapsegs["data"][pixfield]

  def get_destinations(self, room_id):
      return self.destinations["room_position_dict"][str(room_id)]

if __name__ == "__main__":
    rospy.init_node('order_que')
    server = QueServer()
    rospy.spin()
