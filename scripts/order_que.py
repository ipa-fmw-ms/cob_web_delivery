#!/usr/bin/env python

from cob_web_delivery.srv import *
from cob_web_delivery.msg import *

import rospy
import actionlib


class QueServer:
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.s = rospy.Service('order_que', OrderQue, self.handle_order_que)
        self.ac = actionlib.SimpleActionClient('delivery', DeliveryAction)
        print "Waiting for delivery Actions Server"
        self.ac.wait_for_server()
        print "Waiting for segmentation data"
        self.parameters = rospy.get_param('web_order')  # try wait
        print "Ready to Accept Orders"

    def handle_order_que(self, req):
        room_id = self.get_room(pixels=req.pixels, mapsegs=self.parameters["segs"])
        if room_id == (0 or 255):
            accepted = False
            print "invalid destination"
            self.ac.get_result().Error = "Invalid Selection"
        pickup = self.get_param_by_id(param="pickup_positions", param_id=req.item)
        target = self.get_param_by_id(param="destinations", param_id=room_id)
        #Todo: append clicked pose
        print target
        print "Appending %s with Priority %d and destination x: %F y: %F to Que  " % (
            req.item, req.priority, target[0][0], target[0][1])
        goal = DeliveryGoal()
        temp_poses = OrderQueResponse() #non list poses only available from Response
        temp_poses.room_pose1.position.x = target[0][0] #two postions
        temp_poses.room_pose1.position.y = target[0][1]
        goal.pickup_poses.append(temp_poses.room_pose1)
        temp_poses.room_pose2.position.x = pickup[1] #single position TODO: get amount of positions for different cases
        temp_poses.room_pose2.position.y = pickup[0]
        goal.destinations.append(temp_poses.room_pose1)
        goal.item = req.item
        self.ac.send_goal(goal)  # feedbackCB in here?
        accepted = self.ac.wait_for_result()
        print "Action Returned Error: %s in state: %d " % (self.ac.get_result().Error, self.ac.get_result().state)

    def get_room(self, pixels, mapsegs):
        pixfield = mapsegs["width"] * (pixels[1] - 1) + (pixels[0] - 1)
        return mapsegs["data"][pixfield]

    def get_param_by_id(self, param, param_id):
        try:
            return self.parameters[param][str(param_id)]
        except:
            print "param not found"
            return 0

    def shutdown(self):
        print "\n shutdown order que"
        self.ac.cancel_all_goals
        self.s.shutdown()


if __name__ == "__main__":
    rospy.init_node('order_que')
    server = QueServer()
    rospy.spin()

