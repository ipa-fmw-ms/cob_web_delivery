#!/usr/bin/env python

from cob_web_delivery.srv import *
from cob_web_delivery.msg import *
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion, Twist

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
        pickups = self.assign_poses(self.get_param_by_id(param="pickup_positions", param_id=req.item))
        print "pickup assign", pickups
        targets = self.get_param_by_id(param="destinations", param_id=room_id)
        # Todo: append clicked pose

        print "Appending %s with Priority %d and destination x: %F y: %F to Que  " % (
            req.item, req.priority, targets[0][0], targets[0][1])

        goal = DeliveryGoal()
        test = dict(zip(['x', 'y', 'theta'], targets[0]))

        temp_poses = Pose2D(**test)
        goal.pickup_poses.append(temp_poses)

        #temp_poses.position.x = pickups[1]  #single position TODO: get amount of positions for different cases
        #temp_poses.position.y = pickups[0]

        goal.destinations.append(temp_poses)
        goal.item = req.item
        self.ac.send_goal(goal)  # feedbackCB in here?
        accepted = self.ac.wait_for_result()
        print "Action Returned Error: %s in state: %d " % (self.ac.get_result().Error, self.ac.get_result().state)

    def assign_poses(self, pose_list):

        if isinstance(pose_list[0], float):
            return Pose2D(**dict(zip(['x', 'y', 'theta'], pose_list)))
        else:
            for i in len(pose_list):
                Pose2D(**dict(zip(['x', 'y', 'theta'], pose_list[i])))
            #Todo: append where?




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

