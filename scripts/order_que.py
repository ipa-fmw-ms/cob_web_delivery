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
        print "Waiting for segmentation data"
        self.parameters = rospy.get_param('web_order')  # try wait
        print "Waiting for delivery Actions Server"
        self.ac.wait_for_server()
        print "Ready to Accept Orders \n"

    def handle_order_que(self, req):
        servback = OrderQueResponse()
        room_id = self.get_room(pixels=req.pixels, mapsegs=self.parameters["segs"])
        print "RoomId: %i" % room_id
        if room_id == 0:
            print "invalid destination"
            #self.ac.get_result().Error = "Invalid Selection"
            servback.accepted = False
            servback.room = "no Room"
            return servback
        pickups = self.assign_poses(self.get_param_by_id(param="pickup_positions", param_id=req.item))
        print "pickup assign", pickups
        targets = self.assign_poses(self.get_param_by_id(param="destinations", param_id=room_id))
        print "targets assign", targets
        # Todo: append clicked pose

        goal = DeliveryGoal()
        goal.item = req.item
        goal.destinations = targets
        goal.pickup_poses = pickups

        self.ac.send_goal(goal)  # feedbackCB in here?

        accepted = self.ac.wait_for_result()
        #print "Action Returned Error: %s in state: %d " % (self.ac.get_result().Error, self.ac.get_result().state)
        print "action returned", self.ac.get_result()
        print "\n"

    def assign_poses(self, pose_list):
        array = []
        if isinstance(pose_list[0], float):
            array.append(Pose2D(**dict(zip(['x', 'y', 'theta'], pose_list))))
        else:
            for i in range(len(pose_list)):
                array.append(Pose2D(**dict(zip(['x', 'y', 'theta'], pose_list[i]))))
        return array

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

