#!/usr/bin/env python

from cob_web_delivery.srv import *
from cob_web_delivery.msg import *
from move_base_msgs.msg import *
from cob_phidgets.msg import DigitalSensor
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion, Twist

import rospy
import actionlib
import tf


class DeliveryServer:
    # create messages that are used to publish feedback/result
    _feedback = DeliveryFeedback()
    _result = DeliveryResult()

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        print "Starting delivery Server"
        self.deliver_as = actionlib.SimpleActionServer('delivery', DeliveryAction, self.execute, False)
        self.deliver_as.start()
        rospy.Subscriber("FB_phidget", DigitalSensor, self.phidget_fb_cb)
        print "Delivery Server started"
        print "Connecting to move_base ActionServer"
        self.mbac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mbac.wait_for_server()
        print "Connected to MoveBase"

        self.wait_button_fb = False  # not here
        self.wait_button = -1  # not here

    def execute(self, goal):
        print "Got request for %s" % (goal.item)
        self._feedback.state = 0
        self.deliver_as.publish_feedback(self._feedback)
        self.move_base_multiple(goal.pickup_poses)
        print "waiting for product"
        self.wait_fb(0)
        self.move_base_multiple(goal.destinations)
        print "waiting for customer"
        self.wait_fb(0)
        print "delivery successful"
        self._result.success = True
        self._result.Error = "Arrived"
        self._result.state = 1
        self.deliver_as.set_succeeded(self._result)
        print "Action done"

    def move_base_multiple(self, pose_array):
        bgoal = MoveBaseGoal()
        for pose in pose_array:
            bgoal.target_pose = self.pose2d_to_spose(pose)  # todo: check and go to next position, if unreachable
            print "moving to position pose: ",  bgoal.target_pose.pose.position.x, bgoal.target_pose.pose.position.y, bgoal.target_pose.pose.position.theta
            self.mbac.send_goal(bgoal)
            reached = self.mbac.wait_for_result(rospy.Duration.from_sec(10.0))
            if reached:
                return True
            print "failed"
            #rospy.WARN("unreachable")
        print "completely failed"
        print self.mbac.get_result()
        return False


    def pose2d_to_spose(self, pose2d):
        spose = PoseStamped()
        spose.header.frame_id = "map"
        spose.header.stamp = rospy.Time.now()
        spose.pose.position.x = pose2d.x
        spose.pose.position.y = pose2d.y
        orientation = Quaternion(
            **dict(zip(['x', 'y', 'z', 'w'], tf.transformations.quaternion_from_euler(0, 0, pose2d.theta))))
        spose.pose.orientation = orientation
        return spose

    def phidget_fb_cb(self, data):
        if (self.wait_button >= 0):  # only one wait at a time possible
            if data.state[self.wait_button]:
                self.wait_button_fb = True

    def wait_fb(self, button):
        self.wait_button_fb = False
        self.wait_button = button
        while not self.wait_button_fb:
            rospy.sleep(rospy.Duration.from_sec(0.5))  # better
        self.wait_button = -1 #not waiting 
        return True

    def shutdown(self):
        print "\n shutdown delivery manager"
        self.mbac.cancel_all_goals()
        self.deliver_as.need_to_terminate = True
        # self.deliver_as.set_aborted()
        if self.deliver_as.is_active():
            self._feedback.state = 6
            self._result.success = False
            self._result.Error = "Node Killed"
            self._result.state = 6
            self.deliver_as.set_preempted(self._result, 'Node got killed')
            #todo publish aborted state


if __name__ == '__main__':
    rospy.init_node('delivery_server')
    server = DeliveryServer()
    print "Ready to Serve"
    rospy.spin()
  

