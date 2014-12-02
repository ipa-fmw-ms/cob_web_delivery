#!/usr/bin/env python

from cob_web_delivery.srv import *
import rospy

def handle_order_que(req):
    d = rospy.Duration.from_sec(7.1)
    rospy.sleep(d)
    print "Appending %s with Priority %d and destination x: %F y: %F to Que  "%(req.item, req.priority, req.target_pose1.position.x, req.target_pose1.position.y)
    return OrderQueResponse(True)

def order_que_server():
    rospy.init_node('order_que')
    s = rospy.Service('order_que', OrderQue, handle_order_que)
    print "Ready to Accept Order"
    rospy.spin()

if __name__ == "__main__":
    order_que_server()
