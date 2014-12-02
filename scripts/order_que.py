#!/usr/bin/env python

from cob_web_delivery.srv import *
import rospy

def handle_order_que(req):
    print "Appending %s with Priority %u to Que %s "%(req.item, req.priority)
    return OrderQueResponse(true)

def order_que_server():
    rospy.init_node('order_que')
    s = rospy.Service('order_que', OrderQue, handle_order_que)
    print "Ready to Accept Order"
    rospy.spin()

if __name__ == "__main__":
    order_que_server()
