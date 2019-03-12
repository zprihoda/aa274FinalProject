#!/usr/bin/env python

"""
COMMENT
"""

import rospy
from geometry_msgs.msg import PointStamped, Point, Pose2D
import tf
from aa274_final.msg import DetectedObject
import numpy as np
import numpy.linalg as npl

class CarTracker():

    def __init__(self):

        # Init Node
        rospy.init_node('CarChaser', anonymous=True)

        # Subscribers
        rospy.Subscriber('/detector/truck', DetectedObject,
            self.detected_car_callback, queue_size=10)
        self.trans_listener = tf.TransformListener()

        # Publishers
        self.carMapCoords_pub = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

    def getMapCoords(self, r, th):
        """ Convert r and theta to map coordinates """
        dx = r * np.cos(th)
        dy = r * np.sin(th)
        point = PointStamped()
        point.point.x = dx
        point.point.y = dy
        point.header.frame_id = '/camera'
        point.header.stamp = rospy.Time(0)

        self.trans_listener.waitForTransform('/map','/camera',rospy.Time(0),rospy.Duration(4.0))
        p_out = self.trans_listener.transformPoint('/map',point)
        return p_out


    def detected_car_callback(self, msg):
        dist_car = msg.distance
        thetaleft = ob_msg.thetaleft
        thetaright = ob_msg.thetaright
        if thetaright > np.pi:
            thetaright -= 2*np.pi
        theta = (thetaleft+thetaright)/2
        p_out = self.getMapCoords(dist_car,theta)

        car_pose =  Pose2D()
        car_pose.x = p_out.point.x
        car_pose.y = p_out.point.y
        car_pose.theta = 0

        self.carMapCoords_pub.publish(car_pose)


    def run(self):
        """ main loop """
        rospy.spin()


if __name__ == "__main__":
    tracker = CarTracker()
    tracker.run()