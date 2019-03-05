#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from aa274_final.msg import ObjectLocations

class LTMarker():
    def __init__(self):
        # initialize node
        rospy.init_node('location_tracker_marker', anonymous=True)

        # Subscribers
        rospy.Subscriber('/objectLocations', ObjectLocations, self.obj_loc_callback)
        self.received = False

        # publishers
        self.marker_pub = rospy.Publisher('locationTrackerMarker', Marker, queue_size=10)

        ## initialize markers
        # robot body
        self.tmp = Marker()
        self.tmp.header.frame_id = '/map'
        self.tmp.header.stamp    = rospy.get_rostime()
        self.tmp.ns = "location_marker"
        self.tmp.id = 0
        self.tmp.type = Marker.CYLINDER
        self.tmp.action = 0
        self.tmp.pose.position.x = 0
        self.tmp.pose.position.y = 0
        self.tmp.pose.position.z = 0
        self.tmp.scale.x = 0.1
        self.tmp.scale.y = 0.1
        self.tmp.scale.z = 0.01
        self.tmp.color.r = 0.0
        self.tmp.color.g = 1.0
        self.tmp.color.b = 0.0
        self.tmp.color.a = 1.0  # alpha
        self.tmp.lifetime = rospy.Duration(0)

    def obj_loc_callback(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.names = msg.names
        self.received = True

    def publish(self):
        """ update markers """
        if self.received:
            for i in range(len(self.names)):
                mrkr = self.tmp
                mrkr.pose.position.x = self.x[i]
                mrkr.pose.position.y = self.y[i]
                mrkr.id = i
                self.marker_pub.publish(mrkr)

    def run(self):
        """ main loop """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    marker = LTMarker()
    marker.run()
