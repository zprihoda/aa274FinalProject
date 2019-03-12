#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TBMarker():
    def __init__(self):
        # initialize node
        rospy.init_node('turtlebot_marker', anonymous=True)

        # publishers
        self.marker_pub = rospy.Publisher('robotMarker', Marker, queue_size=10)

        ## initialize markers
        # robot body
        self.body = Marker()
        self.body.header.frame_id = '/base_footprint'
        self.body.header.stamp    = rospy.get_rostime()
        self.body.ns = "robot_marker"
        self.body.id = 0
        self.body.type = Marker.CYLINDER
        self.body.action = 0
        self.body.pose.position.x = 0
        self.body.pose.position.y = 0
        self.body.pose.position.z = 0
        self.body.scale.x = 0.15
        self.body.scale.y = 0.15
        self.body.scale.z = 0.3
        self.body.color.r = 1.0
        self.body.color.g = 0.0
        self.body.color.b = 0.0
        self.body.color.a = 1.0  # alpha
        self.body.lifetime = rospy.Duration(0)

        # left wheel
        self.lwheel = Marker()
        self.lwheel.header.frame_id = '/base_footprint'
        self.lwheel.header.stamp    = rospy.get_rostime()
        self.lwheel.ns = "robot_marker"
        self.lwheel.id = 1
        self.lwheel.type = Marker.CYLINDER
        self.lwheel.action = 0
        self.lwheel.pose.position.x = 0
        self.lwheel.pose.position.y = -0.15/2
        self.lwheel.pose.position.z = 0.01
        self.lwheel.scale.x = 0.05
        self.lwheel.scale.y = 0.05
        self.lwheel.scale.z = 0.025
        self.lwheel.pose.orientation.x = 1.0;
        self.lwheel.pose.orientation.y = 0.0;
        self.lwheel.pose.orientation.z = 0.0;
        self.lwheel.pose.orientation.w = 1.0;
        self.lwheel.color.r = 0.0
        self.lwheel.color.g = 0.0
        self.lwheel.color.b = 1.0
        self.lwheel.color.a = 1.0  # alpha
        self.lwheel.lifetime = rospy.Duration(0)

        # right wheel
        self.rwheel = Marker()
        self.rwheel.header.frame_id = '/base_footprint'
        self.rwheel.header.stamp    = rospy.get_rostime()
        self.rwheel.ns = "robot_marker"
        self.rwheel.id = 2
        self.rwheel.type = Marker.CYLINDER
        self.rwheel.action = 0
        self.rwheel.pose.position.x = 0
        self.rwheel.pose.position.y = 0.15/2
        self.rwheel.pose.position.z = 0.01
        self.rwheel.scale.x = 0.05
        self.rwheel.scale.y = 0.05
        self.rwheel.scale.z = 0.025
        self.rwheel.pose.orientation.x = 1.0;
        self.rwheel.pose.orientation.y = 0.0;
        self.rwheel.pose.orientation.z = 0.0;
        self.rwheel.pose.orientation.w = 1.0;
        self.rwheel.color.r = 0.0
        self.rwheel.color.g = 0.0
        self.rwheel.color.b = 1.0
        self.rwheel.color.a = 1.0  # alpha
        self.rwheel.lifetime = rospy.Duration(0)

        # heading marker
        self.heading = Marker()
        self.heading.header.frame_id = '/base_footprint'
        self.heading.header.stamp    = rospy.get_rostime()
        self.heading.ns = "robot_marker"
        self.heading.id = 3
        self.heading.type = Marker.ARROW
        self.heading.action = 0
        self.heading.points =  [Point(0,0,0.3), Point(0.15/2,0,0.3)]
        self.heading.scale.x = 0.02
        self.heading.scale.y = 0.02
        self.heading.scale.z = 0.02
        self.heading.color.r = 0.0
        self.heading.color.g = 1.0
        self.heading.color.b = 0.0
        self.heading.color.a = 1.0  # alpha
        self.heading.lifetime = rospy.Duration(0)



    def publish(self):
        """ update markers """
        self.marker_pub.publish(self.body)
        self.marker_pub.publish(self.lwheel)
        self.marker_pub.publish(self.rwheel)
        self.marker_pub.publish(self.heading)


    def run(self):
        """ main loop """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    marker = TBMarker()
    marker.run()
