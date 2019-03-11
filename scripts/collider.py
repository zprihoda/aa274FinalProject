#!/usr/bin/env python

import rospy
import os
import tf
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class Collider:

    def __init__(self):
        rospy.init_node('collision_detector', anonymous=True)
        
        self.pub = rospy.Publisher('/collision', Bool, queue_size=0)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        
        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_time = rospy.Time(0)
        
        # current velocity
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.valid_vel = True
        self.cmd_vel = Twist()

        # if using gazebo, then subscribe to model states
        self.trans_listener = tf.TransformListener()

        # set default ros parameters

    def updateVel(self):

        try:
            origin_frame = "/map" # "/world" # if mapping else "/odom"
            (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rotation)
            
            lin_diff = np.sqrt((self.x - translation[0])**2 + (self.y - translation[1])**2)
            ang_diff = ((euler[2] - self.theta + math.pi) % (2*math.pi)) - math.pi # handle wrap arounds
            dur = rospy.Time.now() - self.pose_time
            time_diff = dur.to_sec()

            self.lin_vel = lin_diff / time_diff
            self.ang_vel = ang_diff / time_diff
            
            self.x = translation[0]
            self.y = translation[1]
            self.theta = euler[2]
            self.pose_time = rospy.Time.now()

            self.valid_vel = time_diff <= rospy.get_param("pos_dur_thres", 0.1)

            return True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
            

    def cmd_callback(self, msg):
        self.cmd_vel = msg

    def collided(self):
        """ compute if command velocity >> movement from tf """

        # command vel
        lin_cmd = np.sqrt(self.cmd_vel.linear.x**2 + self.cmd_vel.linear.y**2) # y component should be 0
        ang_cmd = self.cmd_vel.angular.z

        # difference in command vs measured speed
        lin_vel_diff = lin_cmd - self.lin_vel
        ang_vel_diff = ((ang_cmd - self.ang_vel + math.pi) % (2*math.pi)) - math.pi

        # check thresholds
        fast_move = lin_vel_diff > rospy.get_param("lin_vel_thres",0.05) 
        fast_turn = abs(ang_vel_diff) > rospy.get_param("ang_vel_thres", math.pi / 4)
        collision = fast_move or fast_turn
        
        # debug
        if fast_move: 
            rospy.loginfo("Linear collision detected with predicted speed {} and command speed {}!".format(self.lin_vel, lin_cmd))
        if fast_turn:
            rospy.loginfo("Angular collision detected with predicted speed {} and command speed {}!".format(self.ang_vel, ang_cmd))
        
        # format as Bool message
        msg = Bool()
        msg.data = collision & self.valid_vel
        
        return msg
    
    
    def run(self):
        r = rospy.get_param("collision_detector_rate",20)
        rate = rospy.Rate(r) # 20 Hz
        while not rospy.is_shutdown():
            success = self.updateVel()
            if not success:
                rospy.loginfo("Failed to update velocity from tf")
            self.pub.publish(self.collided())
            rate.sleep()


if __name__=='__main__':
    c = Collider()
    c.run()
