#!/usr/bin/env python

"""
FOr testing:
# manually publish object locations
rostopic pub -r 10 /objectLocations aa274_final/ObjectLocations '{names: [apple,banana], x: [2.0,0.25],  y: [0.25,1.5]}'

"""

from argparse import ArgumentParser
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from aa274_final.msg import DetectedObject, ObjectLocations
import tf
import math
from enum import Enum
import numpy as np

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    FOODNAV = 7
    FOODPICKUP = 8


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):

        rospy.init_node('turtlebot_supervisor', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.home = None    # will be set on first state update
        self.food_index = 0
        self.mode = Mode.IDLE
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.stop_loc_list=[]

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        rospy.Subscriber('/objectLocations', ObjectLocations, self.food_list_callback)
        #Subscribe to the stop sign locations.
        rospy.Subscriber('/stopLocations', ObjectLocations, self.stop_list_callback)
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)

        rospy.Subscriber('/stoplist_request',String,self.stop_request_callback)
    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        if self.home == None:
            self.home = [self.x,self.y,self.theta]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:

            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

    def stop_list_callback(self, msg):
        #store list of all stops
        for i in range(len(msg.names)):
            self.stop_loc_list += np.array([msg.x[i], msg.y[i]])
            
    def near_a_stop(self):
        #return true if we are near a stop based on List of stops, else return false
        n=len(self.stop_loc_list)
        distance=0.1;
        if n==0:
            return False
        for k in range(n):
            d=np.sqrt((self.x-self.stop_loc_list[0][0])^2+(self.y-self.stop_loc_list[0][1])^2)
            if d<distance:
                return True
        return False
                
        
    
    def food_list_callback(self, msg):
        #store list of all foods
        self.food_loc_dict = {}
        for i in range(len(msg.names)):
            food = msg.names[i]
            self.food_loc_dict[food] = [msg.x[i], msg.y[i]]

    def delivery_request_callback(self, msg):
        self.food_pickup_list = msg.data.split(',')
        self.pickup_idx = 0
        self.mode = Mode.FOODNAV
        
    def stop_request_callback(self, msg):
        boolean=self.near_a_stop()
        if boolean:
            self.mode=Mode.STOP

    def set_food_pickup_loc(self):
        while True:
            food = self.food_pickup_list[self.pickup_idx]
            if food not in self.food_loc_dict:
                print food + ' not in tracked objects.  Skipping'
                self.pickup_idx += 1
                if self.pickup_idx == len(self.food_pickup_list): # reached end of list
                    self.return_home()
                    self.mode = Mode.NAV
                    return
            else:
                break
        self.x_g, self.y_g = self.food_loc_dict[food]
        self.theta_g = 0

    def init_pickup(self):
        self.pickup_start = rospy.get_rostime()
        self.mode = Mode.FOODPICKUP

    def has_pickedup(self):
        """ checks if pickup maneuver is over """
        return (self.mode == Mode.FOODPICKUP and (rospy.get_rostime()-self.pickup_start)>rospy.Duration.from_sec(STOP_TIME))

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """
        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def pos_close_to(self,x,y):
        """ checks if the robot is at a pose within some threshold """
        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def return_home(self):
        self.x_g = self.home[0]
        self.y_g = self.home[1]
        self.theta_g = self.home[2]

    def update_state(self):
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
                if self.home == None:
                    self.home = [self.x,self.y,self.theta]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass


    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        self.update_state()

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.FOODNAV:
            self.set_food_pickup_loc()
            if self.pos_close_to(self.x_g,self.y_g):
                self.init_pickup()
            else:
                self.nav_to_pose()

        elif self.mode == Mode.FOODPICKUP:
            if self.has_pickedup():
                self.pickup_idx +=1
                if self.pickup_idx == len(self.food_pickup_list):
                    self.return_home()
                    self.mode = Mode.NAV
                else:
                    self.mode = Mode.FOODNAV
            else:
                self.stay_idle()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':

    sup = Supervisor()
    try:
        sup.run()
    except KeyboardInterrupt:
        sup.stay_idle() # send command to stop


