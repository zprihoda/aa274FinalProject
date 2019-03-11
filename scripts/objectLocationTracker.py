#!/usr/bin/env python

"""
Spawns the LocationTracker node
Takes detections and converts them to map locations
"""

import rospy
from geometry_msgs.msg import PointStamped, Point
import tf
from aa274_final.msg import DetectedObjectList, ObjectLocations
import numpy as np
import numpy.linalg as npl
import os

ERR_THRESH = 0.25   # meters before we make a new object


class ObjectLocationTracker():

    def __init__(self):
        self.object_dict = {}
        self.food_list = self.getFoodList()
        self.detected_flag = False

        # Init Node
        rospy.init_node('LocationTracker', anonymous=True)

        # Sbuscribers
        rospy.Subscriber('/detector/objects', DetectedObjectList,
                         self.detected_objects_callback, queue_size=10)
        self.trans_listener = tf.TransformListener()

        # Publishers
        self.obj_pub = rospy.Publisher('/objectLocations', ObjectLocations, queue_size=10)

    @classmethod
    def getFoodList(cls):
        """ Return list of labels to identify as foods """
        path = '/'.join(os.path.realpath(__file__).split('/')[0:-1])
        with open(path+'/list_foods') as f:
            lines = f.read()
            foods = lines.split()
        return foods

    def getMapCoords(self, r, th):
        """ Convert r and theta to map coordinates """
        dx = r * np.cos(th)
        dy = r * np.sin(th)
        point = PointStamped()
        point.point.x = dx
        point.point.y = dy
        point.header.frame_id = '/base_footprint'
        point.header.stamp = rospy.Time(0)

        self.trans_listener.waitForTransform('/map','/base_footprint',rospy.Time(0),rospy.Duration(4.0))
        p_out = self.trans_listener.transformPoint('/map',point)
        return p_out

    def getObjectIdx(self,lbl,p_out):
        """
        Determines whether an object is already being tracked
        If tracked: returns 0
        If not: returns new idx
        """
        matching_lbls = [k for k in self.object_dict.keys() if lbl in k]
        for tracked_lbl in matching_lbls:
            tracked_pt = self.object_dict[tracked_lbl]
            dist_err = npl.norm([p_out.point.x-tracked_pt[0], p_out.point.y-tracked_pt[1]])
            if dist_err < ERR_THRESH:
                return 0    # already tracked
        return len(matching_lbls)

    def detected_objects_callback(self, msg):
        detected_objects = msg

        for ob_msg in detected_objects.ob_msgs:
            lbl = ob_msg.name
            if lbl in self.food_list:

                dist = ob_msg.distance
                thetaleft = ob_msg.thetaleft
                thetaright = ob_msg.thetaright
                if thetaright > np.pi:
                    thetaright -= 2*np.pi
                theta = (thetaleft+thetaright)/2
                p_out = self.getMapCoords(dist,theta)

                if lbl in self.object_dict.keys():
                    new_flag = self.getObjectIdx(lbl,p_out)
                    if new_flag:
                        lbl += str(new_flag)
                    else:   # is already tracked
                        continue

                self.object_dict[lbl] = [p_out.point.x, p_out.point.y]
                self.detected_flag = True

    def publish(self):
        names = self.object_dict.keys()
        pts = self.object_dict.values()
        x,y = zip(*pts)
        objloc = ObjectLocations()
        objloc.names=names
        objloc.x = x
        objloc.y = y
        self.obj_pub.publish(objloc)

    def run(self):
        """ main loop """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.detected_flag:
                self.publish()
            rate.sleep()


if __name__ == "__main__":
    tracker = ObjectLocationTracker()
    tracker.run()
