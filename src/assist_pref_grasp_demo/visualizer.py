#!/usr/bin/env python3


import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

class Visualizer(object):
    def __init__(self,):
        self.rviz_publisher = rospy.Publisher('visualization_marker_array',MarkerArray,queue_size=50)
        