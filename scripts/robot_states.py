#!/usr/bin/env python3

"""
.. module:: robot_states
    :platform: ROS
    :synopsis: Python module for the node manager class.

.. moduleauthor:: Sara Sgambato s4648592@studneti.unige.it

This node has a class in which two services are defined: one to get the current robot pose and one to set it.
Also, the class defines a publisher to notify that the battery is low.
"""

import threading
import rospy
from Assignment_1 import architecture_name_mapper as anm
from std_msgs.msg import Bool
from Assignment_1.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse

# A tag for identifying logs producer
LOG_TAG = anm.NODE_ROBOT_STATE

# constant value representing the battery usage time
BATTERY_TIME = 120

class RobotState:
    """
    Class implementing the services or the robot position and the publisher for the battery level.
    """

    def __init__(self):
        # Initialise this node
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialise robot position
        self._pose = None
        # Initialise battery level
        self._battery_low = False
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)
        # Start publisher on a separate thread
        th = threading.Thread(target=self._is_battery_low)
        th.start()
        # Log information
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f'`{anm.SERVER_SET_POSE}`, and topic {anm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    def set_pose(self, request):
        """
        Function implementing the 'robot/set_pose' service.

        Args:
            request(Point): current robot position to be set

        Returns:
            SetPoseResponse(): empty responce
        """

        if request.pose is not None:
            # Store the new current robot position
            self._pose = request.pose
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        return SetPoseResponse()

    def get_pose(self, request):
        """
        Function implementing the 'robot/get_pose' service.
        
        Args:
            request(Point): given by th client as empty, it is not used
            
        Returns:
            responce(GetPoseResponce): current robot position
        """

        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        # Create the response with the robot pose and return it
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def _is_battery_low(self):
        """
        Fucntion that publishes the changes of the battery level.
        
        Args:
            None
            
        Returns:
            None
        """

        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        self._battery_notifier(publisher)

    def _battery_notifier(self, publisher):
        """
        Function that publishes when the battery changes state (high/low) based on a constant delay.
        
        Args:
            publisher(Publisher): publisher for the message,
            
        Returns:
            None
        """

        while not rospy.is_shutdown():
            # Publish battery level: 'True' if the battery is low, 'False' otherwise
            publisher.publish(Bool(self._battery_low))
            # Simulate battery usage
            rospy.sleep(BATTERY_TIME)
            # Change battery state
            self._battery_low = not self._battery_low

if __name__ == "__main__":
    # Instantiate the node manager class and wait
    RobotState()
    rospy.spin()