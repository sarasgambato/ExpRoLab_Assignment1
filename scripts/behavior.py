#!/usr/bin/env python3 

"""
.. module:: behavior
    :platform: ROS
    :synopsis: Python module for the initialization and definition of the finite state machine.
    
.. moduleauthor:: Sara Sgambato s4648592@studenti.unige.it

This module defines the behavior of the Finite State Machine.
"""

import rospy
import random
import smach_ros
from smach import StateMachine, State
from load_ontology import LoadMap
from Assignment_1 import architecture_name_mapper as anm
from helper import InterfaceHelper, BehaviorHelper
from Assignment_1.msg import Point, ControlGoal, PlanGoal
from os.path import dirname, realpath

# list of states in the machine
STATE_INIT = 'INITIALIZE_MAP'
STATE_DECISION = 'DECIDE_LOCATION'
STATE_PLAN_TO_RANDOM_POSE = 'PLAN_TO_RANDOM_POSE'
STATE_GO_TO_RANDOM_POSE = 'GO_TO_RANDOM_POSE'
STATE_NORMAL = 'NORMAL'
STATE_RECHARGING = 'RECHARGING'

# list of transition states
TRANS_INITIALIZED = 'everything_loaded'
TRANS_DECIDED = 'target_acquired'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_RECHARGING = 'recharging'
TRANS_RECHARGED = 'recharged'
TRANS_WENT_RANDOM_POSE = 'went_to_random_pose'

# constant values
SLEEP_TIME = 0.3

class LoadOntology(State):
    """
    Class related to the state that loads the ontology.
    """

    def __init__(self):
        State.__init__(self, outcomes=[TRANS_INITIALIZED])

    def execute(self, userdata):
        """
        Function that is executed when entering the INITIALIZE_MAP state of the fsm. 
        It loads the topological map and transitions to the state NORMAL.

        Args: 
            userdata

        Returns: 
            Str: 'everything_loaded' once the map has been intialized
        """

        LoadMap()
        print('The map is loaded')
        return TRANS_INITIALIZED

class Recharging(State):
    """
    Class related to the state that recharges the robot's battery.
    """

    def __init__(self, helper):
        State.__init__(self, outcomes=[TRANS_RECHARGED])
        self._helper = helper

    def execute(self, userdata):
        """
        Function that is executed every time the fsm enters the RECHARGING state.
        It recharges the battery of the robot and transitions to the state NORMAL.
        
        Args:
            userdata
            
        Returns:
            Str: 'recharged' once the battery is recharged
        """

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if not self._helper.is_battery_low():
                    self._helper.reset_states()
                    return TRANS_RECHARGED
            finally:
                self._helper.mutex.release()
            rospy.sleep(SLEEP_TIME)

class DecideTarget(State):
    """
    Class related to the state that reasons about the next location that the robot will check.
    """

    def __init__(self, helper, sm_helper):
        self._helper = helper
        self._sm_helper = sm_helper
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes=[TRANS_DECIDED, TRANS_RECHARGING], 
                       output_keys=['current_pose', 'choice', 'random_plan'])

    def execute(self, userdata):
        """
        Function that is executed every time the fsm enters the DECIDE_LOCATION state.
        It decides in which location to go next based on some criteria.

        Args:
            userdata

        Returns:
            Str: 'recharging' if the battery is low, 'target_acquired' otherwise
        """

        # plan a random goal with some number of via points to simulate movement
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]), 
                            y = random.uniform(0, self.environment_size[1]))
        self._helper.planner_client.send_goal(goal)
        current_pose, choice = self._sm_helper.decide_location()

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low(): #higher priority
                    self._helper.planner_client.cancel_goals()
                    self._sm_helper.recharge(current_pose)
                    return TRANS_RECHARGING
                
                if self._helper.planner_client.is_done():
                    userdata.choice = choice
                    userdata.current_pose = current_pose
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED

            finally:
                self._helper.mutex.release()
            rospy.sleep(SLEEP_TIME)

class CheckTarget(State):
    """
    Class related to the state that check the location the robot is in.
    """

    def __init__(self, helper, sm_helper):
        self._helper = helper
        self._sm_helper = sm_helper
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes=[TRANS_WENT_RANDOM_POSE, TRANS_RECHARGING], 
                       input_keys=['current_pose', 'choice', 'random_plan'])

    def execute(self, userdata):
        """
        Function that is executed every time the fsm enters the GO_TO_RANDOM_POSE state.
        It makes the robot go to the wanted location and checks it.
        
        Args:
            userdata
            
        Returns:
            Str: 'recharging' if the battery is low, 'went_to_random_pose' otherwise
        """

        # send the goal to the controller and check that the via points are reached
        plan = userdata.random_plan
        current_pose = userdata.current_pose
        goal = ControlGoal(via_points=plan)
        self._helper.controller_client.send_goal(goal)

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low():
                    self._helper.controller_client.cancel_goals()
                    self._sm_helper.recharge(current_pose)
                    return TRANS_RECHARGING
                    
                if self._helper.controller_client.is_done():
                    choice = userdata.choice
                    self._sm_helper.check_location(current_pose, choice)
                    rospy.sleep(5)
                    return TRANS_WENT_RANDOM_POSE
            finally:
                self._helper.mutex.release()
            rospy.sleep(SLEEP_TIME)

def main():
    """
    Main function which initalizes the Finite State Machine node 'behavior'.
    Two helpers are defined to help the state macine take decisions and to manage the battery.
    The fsm is hierarchical as to create a more modular and robust architecture.
    """
    rospy.init_node('behavior', log_level = rospy.INFO)

    # define the helper that manages the battery stimulus
    helper = InterfaceHelper()

    # define the helper for the states of the fsm
    sm_helper = BehaviorHelper()

    sm_main = StateMachine([])
    with sm_main:
        StateMachine.add(STATE_INIT, LoadOntology(),
                         transitions={TRANS_INITIALIZED: STATE_NORMAL})

        sm_normal = StateMachine(outcomes=[TRANS_BATTERY_LOW])
        with sm_normal:
            StateMachine.add(STATE_DECISION, DecideTarget(helper, sm_helper),
                            transitions={TRANS_DECIDED: STATE_GO_TO_RANDOM_POSE,
                                        TRANS_RECHARGING: TRANS_BATTERY_LOW})
            StateMachine.add(STATE_GO_TO_RANDOM_POSE, CheckTarget(helper, sm_helper),
                            transitions={TRANS_WENT_RANDOM_POSE: STATE_DECISION,
                                        TRANS_RECHARGING: TRANS_BATTERY_LOW})

        StateMachine.add(STATE_NORMAL, sm_normal,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING})   
        StateMachine.add(STATE_RECHARGING, Recharging(helper),
                         transitions={TRANS_RECHARGED: STATE_NORMAL}) 

    # Create and start the introspection server for visualization 
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   
if __name__ == '__main__':
    main()