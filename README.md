# ExpRoLab Assignment 1 
The code documentation for this repository can be found [here](https://sarasgambato.github.io/ExpRoLab_Assignment1/index.html).

## <a id="intro"></a> Introduction
This repository contains a ROS-based software that implements a Finite State Machine (FSM) to control the behavior of a robot in a indoor environment with rooms, doors and corridors for surveillance purposes. The robot’s objective is to visit the different locations of the environment and check them.

The classes were:
- `LOCATION`: can either be a `ROOM` (location with one door) or a `CORRIDOR` (location with *at least* one door).
- `DOOR`: entity that connects two locations.
- `ROBOT`: entity that moves in the environment.

The requirements were:
1. The robot should spawn in a room dedicated to recharging.
2. The robot should mainly check the corridors.
3. If a reachable room has not been visited for more than a certain amount of time, the robot should visit it.
4. As soon as the signal of low battery is received, the robot must go to the recharging room.

## Software architecture
#### Finite State Machine
The Finite State Machine implemented by the software is shown in the following figure.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/finitestate.png" width=40%, height=40%>
</p>

As can be seen, the author decided to create a hierarchical state machine. In particular, we have the following states:
- **INITIALIZE_MAP**: this state uses the function `load_ontology.py` to create the environment and it does so by asking the user to input the number of corridors and the number of rooms for each corridor. Obviously, the user can create an environment with as many corridors and rooms as prefered, however the author decided to create an environment similar to the one suggested for the assignment, which is shown in [this section](#env).
- **NORMAL**: inner FSM composed of two other states:
    1. **DECIDED_LOCATION**: this state reasons about the location that the robot will visit next. 
    2. **CHECK_TARGET**: this state changes the position of the robot and checks the location. 
- **RECHARGING**: this states recharges the battery of the robot.

#### UML diagram
In the following figure the complete software architecture can be seen. It is important to notice that the node `behavior` communicates with the action servers, with the aRMOR server and with the `robot_state` node through a `helper`, which will be descripted in the following sub-section. 
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/UMLdiag.png" width=70%, height=70%>
</p>

Note that when sending a query to the aRMOR server, only the input parameter `dataprop` is compulsory, the `individual` parameter sometimes is not needed.

#### Temporal diagram
In the following figure the temporal interaction between the software components can be seen.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/temporal.png" width=60%, height=60%>
</p>

The signal of low battery is the one with the highest priority. So, once the signal has been received by the FSM, the software cancels all the other requests to make the robot transition to the state **RECHARGING**. 

In the following section, all the components seen in the diagrams are described in detail.

### Components
#### The `behavior` node and the `helper`
This node initializes the FSM and defines its behavior. In particular, in order not to make a "heavy" code, the author decided to implement a 
`helper.py` script which can be found [here](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/scripts/helper.py), where almost all of the reasoning is done. This script has three classes, each implementing a different helper:
1. **ActionClientHelper** simplifies the implementation of a client for ROS action servers.
2. **InterfaceHelper** manages the synchronization with subscribers and action servers.
3. **BehaviorHelper** communicates with the aRMOR server by sending queries mainly about the robot position, the reachable locations and the urgent ones. 

By doing so, by looking at the `behavior.py` script the user is able to clearly understand how the FSM transistions from one state to another; by looking at the `helper.py` the user can understand how the FSM reasons to make the robot change location, go recharge, etc.

#### The `robot_state` node
This node implements two services, `state/set_pose` and `state/get_pose`, allowing to set and get the current robot position, a knowledge that is shared with the nodes `planner` and `controller` :
- `state/set_pose` requires a `Point` to be set and returns nothing
- `state/get_pose` requires nothing and returns a `Point`

Also, the node implements a publisher of `Boolean` messages into the `state/battery_low` topic: the message is published every time the battery changes state, which is every 120 seconds, and it is equal to `True` if the battery is low, `False` otherwise. The user can change the value of the battery time by changing the constant variable `BATTERY_TIME` in the [architecture_name_matter.py](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/utilities/Assignment_1/architecture_name_mapper.py) script.

Moreover, the node takes the parameter `config/environment_size` (more details are given in [this section](#param)) and, through the `helper`, exploits the `state/set_pose` service to set the intial robot position.

#### The `planner` & `controller` nodes
The user can find a detailed decription of these two nodes in the [README](https://github.com/buoncubi/arch_skeleton/blob/main/README.md) of the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository.

## Installation & running
To correctly use this software, the user must follow these steps to install the required packages/repositories.
1. Given that the author decided to use the files [planner.py](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/planner.py) and [controller.py](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/controller.py) from the repository [arch_skeleton](https://github.com/buoncubi/arch_skeleton), and [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl) from the repository [topological_map](https://github.com/buoncubi/topological_map) (both repositories by author [Luca Buoncompagni](https://github.com/buoncubi)), the user must clone the mentioned repositories and the current one in the ROS workspace.
2. Install the [aRMOR package](https://github.com/EmaroLab/armor) by following [this tutorial](https://github.com/EmaroLab/armor/issues/7).
3. Clone the [armor_py_api](https://github.com/EmaroLab/armor_py_api) repository in your ROS workspace.
4. Install the [SMACH package](http://wiki.ros.org/smach) by running `sudo apt-get install ros-noetic-executive-smach*` or equivalent for your ROS distribution.
5. Run `chmod +x <fine_name>` for each file inside the folder `scripts` of the package `ExpRoLab_Assignment1`.
6. Run `catkin_make` from the root of your ROS workspace.

### Note
The author found some issues with the function proposed in the script [armor_manipulation_client.py](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/scripts/armor_manipulation_client.py) to disjoint the individuals of the ontology. Hence, another function was created in the script as:
```py
def disj_inds(self, inds):
    try:
        res = self._client.call('DISJOINT', 'IND', '', inds)

    except rospy.ROSException:
        raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")

    if res.success:
        return res.is_consistent
    else:
        raise ArmorServiceInternalError(res.error_description, res.exit_code)
```
the user should copy and add this function in the script to make the software work correctly.

### Using roslaunch
In order to make the launcher work correctly, the user should install `xterm` with the command `sudp apt-get -y install xterm`
Then the user can run the software by using `roslaunch`:
```sh
roslaunch Assignment_1 assignment_launch.launch
```
After this, another terminal should open displaying the node `behavior`.

The author decided not to display the behavior of the other nodes. If the user wants to, it is sufficient to modify the launch file [assignment_launch.launch](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/launch/assignment_launch.launch) inside the `launch` folder by adding the line `launch-prefix = "xterm -fa 'Monospace' -fs 10 -e` when launching a node.

### Launching each node manually
To run each node manually, the user should open 5 different tabs and run the following lines:
```sh
# Terminal tab 1, run the aRMOR server
roscore &
rosrun armor execute it.emarolab.armor.ARMORMainService

# Terminal tab 2, run the planner node
rosrun arch_skeleton planner.py

# Terminal tab 3, run the controller node
rosrun arch_skeleton controller.py

# Terminal tab 4, run the robot_state node
rosrun Assignment_1 robot_states.py

# Terminal tab 5, run the behavior node
rosrun Assignment_1 behavior.py
```

### <a id="param"></a> ROS parameters
The software requires the following ROS parameters:
- `config/environment_size`: list of two float numbers, `[x_max, y_max]`. The *x*-th and *y*-th coordinates of the environment will be in the range of `[0, x_max)` and `[0, y_max)` respectively.
- `state/intial_pose`: initial robot position in `[x, y]` coordinates within the `environment_size`.
- `test/random_plan_points`: list of two integer numbers, `[min_n, max_n]` used to randomly chose the number of via points.
- `test/random_plan_time`: list of two float numbers, `[min_time, max_time]` used to randomly chose the time required to compute the next via point of the plan.
- `test/random_motion_time`: list of two float numbers, `[min_time, max_time]` used to randomly
chose the time required to reach the next via point.

## Running code behavior
In the following figure the user can see all the relevant parts showing how the software works
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/init.png" width=49%, height=49%>
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/corr.png" width=49%, height=49%>
</p>
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/urgent.png" width=49%, height=49%>
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/recharge.png" width=49%, height=49%>
</p>

In particular:
- ***Top left***: the software creates the ontology based on the user inputs; note that the software is able to understand when a wrong type input has been received.
- ***Top right***: when there are no reachable urgent rooms, the robot continuosly check the corridors.
- ***Bottom left***: when there are reachable urgent rooms, the robot chooses to checks them one by one, choosing firstly the one with the highest priority, i.e. the one that has not been visited for the longest.
- ***Bottom right***: as soon as the signal of low battery has been received, the software stops the current action to go to the recharging room. In the figure we can see the the robot was in room R3 and it decided to go to corridor C2, however the signal of low battery was received, so, from room R3, the robot started randomly calculating a path to the recharging room.

## System's features
### <a id="env"></a> Environment
The environment used for the simulation is shown in the following figure. 
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/env.png" width=30%, height=30%>
</p>

### <a id="req"></a> Working hypothesis
In order to accomplish all the objectives listed in the [introduction](#intro), a few hypothesis were made by the author:
- If there are no urgent rooms, then the robot continuously checks the corridors.
- Corridors are not checked for urgency, given that the robot mainly stays in the corridor and also because every time the robot checks a room, then it inevitably has to go check the connected corridor.
- The battery time has been set to 2 minutes, after which the robot must go to room E. The downside of choosing an autonomy time this high is that the robot will take 2 minutes also to recharge. However, the author considered this a necessity for simulation purposes, in order to see if the algorithm worked in a proper way (i.e. the robot checks only the corridors until some room becomes urgent). When the robot needs to recharge, if room E is not directly reachable, it moves randomly until room E is reachable.
- the `urgencyThreshold`, a parameter that can be changed in the file [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl), was set to 50 seconds for simulation purposes, i.e. if the threshold was too small, the robot found itself stuck in a corridor continuosly checking the rooms it was connected to.

### Limitations and future technical improvement
As mentioned in the [working hypothesis section](#req), when the robot needs to recharge it moves randomly until it reaches room E. To simulate movement, the author put a `rospy.sleep(BUSY_TIME)`, which blocks all the system but was chosen for sake of simplicity. Thus, this could be improved by using a service.

Another problem is that the robot reaches the recharging room randomly. With this specific type of environment (each room is connected only to one corridor and each corridor is connected to the recharging room) the problem is not relevant, but if, for example, not all corridors where connected to the recharging room, then the robot could find itslef moving randomly for a long time before finding the recharging room. So, the software could be improved by computing a direct path from the robot position to the recharging room.

Moreover, the robot may find itself checking the corridors for a very long period of time because the `urgencyThreshold` is high, and the more rooms a corridor has, the more it should be high with the current software implementation, because of the reason mentioned in the [previous section](#req). One possible improvement could be to make the robot aware that there are more urgent rooms to check, even if they are not directly reachable, and make it go there with the same method used to reach the recharging room.

## Authors & Contacts
[Sara Sgambato](https://github.com/sarasgambato)

sarasgambato@yahoo.it (personal email)

s4648592@studenti.unige.it (institutional email)
