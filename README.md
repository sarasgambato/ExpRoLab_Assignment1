# ExpRoLab Assignment1

## Introduction
The aim of this assignment consists in the development of an algorithm to control the behavior of a robot in a indoor environment with doors and corridors for surveillance purposes. The robot’s objective is to visit the different locations of the environment and stay there for some time.

The requirements were:
1. The robot should spawn in a room dedicated to recharging
2. The robot should mainly check the corridors
3. If a reachable room has not been visited for more than a certain amount of time, the robot should visit it
4. As soon as the signal of low battery is received, the robot must go to the recharging room

## Software architecture

## Installation & running
Given that the author decided to use the files [planner.py](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/planner.py) and [controller.py](https://github.com/buoncubi/arch_skeleton/blob/main/scripts/controller.py) from the repository [arch_skeleton](https://github.com/buoncubi/arch_skeleton), and [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl) from the repository [topological_map](https://github.com/buoncubi/topological_map) (both repositories by author [Luca Buoncompagni](https://github.com/buoncubi)), the user must clone the mentioned repositories and the current one in the ROS workspace:
```sh
git clone https://github.com/buoncubi/arch_skeleton
git clone https://github.com/buoncubi/topological_map
git clone https://github.com/sarasgambato/ExpRoLab_Assignment1
```
### Using roslaunch
In order to make the code function correctly, the user should run the following line in the shell to install `xterm`, if not already installed:
```sh
sudo apt-get update
sudp apt-get -y install xterm
```
Then, after having done `catkin_make` inside the ROS workspace, the user can run the project by running the following line in the shell:
```sh
roslaunch Assignment_1 assignment_launch.launch
```
After this, another terminal should open displaying the node `behavior`.

The author decided not to display the behavior of the other nodes. If the user wants to, it is sufficient to modify the launch file [assignment_launch.launch](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/launch/assignment_launch.launch) inside the `launch` folder by adding the line `launch-prefix = "xterm -fa 'Monospace' -fs 10 -e` when launching a node. For example, if the user wants to see the behavior of the `planner` node:
```xml
<node pkg = "arch_skeleton"  
          type = "planner.py"      
          name = "planner"   
          launch-prefix = "xterm -fa 'Monospace' -fs 10 -e"
    > </node>
```

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

# Terminal tab 4, run the robot states node
rosrun Assignment_1 robot_states.py

# Terminal tab 5, run the behavior node
rosrun Assignment_1 behavior.py
```

## System's features
### Environment
To load the environment, the script `toad_ontology.py` was used, in which the user is asked to insert the number of corridors and the number of rooms per corridor. Obviously, one can put as many rooms/corridors as they want; the author decided to build the environment that was suggested by the assignment, which is the following:
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/environment.png" width=30%, height=30%>
</p>

There are two main classes:
- `LOCATION`: can be either a `ROOM` (location with one door) or a `CORRIDOR` (location with *at least* one door)
- `DOOR`: individual that connects two locations

### <a id="req"></a> Working hypothesis
In order to accomplish all these objectives, a few hypothesis were made by the author:
- If there are no urgent rooms, then the robot continuously checks the corridors.
- Corridors are not checked for urgency, given that the robot mainly stays in the corridor and also because every time the robot checks a room, then it inevitably has to go check the connected corridor.
- The battery time has been set at 2 minutes, after which the robot must go to room E. The downside of choosing an autonomy time this high is that the robot will take 2 minutes also to recharge. However, the author considered this a necessity for simulation purposes, in order to see if the algorithm worked in a proper way (i.e. the robot check only the corridors until some room becomes urgent). If the user wants to change the battery time, it is sufficient to change the value of the variable `BATTERY_TIME` in the script `robot_states.py`. When the robot needs to recharge, if room E is not directly reachable, it moves randomly until room E is reachable.
- the `urgencyThreshold` was set to 50 seconds for simulation purposes, i.e. if the threshold was to small, the robot found itself stuck in a corridor continuosly checking the rooms it was connected to.

### Limitations


### Possible technical improvement
As mentioned in the [working hypothesis section](#req), when the robot needs to recharge it moves randomly until it reaches room E. To simulate movement, the author put a `rospy.sleep(5)`, which could be improved by using a service.

Moreover, the `urgencyThreshold` is high, and the more rooms a corridor has, the more it should be high with the current algorithm implementation, because of the reason mentioned in the [previous section](#req). One poossible improvement could be to make the robot aware that there are more urgent rooms to check, even if they are not directly reachable, and make it go there with the same method used to reach the recharging room.

## Authors & Contacts
[Sara Sgambato](https://github.com/sarasgambato)

sarasgambato@yahoo.it (personal email)

s4648592@studenti.unige.it (institutional email)
