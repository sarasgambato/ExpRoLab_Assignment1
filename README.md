# ExpRoLab Assignment1

## Introduction
The aim of this assignment consists in the development of an algorithm to control the behavior of a robot in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations of the environment and stay there for some time.

## Installation & Running
### Using roslaunch
In order to make the code function correctly, the user should run the following line in the shell to install `xterm`, if not already installed
```sh
sudo apt-get update
sudp apt-get -y install xterm
```
Then, after having cloned and built the repository inside the ROS workspace, the user can run the code by running the following line in the shell
```sh
roslaunch Assignment_1 assignment_launch.launch
```
After this, another terminal should open displaying the node `behavior`.

The author decided not to display the behavior of the other nodes. If the user wants to, it is sufficient to modify the launch file [assignment_launch.launch](https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/launch/assignment_launch.launch) inside the `launch` folder by adding the line `launch-prefix = "xterm -fa 'Monospace' -fs 10 -e` when launching a node. For example, if the user wants to see the behavior of the planner:
```xml
<node pkg = "Assignment_1"  
          type = "planner.py"      
          name = "planner"   
          launch-prefix = "xterm -fa 'Monospace' -fs 10 -e"
    > </node>
```

### Launching each node manually
To run each node manually, the user should open 5 different tabs and run the following lines:
- Terminal tab 1, start the aRMOR server
```sh
roscore &
rosrun armor execute it.emarolab.armor.ARMORMainService
```
- Terminal tab 2, run the planner node
```sh
rosrun Assignment_1 planner.py
```
- Terminal tab 3, run the controller node
```sh
rosrun Assignment_1 controller.py
```
- Terminal tab 4, run the robot states node
```sh
rosrun Assignment_1 robot_states.py
```
- Terminal tab 5, run the behavior node
```sh
rosrun Assignment_1 behavior.py
```

## Working Hypothesis & Environment
### Environment
The location which was used to simulate the environment is the following.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/environment.png" width=30%, height=30%>
</p>

There are two main classes:
- `LOCATION`: can be either a `ROOM` (location with only one door) or a `CORRIDOR` (location with *at least* one door)
- `DOOR`: individual that connects two locations

### Requirements & Working hypothesis
The requirements were:
1. The robot should spawn in room E, which is dedicated to recharging
2. The robot should mainly check the corridors
3. If a reachable room has not been visited for more than a certain amount of time, the robot should visit it
4. As soon as the signal of low battery is received, the robot must go to room E to recharge

In order to accomplish all these objectives, a few hypothesis were made by the author:
- Room E is only visited when the robot needs to recharge, hence it is not checked for urgency
- If there are no urgent rooms, then the robot continuously checks the corridors C1 and C2
- Corridors C1 and C2 are not checked for urgency, given that the robot mainly stays in the corridor and also every time the robot checks a room, then it inevitably has to go check the connected corridor
- The battery time has been set at 2 minutes, after which the robot must go to room E. The downside of choosing an autonomy time this high is that the robot will take 2 minutes also to recharge. However, the author considered this a necessity for simulation purposes, in order to see if the algorithm worked in a proper way (i.e. the robot check only the corridors until some room becomes urgent). If the user wants to change the battery time, it is sufficient to change the value of the variable `BATTERY_TIME` in the script `robot_states.py`

### System's features


### System's limitations


### Possible technical improvement


## Authors & Contacts
[Sara Sgambato](https://github.com/sarasgambato)

sarasgambato@yahoo.it (personal email)

s4648592@studenti.unige.it (institutional email)
