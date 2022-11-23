# ExpRoLab Assignment1

## Introduction
The aim of this assignment consists in the development of an algorithm to control the behavior of a robot in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations of the environment and stay there for some time.

## Installation & Running
In order to make the code function correctly, the user should run the following line in the shell to install `xterm`
```sh
$ sudp apt-get install xterm
```
Then, after having cloned and built the repository inside the ROS workspace, the user can run the following line in the shell
```sh
$ rosrun Assignment_1 assignment_launch
```
After this, another terminal should open displaying the node `behavior`.

## Working Hypothesis & Environment
The location which was used to simulate the environment is the following.
<p align="center">
<img src="https://github.com/sarasgambato/ExpRoLab_Assignment1/blob/master/images/environment.png" width=30%, height=30%>
</p>

There are two main classes:
- `LOCATION`: can be either a `ROOM` (location with only one door) or a `CORRIDOR` (location with *at least* one door)
- `DOOR`: individual that connects two locations

The requirements were:
1. The robot should mainly check the corridors
2. If a reachable room has not been visited for more than a certain amount of time, the robot should visit it
3. When the battery is low, the robot must go to room E to recharge

In order to accomplish all these objectives, a few hypothesis were made by the author:
- Room E is only visited when the robot needs to recharge, hence it is not checked for urgency
- If there are no urgent rooms, then the robot continuously checks the corridors C1 and C2
- Corridors C1 and C2 are not checked for urgency, given that the robot mainly stays in the corridor and also every time the robot checks a room, then it inevitably has to go check the connected corridor

## Authors & Contacts
[Sara Sgambato](https://github.com/sarasgambato)

sarasgambato@yahoo.it (personal email)

s4648592@studenti.unige.it (institutional email)
