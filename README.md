# multi_mobile_manipulation_control

## Summary

This package contains scripts that use [*actionlib*](http://wiki.ros.org/actionlib) client and server scripts to send trajectories to each TurtleBot's base and arm simultaneously.

## [Physical Robot] Instructions for Executing Multi-Robot Trajectories

### Step 0: Follow *turtlebot3_manipulation* package instructions

Prior to following the steps below, make sure you have followed all the instructions in the [__turtlebot3_manipulation__](https://github.com/JBVAkshaya/turtlebot3_manipulation) repository first.

### Step 1: Launch server scripts for the base and arm

On __your PC__, launch the movebase and movearm server scripts __in separate terminals__. Assuming you are in the `/src` directiory for the package, you would run:

```rosrun multi_mobile_manipulation_control movearm_server.py```

```rosrun multi_mobile_manipulation_control movebase_server.py```

[__*Add note regarding making the file executable?*__]

### Step 2: Run the multi-threaded client script

On __your PC__, run the following client script to simultaneously execute trajectories on the bases and arms of the TurtleBots.

```rosrun multi_mobile_manipulation_control multi_mobile_manipulator_client_threading.py```

### Notes:

By default, the scripts listed in the instructions above assume you are using *two* TurtleBots. The scripts can be easily modified to accomodate different numbers of robots, but for simplicity, below are the scripts you should run if you are testing on *one* or *three* TurtleBots.

__If using one TurtleBot__:

* In __Step 1__, replace `movearm_server.py` with `single_ns_movearm_server.py` and `movebase_server.py` with `single_ns_movebase_server.py`.

* In __Step 2__, replace `multi_mobile_manipulator_client_threading.py` with `single_ns_multi_mobile_manipulator_client_threading.py`.

__If using three TurtleBots__:

* In __Step 1__, replace `movearm_server.py` with `three_tb3_movearm_server.py` and `movebase_server.py` with `three_tb3_movebase_server.py`.

* In __Step 2__, replace `multi_mobile_manipulator_client_threading.py` with `three_tb3_multi_mobile_manipulator_client_threading.py`.
 

## [Simulation] Instructions for Executing Multi-Robot Trajectories

[Coming soon...]
