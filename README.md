ROS_PlayerStage_Playground
==========================

This repository contains a basic setup to allow quickly diving into navigation algorithms using Player/Stage inside ROS to simulate a simple robot with a laser ranger and odometry sensors.

Follow the instructions below to get started. If you have not already complete the instructions in the new members tutorials from our GitHub wiki before setting this up.

Get code: (modify directories as necessary)

`cd ~/ros_workspace`

`git clone git@github.com:MST-Robotics/ROS_PlayerStage_Playground.git`

`rosws set ~/ros_workspace/ROS_PlayerStage_Playground`

`source ~/ros_workspace/setup.bash`

Build code:

`rosmake navigation_playground`

Run simulation:

`roslaunch navigation_playground playground.launch`

The blue square represents your robot, it should be just driving in a circle if you didn't modify the code.

Now that you know how to run the code modify navigation_playground/src/Navigation.cpp and add your navigation code in the Navigation::run() function. Feel free to modify anything else as you wish, playing around will help you learn.

Please don't push your navigation code back to the repository. It is intended as a starting codebase and only changes to the base setup should be pushed.
