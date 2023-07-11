# ROS1_Gazebo_Simulation
ROS1 Gazebo Simulation using Moveit

This repository is solely for recording what I have worked on using ROS1.

In essence, the packages within this repository is made for simulating two 6DOF robots on Gazebo. It is not meant to be used with actual, physical robots.

Within arm_test package, there are five task allocation algorithms written in C++ (named grasp_decider_#.cpp). The allocated tasks, for simplicity's sake, are basic pick and place tasks in which an object spawned in Gazebo is pick-and-placed by either robots. 
  grasp_decider_1 is a task allocation based on the location of task (or spawned object).
  grasp_decider_2 always allocates robot #1 to be used. Robot #2 is only used when robot #1 is unavailable to be used.
  grasp_decider_3 allocates the task evenly between the two robots. (ie 1st task by robot #1, 2nd by robot #2, 3rd by robot #1, 4th by robot #2, etc)
  grasp_decider_4 allocates the tasks based on time. (ie robot #1 works for 4minutes, after which robot #2 works for 3minutes, etc)
  grasp_decider_5 is a culmination of the first three task allocation algorithms. It is dynamic in a sense that whenever certain task conditions repeat for a given number of times, the algorithm changes the active algorithm (either the first, second or third algorithm). For example, if objects are only spawned near robot #1, grasp_decider_2 is used.
