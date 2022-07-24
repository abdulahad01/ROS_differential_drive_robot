# ROS_differential_drive_robot
 This is a ROS simulation of various common robot behaviours namely :
1. Obstacle avoidance
2. Wall following
3. Go to goal
4. [Bug 0](https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf)

Last tested on :
ROS Noetic
Ubuntu 20.04

for running the simulations copy file to your ros workspace and after running catkin_make, open the terminal and run :

```
$ roslaunch diff_description bug0.launch 
```

## The obstacle world
![Obstacle world](img/obstacle_world.jpg?raw=true "Obstacle World")
![Top view](img/top_view.jpg?raw=true "Top view")


