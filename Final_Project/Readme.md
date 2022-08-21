# Pick and place Robot ![image](https://img.shields.io/badge/Phi%20Science-Final%20Project-brightgreen)

### Description 
  - Implementation Universal robot (UR5) with Robotiq Gripper pick and place in Ros Gazebo.

### Final Output
![image](https://github.com/Alie20/Phi_Path_Planning-moveit/blob/main/Final_Project/Images/final.gif)

#### How to use this repository
- This project was tested in Ubuntu 20.04 with ROS Noetic.
- Make sure you have installed Python3.
- Install ROS Noetic, Gazebo, universal robot, Moveit, RViz.

#### How to run the file 
If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/Alie20/Phi_Path_Planning-moveit/tree/main/Final_Project
$ cd ~/catkin_ws
$ cakin_make
```

Now from a terminal window: 
We need three terminals for the following 
  1. Gazebo ![image](https://img.shields.io/badge/Gazebo-simulation%20%20%20%20%20%20%20%20-blue)
  2. Rviz ![image](https://img.shields.io/badge/Moveit-Motion%20Planning%20%20%20%20%20%20%20%20-lightgrey)
  3. Pick and place Node ![image](https://img.shields.io/badge/Python%20Node-pick%20%26%20place-green)

```sh
1. Run Gazebo launch file for simulation
$ roslaunch urg5_moveit_config gazebo.launch

2. Run demo_planning_execution launch file Which contain Rviz for motion planning
$ roslaunch urg5_moveit_config demo_planning_execution.launch

After Rviz open go to object scene import object then go to urg5_moveit_config/world/object1.scene.
Then mark box1 as a part of world, Then publish this scene.

3. Run pick and place node 
$ rosrun urg5_moveit_config pickandplace.py
```



