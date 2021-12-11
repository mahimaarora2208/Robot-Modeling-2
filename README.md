# Project 2
> Mahima Arora, Tyler Barrett

### Installing Project 2 Files

Our project files are set up as a single ROS package named `ck_robot`. To install and run our project files, we can simply add the `ck_robot` package to your preferred ROS workspace. 

Copy the `catkin_ws/src/ck_robot` folder and place in the `src` of that workspace. In your workspace folder, run `catkin_make` and add the executables to your ROSPATH with `source devel/setup.bash`. At this point, you are able to test all of our software. 

### Spawning CyberKnife in Gazebo

In order to see CyberKnife in Gazebo, run the command below. This will show visualize the robot in Gazebo and create multiple topics used to control the robot. 

`roslaunch ck_robot gazebo.launch`

Gazebo should launch in a few seconds. After it is loaded, feel free to test / inspect the results as you wish. 

### Testing CyberKnife's Kinematics

To allow CyberKnife to target this tumor, source the workspace's `devel/setup.bash` file just like before and then run the following command after going to `catkin_ws/src/ck_robot` location:

`rosrun ck_robot control_cyber_knife.py`

With this script running, one should see the manipulator in its home position in Gazebo.The terminal might seem "stuck" at "13" but it is actually waiting for the tumor location as input. By running the next step in a new terminal, it should work. You can also follow the steps in the video in report.

### Creating a Tumor Object in Gazebo

For our demonstration scenario, CyberKnife will apply joint velocities to point its end effector at this simulated tumor and continuously track the tumor until the user closes the application. 

For visualization purposes, the user can create a tumor in Gazebo. Navigate to the `catkin_ws/src/ck_robot` folder and create a tumor by running the following commands in a terminal window. 

`source ../../devel/setup.bash;
rosrun ck_robot generate_tumor.py`

If Gazebo is open, a green ball will appear floating along the Y axis. This is the tumor that CyberKnife will target. 

With this script running, one should see repeated output of some of the robot's joint locations. In Gazebo, CyberKnife should slowly start to move its end effector towards the tumor. After it has reached the tumor, CyberKnife will continue to point towards its target until the user closes the windows. 

### Creating a Tumor Object in Gazebo
