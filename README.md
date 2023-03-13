## Teach by Simulation
A robot teaching platform running on ROS with a front-end GUI built on Angular. The user opens the web application in order to teach the assembly through a 3D simulation environment. [Demo Video](https://drive.google.com/file/d/13pZFC7mTYEvzkSX0FFcxbXecyGgloEgz/view?usp=share_link)

#### How to use this repository
 - This project was tested in Ubuntu 18.04 with ROS kinetic.
 - Make sure you have installed Python2.7 and some useful libraries/packages, such as Numpy, cv2, etc.
 - Install ROS kinetic, Gazebo, universal robot, Moveit, RViz, Gzweb. 
 - Assuming your universal robot workspace is named as `ur_ws`, after cloning the repository copy the `ur5_ROS-Gazebo` folder to `ur_ws/src/`
 - Under `ur_ws/src`, there are two folders: one is the official `universal_robot`, and the other is `ur5_ROS-Gazebo`. Open file `ur5_joint_limited_robot.urdf.xacro` under `ur_ws/src/universal_robot/ur_description/urdf/`, and __make the following change to the joint limit:__
  ```
    shoulder_pan_lower_limit="${-2*pi}" shoulder_pan_upper_limit="${2*pi}"
  ```
 - In the same directory, make a copy of `common.gazebo.xacro` and `ur5.urdf.xacro` in case of any malfunction. 
These two default files do not include camera and vacuum gripper modules. 
So we would replace these two files with customized files. 
Under directory `ur_ws/src/ur5_ROS-Gazebo/src/ur_description/`, copy `common.gazebo.xacro` and `ur5.urdf.xacro` to `ur_ws/src/universal_robot/ur_description/urdf/`.
 - Build the code under directory `ur_ws/`,
  ```
  $ catkin_make
  $ source devel/setup.bash  
  ```
  - Unzip and copy the files under `ur5_ROS-Gazebo/models` to `~/.gazebo/models` directory
  - Run `npm install` in `gui` and `server` folders

#### Running the code

 - Run the code with ROS and Gazebo
  ```
  $ roslaunch ur5_notebook playground.launch 
  ```
 - Launch gzweb
 -  Run `npm start` in `gui` and `server` folders 
 - Open the GUI locally on a web browser at http://localhost:4200
