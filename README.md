# Ur5_gripper_gzweb

Follow these steps to install all the necessary libraries to run the ur5 + robotiq gripper in gzweb in Ubuntu 20 (ROS Noetic):

Clone this repo in your home
```
git clone https://github.com/gianmarco96/Ur5_gripper_gzweb
```
locate the catkin workspace
```
cd ~/Ur5_gripper_gzweb/gzweb_ws
```
Clean the catkin build space
```
catkin clean -b
```
Install any necessary package with rosdep
```
rosdep install --from-paths src --ignore src -r -y
```
Build the workspace, this can take up to 5 minutes the first time
```
catkin build
```
Now, there are a few changes to some scripts that are necessary. This is because normally ROS uses a [find package](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem) function to navitage through the file system and locate packages and robots meshes and urdfs. However, the function assumes that all the files to be located are within the ROS workspace, while gzweb requires the meshes to be in the assets folder. If we were dealing with a normal markup language (like html), we could use a relative path to solve the issue. However, the urdf file of the robot is parsed as a ROS parameter (in the form of a string) therefore all sense of relative paths are lost. 

A way around this is to use a xacro file which specifies the path of the robot's meshes to create the urdf. These are the steps required to spawn the robot in the gzweb environment. I suggest to start from a [clean gzweb environment](http://gazebosim.org/gzweb.html#install-collapse-1) for simplicity and then reptoduce the same steps for the mirocode environment.

First of all, copy the content of this repo's assets folder into your gzweb environment's asset folder (it should be located in ~/gzweb/http/client/assets). Then open the robot.xacro file and change the path argument to the gzweb's asset folder. **IMPORTANT:** this has to be an absolute path: something like _/home/$you_user_name/gzweb/http/client/assets/_. 

Now you can generate the urdf file by running the following command:
```
xacro robot.xacro > robot.urdf
```

Good, we are almost there. There's one last change that it's necessary. Find the ur5_cubes.launch file in ~/Ur5_gripper_gzweb/gzweb_ws/src/ur5_gazebo/launch and 
change the defualt path (_line 9_) to the assets path (in the gzweb folder).

All done! Now source the workspace. **IMPORTANT:** This should be done every time you open a new terminal. 
```
source ~/gzweb_ws/devel/setup.bash
```
If you prefer you can add the above line to the bashrc file to avoid sourcing the setup.bash every time `echo "source ~/gzweb_ws/devel/setup.bash" >> ~/.bashrc`

Run the demo. Open one terminal and launch 
```
roslaunch roslaunch ur5_gazebo ur5_cubes.launch gui:=false
```
Then in another terminal
```
cd gzweb
npm start
```
Go to 0.0.0.0:8080 to visualise the env on your browswer and press play. In a third terminal
```
roslaunch mirocloud_manipulator_config ur5_moveit_planning_execution.launch sim:=true 
```
And finally call the python scirpts that controls the robotic arm and gripper:
```
rosrun ur5_gazebo test.py --value 0.50
rosrun ur5_gazebo send_gripper.py --value 0.38
```
## Run in mirocloud


