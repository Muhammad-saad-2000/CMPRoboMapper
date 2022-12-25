# CMPRoboMapper
<!-- add code block -->
## Needed packages
```bash
sudo apt-get install ros-noetic-navigation -y
sudo apt-get install ros-noetic-gmapping -y
sudo apt-get install ros-noetic-robot-localization -y
sudo apt-get install ros-noetic-mavros-msgs -y
sudo apt-get install ros-noetic-velocity-controllers -y
sudo apt-get install ros-noetic-twist-mux -y
sudo apt-get install ros-noetic-teleop-twist-keyboard -y

```
## Initialize the project 
```bash
cd CMPRoboMapper
#____________
catkin_make
#try this if it does not continue (pip install pyparsing==2.4.7) and then catkin_make
#____________

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

chmod +x "src/commander/scripts"
```
## Launch the project
```bash
roslaunch commander launch_project.launch
```
## Launch the keyboard controller node
```bash
rosrun commander keyboard_controller.py
```
