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
#try this if it does not continue and then (catkin_make)
pip install pyparsing==2.4.7
#____________

echo "source ./devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

chmod +x "src/commander/scripts/keyboard_controller.py"
chmod +x "src/commander/scripts/sensor_alignment.py"
chmod +x "src/commander/scripts/mapping.py"
```
## Launch Gazibo and Rviz
```bash
source ~/CMPRoboMapper/devel/setup.bash
# for baic simulation
roslaunch commander launch_project.launch
```
## Launch the keyboard controller node
```bash
source ~/CMPRoboMapper/devel/setup.bash
# for external keyboard library (prefered as it is more responsive and has more features)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot/robotnik_base_control/cmd_vel
# for our keyboard controll (req. 1)
rosrun commander keyboard_controller.py
```
## Launch mapping with known pose simultation
```bash
source ~/CMPRoboMapper/devel/setup.bash
# Gazibo and Rviz
roslaunch commander launch_project.launch
# mapping with known pose pyhton Script
rosrun commander mapping.py
```
### **Notes:** 
- To add the occupancy grid map topic (occupancy_grid) to rviz to visualize the mapping.
### **Screen Shots**:
![Mapping 1](https://i.imgur.com/aqovWb3_d.webp?maxwidth=760&fidelity=grand)
![Mapping 2](https://i.imgur.com/T8ms2Vi_d.webp?maxwidth=760&fidelity=grand)

## Launch SLAM simultation
```bash
source ~/CMPRoboMapper/devel/setup.bash
# Gazibo and Rviz
roslaunch commander launch_project.launch
# SLAM pyhton Script
rosrun commander SLAM.py
```
### **Notes:**
- To add the estimate map topic (estimate_map) to rviz to visualize the estimate map of the robot.

- To add the pos topic (estimate_pos) to rviz to visualize the estimate pos of the robot.

- To add the particles of the MCL topic (particles) to rviz to visualize the particles.
### **Screen Shots**:
![SLAM 1](https://i.imgur.com/Rcov3mJ_d.webp?maxwidth=760&fidelity=grand)
![SLAM 2](https://i.imgur.com/xEHY25Y_d.webp?maxwidth=760&fidelity=grand)
## **Happy mapping!**