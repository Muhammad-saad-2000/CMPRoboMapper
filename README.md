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
## Launch the project
```bash
source ~/CMPRoboMapper/devel/setup.bash
# for baic simulation
roslaunch commander launch_project.launch
# for mapping simulation
roslaunch commander launch_mapping.launch
```
## Launch the keyboard controller node
```bash
source ~/CMPRoboMapper/devel/setup.bash
# for external keyboard library (prefered as it is more responsive and has more features)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot/robotnik_base_control/cmd_vel
# for our keyboard controll (req. 1)
rosrun commander keyboard_controller.py
```

