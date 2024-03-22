# cleaner_bot
cleaner_bot is a ros-noetic turtlebot3 based package which simulates a vacuum cleaner robot.

## Installation
To install this repository first make sure you have all the turtlebot3 packages and explore-lite packages installed.
For that:
```bash
sudo apt-get install ros-noetic-move-base ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-navigation ros-noetic-explore-lite
```

## Usage
Clone this repository in `catkin_ws/src` and build your workspace:
```bash
catkin_make
```
## Running
Start the gazebo simulation with
```bash
roslaunch cleaner_bot gazebo.launch
```

If the map of your world is not known then you must run the exploration first which is:
```bash
roslaunch cleaner_bot explore.launch
```

Do not forget to save your map after exploration is done with:
```bash
rosrun map_server map_saver --occ 90 --free 10 -f ~/catkin_ws/src/cleaner_bot/map/<YOUR_MAP> map:=/map
```
