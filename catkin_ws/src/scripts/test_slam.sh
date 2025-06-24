#!/bin/sh

cd /home/workspace/catkin_ws

export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/worlds/project2.world"

# Launch TurtleBot
echo "Launching TurtleBot"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 12

# Launch SLAM
echo "Starting SLAM"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 10

# Launch rviz
echo "Starting RViz"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 7

# Launch keyboard teleop
echo "Starting keyboard teleop..."
echo "Use this terminal to control the robot and build the map!"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch"
