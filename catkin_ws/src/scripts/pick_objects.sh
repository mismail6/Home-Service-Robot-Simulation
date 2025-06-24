#!/bin/sh

cd /home/workspace/catkin_ws

export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/worlds/project2.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/map/map.yaml"

# Launch TurtleBot
echo "Launching TurtleBot"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 12

# Launch SLAM
echo "Starting SLAM"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 10

# Launch rviz
echo "Starting RViz"
xterm -e "cd $(pwd) && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 7

xterm -e " source devel/setup.bash && rosrun pick_objects pick_objects"
