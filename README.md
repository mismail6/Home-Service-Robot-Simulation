For mapping, the gmapping package was used to perform SLAM, allowing the robot to build a map of its environment while being manually controlled via the turtlebot_teleop package. 

For localization, the amcl package enabled the robot to determine its position within the pre-built map.

For navigation, the move_base package provided the core autonomous navigation capabilities, implementing path planning and obstacle avoidance to guide the robot between pickup and drop-off zones. 

For visualization, the visualization_msgs package enables creation of visual markers in RViz for representing virtual objects and providing task status feedback to operators.
Additionally, the turtlebot_gazebo package provides physics simulation, robot dynamics, and sensor modeling for testing navigation algorithms in a controlled virtual environment.

Custom Packages:

**pick_objects**: A custom-developed package that implements autonomous navigation task management. The pick_objects node uses the SimpleActionClient interface to communicate with the move_base action server, sending sequential navigation goals for pickup and drop-off locations. It publishes robot status messages to coordinate with other nodes and includes built-in timing for simulating object pickup operations. The package demonstrates integration of high-level task planning with low-level navigation capabilities.
**add_markers**: A custom-developed package for visual feedback management containing two specialized nodes. The add_markers_time node provides time-based marker visualization for demonstration purposes, automatically cycling through pickup and delivery phases based on predefined timing. The add_markers node provides robot-status-based marker management, subscribing to robot position updates and publishing/deleting markers based on actual robot location and task completion status. Both nodes use the visualization_msgs framework to create dynamic visual representations of virtual objects in RViz.

The complete system integrated custom nodes (pick_objects and add_markers) that communicated via ROS topics to coordinate autonomous navigation with visual feedback, successfully simulating a complete home service robot capable of navigating to designated locations and handling virtual objects.
