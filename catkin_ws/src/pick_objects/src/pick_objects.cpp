#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  // Publisher to communicate robot status to add_markers
  ros::Publisher status_pub = n.advertise<std_msgs::Int32>("robot_status", 1000);

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Publish moving to pickup
  std_msgs::Int32 status_msg;
  status_msg.data = 0;
  status_pub.publish(status_msg);
  ROS_INFO("Robot moving to pickup zone...");

  // PICKUP ZONE - Define position and orientation for the robot to reach
  ROS_INFO("Sending robot to pickup zone...");
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 2.0;   
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);
  
  // Wait for the results
  ac.waitForResult();
  
  // Check if the robot reached the pickup zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached the pickup zone!");
    
    // Publish status: robot at pickup zone
    status_msg.data = 1; // 1 = at pickup zone
    status_pub.publish(status_msg);
    
    // Pause 5 seconds to simulate picking up the object
    ROS_INFO("Simulating pickup... waiting 5 seconds");
    ros::Duration(5.0).sleep();
    ROS_INFO("Object picked up!");
    
    // DROP-OFF ZONE - Send robot to drop-off location
    ROS_INFO("Sending robot to drop-off zone...");
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = -2.7;
    goal.target_pose.pose.position.y = -2.7;  
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    
    // Send the drop-off goal
    ac.sendGoal(goal);
    ac.waitForResult();
    
    // Check if robot reached drop-off zone
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Robot reached the drop-off zone!");
      
      // Publish status: robot at drop-off zone
      status_msg.data = 2;
      status_pub.publish(status_msg);
      
      ROS_INFO("Delivered!");
    } else {
      ROS_INFO("The robot failed to reach the drop-off zone");
    }
    
  } else {
    ROS_INFO("The robot failed to reach the pickup zone");
  }

  return 0;
}
