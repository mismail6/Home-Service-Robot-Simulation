#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

// Global variables to track robot state
bool robot_at_pickup = false;
bool robot_at_dropoff = false;

// Callback function for robot status
void robotStatusCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("Received robot status: %d", msg->data);
  
  if (msg->data == 0) {
    // Robot is moving
    robot_at_pickup = false;
    robot_at_dropoff = false;
  }
  else if (msg->data == 1) {
    // Robot reached pickup zone
    robot_at_pickup = true;
    robot_at_dropoff = false;
  }
  else if (msg->data == 2) {
    // Robot reached drop-off zone
    robot_at_pickup = false;
    robot_at_dropoff = true;
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(10);
    
    // Publishers and subscribers
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber status_sub = n.subscribe("robot_status", 1000, robotStatusCallback);
    
    // Wait for RViz
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    
    // Create marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set marker properties
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f; // Green
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    
    // place marker at pickup zone
    marker.pose.position.x = 2.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    ROS_INFO("Placing marker at pickup zone");
    marker_pub.publish(marker);
    
    // State machine for marker behavior
    enum State { SHOWING_PICKUP, HIDDEN, SHOWING_DROPOFF };
    State current_state = SHOWING_PICKUP;
    
    while (ros::ok())
    {
        ros::spinOnce(); // Process callbacks
        
        switch (current_state)
        {
            case SHOWING_PICKUP:
                if (robot_at_pickup)
                {
                    ROS_INFO("Robot reached pickup zone");
                    marker.action = visualization_msgs::Marker::DELETE;
                    marker_pub.publish(marker);
                    current_state = HIDDEN;
                }
                break;
                
            case HIDDEN:
                if (robot_at_dropoff)
                {
                    ROS_INFO("Robot reached drop-off zone");
                    
                    // Place marker at drop-off zone
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = -2.7; 
                    marker.pose.position.y = -2.7;
                    marker.color.r = 1.0f; // Red
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f;
                    marker_pub.publish(marker);
                    
                    current_state = SHOWING_DROPOFF;
                }
                break;
                
            case SHOWING_DROPOFF:
                ROS_INFO_ONCE("Home service task completed successfully");
                break;
        }
        
        r.sleep();
    }
    
    return 0;
}
