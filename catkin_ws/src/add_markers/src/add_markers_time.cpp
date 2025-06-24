#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers_time");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

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

    visualization_msgs::Marker marker;
    
    // Set the frame ID and timestamp
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;

    // Set the marker action
    marker.action = visualization_msgs::Marker::ADD;

    // PICKUP ZONE - Set the pose of the marker
    marker.pose.position.x = 2.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color - Green for pickup
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Step 1: Publish marker at pickup zone
    ROS_INFO("Publishing GREEN marker at pickup zone (%.1f, %.1f)", marker.pose.position.x, marker.pose.position.y);
    marker_pub.publish(marker);
    
    // Step 2: Wait 5 seconds
    ROS_INFO("Waiting 5 seconds");
    ros::Duration(5.0).sleep();

    // Step 3: Hide the marker
    ROS_INFO("Hiding the marker");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    
    // Step 4: Wait another 5 seconds
    ROS_INFO("Waiting another 5 seconds");
    ros::Duration(5.0).sleep();

    // Step 5: Publish marker at drop-off zone
    ROS_INFO("Publishing RED marker at drop-off zone");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -2.7;  // Drop-off coordinates
    marker.pose.position.y = -2.7;
    marker.color.r = 1.0f;  // Change to red
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);

    ROS_INFO("Time-based marker sequence complete! Red marker will remain visible.");
    
    ros::spin();
    
    return 0;
}
