#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_tf_publisher");
    ros::NodeHandle nh;

    // Create static transform broadcaster
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // Define static transform
    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = "base_link";  // Parent frame
    static_transform.child_frame_id = "laser";       // Child frame

    // Set translation (x, y, z)
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.48;
    static_transform.transform.translation.z = 0.0;

    // Set rotation (x, y, z, w)
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0; // No rotation

    // Broadcast the transform
    static_broadcaster.sendTransform(static_transform);

    ROS_INFO("Static transform published from 'base_link' to 'laser'.");

    ros::spin();  // Keep the node alive
    return 0;
}
