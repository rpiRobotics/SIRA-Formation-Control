#include <vector>
#include <cmath>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ros/time.h>
#include <obstacle_detector/Obstacles.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "obstacle_detector/utilities/segment.h"
#include "obstacle_detector/utilities/circle.h"

class Avoid {
private:
	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber obstacles_sub;

	// Publishers
	ros::Publisher velocity_interrupt;
	ros::Publisher angle_interrupt;

	// Transform Listener
	tf::TransformListener tf_listener;

	// Constants
	const double STOP_ZONE = 0.25;
	const double SIRA_RADIUS = 0.6225851;

	// Distance & Angles
	std::vector<double> distances;
	std::vector<double> angles;

	geometry_msgs::PointStamped convertToPoint (double x_val, double y_val) {
		geometry_msgs::PointStamped point_msg;
		point_msg.header.stamp = ros::Time::now();  // Set the timestamp to now (or original timestamp)
        point_msg.header.frame_id = "laser";    // Set the appropriate frame ID
            
        // Set x and y from the object's center
        point_msg.point.x = x_val;
        point_msg.point.y = y_val;
        point_msg.point.z = 0.0; 

		// Try transforming point
		geometry_msgs::PointStamped new_point_msg;
        try {
            tf_listener.transformPoint("base_link", point_msg, new_point_msg);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Transform failed: %s", ex.what());
            return point_msg;
        }

		return new_point_msg;
	}

	double getCircleDistance (double x_val, double y_val, double radius) {
		double distance = sqrt(pow(x_val, 2) + pow(y_val,2));
		distance = distance - radius - SIRA_RADIUS;
		return distance;
	}
	
	double getSegmentDistance (double x1_val, double y1_val, double x2_val, double y2_val) {
		double distance = abs(x2_val*y1_val - x1_val*y2_val)/sqrt(pow((y2_val - y1_val), 2) + pow((x2_val - x1_val), 2));
		return distance;
	}

	double getSegmentAngle (double x1, double y1, double x2, double y2) {
		double m = (y2 - y1)/(x2 - y1);
		double invm = -1/m;

		double x = (m*x1)/(invm - m);
		double y = m*x - x1 - y1;

		double angle = atan2(y, x);
		return angle;
	}

    int needStop() {
        if (distances.empty()) return -1;
        auto min_it = std::min_element(distances.begin(), distances.end());
        if (*min_it <= STOP_ZONE) {
            return std::distance(distances.begin(), min_it);
        }
        return -1;
    }
	


public:
	Avoid () {
		// Initialize Subscriber
		obstacles_sub = n.subscribe("/obstacles", 1000, &Avoid::obstacleCallback, this);

		// Intialize Publishers
		velocity_interrupt = n.advertise<std_msgs::Bool>("/interrupt", 1000);
		angle_interrupt = n.advertise<std_msgs::Float64>("/obstacle_angle", 1000);
	}	

	// Callback for obstacle processing
	void obstacleCallback (const obstacle_detector::Obstacles::ConstPtr& msg) {
		// Clear previous data
		distances.clear();
		angles.clear();

		// Check if data being received through subscriber
		if (msg->circles.empty() && msg->segments.empty()) {
			ROS_INFO ("No laser scan obstacles being read. :(");
		}
		
		// Determine length of segment & circle arrays
		int circles_length = sizeof(msg->circles) / sizeof(msg->circles[0]);
		int segments_length = sizeof(msg->segments) / sizeof(msg->segments[0]);

		// Process Circles
		for (int i = 0; i < circles_length; i++) {
			// Get Circle Data
			double x_center = msg->circles[i].center.x;
			double y_center = msg->circles[i].center.y;
			double radius = msg->circles[i].radius;

			// Transform Data
			geometry_msgs::PointStamped circle_center = convertToPoint(x_center, y_center);
			x_center = circle_center.point.x;
			y_center = circle_center.point.y;

			// Process Data
			double distance = getCircleDistance(x_center, y_center, radius);
			double angle = atan2(y_center, x_center);

			distances.push_back(distance);
			angles.push_back(angle);

			// Publish distances to see if outliers
			ROS_INFO ("Circle Distance: %f", distance);
		}
		
		// Process Segements
		for (int i = 0; i < segments_length; i++) {
			// Get Segment Data
			double x1 = msg->segments[i].first_point.x;
			double y1 = msg->segments[i].first_point.y;
			double x2 = msg->segments[i].last_point.x;
			double y2 = msg->segments[i].last_point.y;

			// Transform Data
			geometry_msgs::PointStamped point1 = convertToPoint(x1, y1);
			geometry_msgs::PointStamped point2 = convertToPoint(x2, y2);
			x1 = point1.point.x;
			y1 = point1.point.y;
			x2 = point2.point.x;;
			y2 = point2.point.y;

			// Process Data
			double distance = getSegmentDistance(x1, y1, x2, y2);
			double angle = getSegmentAngle(x1, y1, x2, y2);

			distances.push_back(distance);
			angles.push_back(angle);

			// Publish distances to see if outliers
			ROS_INFO ("Segment Distance: %f", distance);
		}

        std_msgs::Bool allow;
        allow.data = true;
        std_msgs::Float64 angle_msg;

        int j = needStop();
        if (j != -1) {
            allow.data = false;
            angle_msg.data = angles[j];
            angle_interrupt.publish(angle_msg);
        }
        velocity_interrupt.publish(allow);
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "obstacle_avoid");
    Avoid node;
    ros::spin();
    return 0;
}