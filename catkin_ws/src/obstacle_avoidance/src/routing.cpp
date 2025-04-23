// receive all obstacle
// receive goal
// Calculate gradient
// publish velocity

#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
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


// Constants
const double GOAL_X = 3.0;
const double GOAL_Y = -2.0;
const double SIRA_RADIUS = 0.6225851;
const double CONST1 = 5;

class Gradient {
public:
    double x;
    double y;

    Gradient () {}
    Gradient (double in_x, double in_y) : x(in_x), y(in_y) {}

};

class Distance {
public:
    double x;
    double y;
    double radius;

    Distance () {}
    Distance (double in_x, double in_y, double rad) : x(in_x), y(in_y), radius(rad) {}

    double getDistance () {
        double distance = sqrt(pow(x, 2) + pow(y,2));
		distance = distance - radius - SIRA_RADIUS;
		return distance;
    }
};


class Routing {
private:
	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber obstacles_sub;
    //ros::Subscriber point_sub;

	// Publishers
    ros::Publisher vel_pub;

	// Transform Listener
	tf::TransformListener tf_listener;

    // Timer
    ros::Timer velocity_timer;

	// Vectors & Arrays
    std::vector<Distance> distances;

    geometry_msgs::Twist current_velocity;
    double prev_grad_x = 0;
    double prev_grad_y = 0;

    double getdX (double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double t = (-1.0*x1*dx + -1.0*y1*dy)/(pow(dy, 2) + pow(dx, 2));

        // Clamp t between 0 and 1 to stay within segment bounds
        double inter =  std::max(0.0, std::min(1.0, t));

        return x1 + inter*(x2 - x1);
    }

    double getdY (double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double t = (-1.0*x1*dx + -1.0*y1*dy)/(pow(dy, 2) + pow(dx, 2));

        // Clamp t between 0 and 1 to stay within segment bounds
        double inter =  std::max(0.0, std::min(1.0, t));

        return y1 + inter*(y2 - y1);
    }

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

    Gradient getRepulsiveGradient () {
        const double INFLUENCE_RADIUS = 2.0;
        const double DECAY_RATE = 3.0;
        const double REPULSION_STRENGTH = 5.0;

        double x_sum = 0;
        double y_sum = 0;

        for (int i = 0; i < distances.size(); i++) {
            double current_distance = distances[i].getDistance();
            double dx = distances[i].x;
            double dy = distances[i].y;
            double repulsive_x_grad = 0;
            double repulsive_y_grad = 0;

            if (current_distance < INFLUENCE_RADIUS) {
                // Exponential decay: stronger as distance decreases
                double repulsion = REPULSION_STRENGTH * exp(-DECAY_RATE * current_distance / INFLUENCE_RADIUS);
                
                // Normalize direction vector
                double norm = current_distance + 1e-6;  // Avoid division by zero
                repulsive_x_grad = -repulsion * dx / norm;
                repulsive_y_grad = -repulsion * dy / norm;
            }

            x_sum += repulsive_x_grad;
            y_sum += repulsive_y_grad;
        }

        Gradient rep_temp(x_sum, y_sum);
        return rep_temp;
    }

    Gradient calculateGradient () {
        const double DAMPING = 0.3;
                
        Gradient rep_temp = getRepulsiveGradient();

        double dist_goal = sqrt(pow(GOAL_X, 2) + pow(GOAL_Y, 2)) + 1e-6;

        double attr_x = CONST1 * GOAL_X / dist_goal;
        double attr_y = CONST1 * GOAL_Y / dist_goal;

        double grad_x = attr_x + rep_temp.x + DAMPING*prev_grad_x;
        double grad_y = attr_y + rep_temp.y + DAMPING*prev_grad_y;

        double magnitude = sqrt(pow(grad_x, 2) + pow(grad_y, 2)) + 1e-6;
        grad_x /= magnitude;
        grad_y /= magnitude;

        Gradient temp(grad_x, grad_y);

        prev_grad_x = grad_x;
        prev_grad_y = grad_y;

        return temp;
    
    }

public:
	Routing () {
		// Initialize Subscriber
		obstacles_sub = n.subscribe("/obstacles", 1000, &Routing::obstacleCallback, this);
        // point_sub = n.subscribe("/point_topic", 1000, &Routing::pointCallback, this);

		// Intialize Publishers
        vel_pub = n.advertise<geometry_msgs::Twist>("/sirar/ridgeback/cmd_vel", 1000);

        // Initialize Timer
        velocity_timer = n.createTimer(ros::Duration(0.2), &Routing::velocityTimerCallback, this);

        // Initialize Velocity
        current_velocity.linear.x = 0;
        current_velocity.linear.y = 0;
        current_velocity.angular.z = 0;
	}	

    // Callback for point receiving
    // void pointCallback (const type::ConstPtr& msg) {
    //     GOAL_X = msg.x;
    //     GOAL_Y = msg.y;
    // }

	// Callback for obstacle processing
	void obstacleCallback (const obstacle_detector::Obstacles::ConstPtr& msg) {
		// Clear previous data
        distances.clear();

		// Process Circles
		for (int i = 0; i < msg->circles.size(); i++) {
			// Get Circle Data
			double x_center = msg->circles[i].center.x;
			double y_center = msg->circles[i].center.y;
			double radius = msg->circles[i].radius;

			// Transform Data
			geometry_msgs::PointStamped circle_center = convertToPoint(x_center, y_center);
			x_center = circle_center.point.x;
			y_center = circle_center.point.y;

			// Assign Data
			Distance dist(x_center, y_center, radius);
			distances.push_back(dist);
		}
		
		// Process Segements
		for (int i = 0; i < msg->segments.size(); i++) {
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
            
            double dx = getdX(x1, y1, x2, y2);
            double dy = getdY(x1, y1, x2, y2);

			// Process Data
            Distance dist(dx, dy, 0);
			distances.push_back(dist);
		}

	}

    // Callback for timer
    void velocityTimerCallback(const ros::TimerEvent&) {
        Gradient current_gradient = calculateGradient();

        const double LINEAR_SCALE = 0.5;

        current_velocity.linear.x = current_gradient.x;
        current_velocity.linear.y = current_gradient.y;

        ROS_INFO("x_vel is: %f, y_vel is: %f", current_velocity.linear.x, current_velocity.linear.y);

        vel_pub.publish(current_velocity);
    }
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "routing");
    
    Routing node;
    ROS_INFO("check");
    ros::spin();
    return 0;
}