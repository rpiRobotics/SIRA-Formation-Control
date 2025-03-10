

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
const double CONST1 = 0.1;
const double CONST2 = 0.1;
const int GRID_SIZE = 80;

class Gradient {
public:
    double x;
    double y;

    Gradient () {}
    Gradient (double in_x, double in_y) : x(in_x), y(in_y) {}

};

class My_Circle {
public:
    double x;
    double y;
    double rad;

    My_Circle () {}
    My_Circle (double in_x, double in_y, double in_rad) : x(in_x), y(in_y), rad(in_rad) {}

    double getValue (double x_in, double y_in) {
        double x_diff = x_in - x;
        double y_diff = y_in - y;
        double temp_distance = sqrt(pow(x_diff, 2) + pow(y_diff,2));
        return pow((temp_distance - SIRA_RADIUS - rad), 2);
    }

    double getdx (double x_in) {
        return x_in - x;
    }

    double getdy (double y_in) {
        return y_in - y;
    }
};

class My_Segment {
public:
    double x1;
    double y1;
    double x2;
    double y2;

    My_Segment () {}
    My_Segment (double in_x1, double in_y1, double in_x2, double in_y2) : x1(in_x1), y1(in_y1), x2(in_x2), y2(in_y2) {}

    double getIntermediate (double x0, double y0) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double t = ((x0 - x1)*dx + (y0 - y1)*dy)/(pow(dy, 2) + pow(dx, 2));
        return t;
    }

    double getdx (double x_in, double y_in) {
        return x1 + getIntermediate(x_in, y_in)*(x2 - x1) - x_in;
    }

    double getdy (double x_in, double y_in) {
        return y1 + getIntermediate(x_in, y_in)*(y2 - y1) - y_in;
    }

    double getValue (double x_in, double y_in) {
        double temp_distance = sqrt(pow(getdx(x_in, y_in), 2) + pow(getdy(x_in, y_in), 2));
        return pow((temp_distance - SIRA_RADIUS), 2);
    }
};

class Avoid {
private:
	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber obstacles_sub;

	// Publishers
	/*Nothing yet, will decide when determining how to publish velocities*/

	// Transform Listener
	tf::TransformListener tf_listener;

	// Vectors & Arrays
    std::vector<My_Circle> circles_detected;
    std::vector<My_Segment> segments_detected;

    double potential[GRID_SIZE][GRID_SIZE];
    Gradient grad[GRID_SIZE][GRID_SIZE];


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


    double getAttractivePotential (double x_point, double y_point) {              
        double x_diff = x_point - GOAL_X;
        double y_diff = y_point - GOAL_Y;
        double temp = pow(x_diff, 2) + pow(y_diff,2);
        return 0.5*CONST1*temp;
    }

    double getRepulsivePotential (double x_point, double y_point) {
        double potential_sum = 0.0;
        for (int i = 0; i < circles_detected.size(); i++) {
            My_Circle temp = circles_detected[i];
            double temp_repulsive_pot = CONST2/temp.getValue(x_point, y_point);
            potential_sum += temp_repulsive_pot;
        }

        for (int i = 0; i < segments_detected.size(); i++) {
            My_Segment temp = segments_detected[i];
            double temp_repulsive_pot = CONST2/temp.getValue(x_point, y_point);
            potential_sum += temp_repulsive_pot;
        }
        return potential_sum;
    }

    Gradient getRepulsiveGradient (double x_point, double y_point) {
        double x_sum = 0;
        double y_sum = 0;

        for (int i = 0; i < circles_detected.size(); i++) {
            My_Circle temp = circles_detected[i];
            double dx = temp.getdx(x_point);
            double dy = temp.getdy(y_point);
            double val = pow((pow(dx, 2) + pow(dy, 2)), 2);

            double repulsive_x_grad = (-2.0*CONST2*dx)/val;
            double repulsive_y_grad = (-2.0*CONST2*dy)/val;

            x_sum += repulsive_x_grad;
            y_sum += repulsive_y_grad;
        }

        for (int i = 0; i < segments_detected.size(); i++) {
            My_Segment temp = segments_detected[i];
            double dx = temp.getdx(x_point, y_point);
            double dy = temp.getdy(x_point, y_point);
            double val = pow((pow(dx, 2) + pow(dy, 2)), 2);

            double repulsive_x_grad = (-2.0*CONST2*dx)/val;
            double repulsive_y_grad = (-2.0*CONST2*dy)/val;

            x_sum += repulsive_x_grad;
            y_sum += repulsive_y_grad;
        }
        
        Gradient rep_temp(x_sum, y_sum);
        return rep_temp;
    }

    void getPotentialAndGradient () {
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                double x_point = (i - (GRID_SIZE*0.5))*0.1;
                double y_point = (j - (GRID_SIZE*0.5))*0.1;

                double current_attractive_pot = getAttractivePotential(x_point, y_point);
                double current_repulsive_pot = getRepulsivePotential(x_point, y_point);

                potential[i][j] = current_attractive_pot + current_repulsive_pot;

                Gradient rep_temp = getRepulsiveGradient(x_point, y_point);

                double grad_x = (GOAL_X - x_point)*CONST1 + rep_temp.x;
                double grad_y = (GOAL_Y - y_point)*CONST2 + rep_temp.y;

                Gradient temp(grad_x, grad_y);
                grad[i][j] = temp;
            }
        }
    }

    void exportToCSV(const std::string& filename) {
        std::ofstream csvFile;
        csvFile.open(filename);
        
        // Add header
        csvFile << "x,y,potential,gradient_x,gradient_y" << std::endl;
        
        // Write data
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                double x_point = (i - (GRID_SIZE*0.5))*0.1;
                double y_point = (j - (GRID_SIZE*0.5))*0.1;
                
                csvFile << std::fixed << std::setprecision(4)
                        << x_point << ","
                        << y_point << ","
                        << potential[i][j] << ","
                        << grad[i][j].x << ","
                        << grad[i][j].y << std::endl;
            }
        }
        
        csvFile.close();
        ROS_INFO("Exported potential field data to %s", filename.c_str());
    }

public:
	Avoid () {
		// Initialize Subscriber
		obstacles_sub = n.subscribe("/obstacles", 1000, &Avoid::obstacleCallback, this);

		// Intialize Publishers
	}	

	// Callback for obstacle processing
	void obstacleCallback (const obstacle_detector::Obstacles::ConstPtr& msg) {
		// Clear previous data

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
			My_Circle current_circle(x_center, y_center, radius);
			circles_detected.push_back(current_circle);
            //ROS_INFO ("Circle values (x, y, rad) are: %f, %f, %f", x_center, y_center, radius);
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

			// Process Data
            My_Segment current_seg(x1, y1, x2, y2);
			segments_detected.push_back(current_seg);
            //ROS_INFO("Segment values (x1, y1, x2, y2) are: %f, %f, %f, %f", x1, y1, x2, y2);
		}

        getPotentialAndGradient();

        exportToCSV("potential_field.csv");
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "obstacle_avoid");
    
    Avoid node;
    ROS_INFO("check");
    ros::spin();
    return 0;
}