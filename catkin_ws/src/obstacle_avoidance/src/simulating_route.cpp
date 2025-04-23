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
double GOAL_X = 3.0;
double GOAL_Y = 0.0;
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
        double counter = 0.0;
        double pos_x =  0.0;
        double pos_y =  0.0;
        double dx1 = 1.17247;
        double dy1 = -1.10995;
        double dx2 = 0.946626;
        double dy2 = 1.12136;
    //Distances
    std::vector<Distance> distances;
	ros::NodeHandle n;

    geometry_msgs::Twist current_velocity;
    double prev_grad_x = 0;
    double prev_grad_y = 0;

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

        double rep_mag = sqrt(pow(rep_temp.x, 2) + pow(rep_temp.y, 2)) + 1e-6;
        double attr_mag = sqrt(pow(attr_x, 2) + pow(attr_y, 2)) + 1e-6;

        std::cout << counter << "," << pos_x << "," << pos_y << "," << grad_x << "," << grad_y << "," << rep_temp.x/rep_mag << "," << rep_temp.y/rep_mag << "," << attr_x/attr_mag << "," << attr_y/attr_mag << "\n";
        return temp;
    
    }

public:
	Routing () {

        while (!(abs(GOAL_X) < 0.1 && abs(GOAL_Y) < 0.1)) {

            distances.clear();

            Distance d1(dx1, dy1, 0);
            Distance d2(dx2, dy2, 0);

            distances.push_back(d1);
            distances.push_back(d2);


            // Initialize Velocity
            current_velocity.linear.x = 0;
            current_velocity.linear.y = 0;
            current_velocity.angular.z = 0;

            Gradient current_gradient = calculateGradient();

            const double LINEAR_SCALE = 0.5;

            current_velocity.linear.x = current_gradient.x;
            current_velocity.linear.y = current_gradient.y;

            double add_x = current_velocity.linear.x*0.2;
            double add_y = current_velocity.linear.y*0.2;
            pos_x += add_x;
            pos_y += add_y;
            dx1 -= add_x;
            dy1 -= add_y;
            dx2 -= add_x;
            dy2 -= add_y;
            GOAL_X -= add_x;
            GOAL_Y -= add_y;
            counter += 0.2;
        }
    }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "routing");
    
    Routing node;
    ROS_INFO("check");
    return 0;
}