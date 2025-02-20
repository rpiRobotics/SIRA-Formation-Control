#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include <cmath>

class VelocityFilter {
private:
    ros::NodeHandle n;
    ros::Subscriber vel_sub;
    ros::Subscriber allow_sub;
    ros::Subscriber angle_sub;
    ros::Publisher vel_pub;
    
    bool is_allowed;
    //int action_allowed;
    geometry_msgs::Twist last_vel;
    double current_angle;  // Store the most recent angle

    bool validVelocity(double linear_x, double linear_y, double angular_z) {
        if (linear_x == 0 && linear_y == 0) {
            return true;
        }
        else {
            double angle_direction = atan2(linear_y, linear_x);
            double upper_range = current_angle + (M_PI/2);
            double lower_range = current_angle - (M_PI/2);

            if (angle_direction > upper_range && angle_direction < lower_range) {
                bool positive = (current_angle > 0);
                double upper_limit = 0;
                double lower_limit = 0;

                if (positive) {
                    lower_limit = -2.0*M_PI + current_angle;
                }
                else {
                    upper_limit = 2.0*M_PI + current_angle;
                }

                if (angular_z < upper_limit && angular_z > lower_limit) {
                    return true;
                }
                else {
                    return false;
                }
            }
            return true;
        }
    }

public:
    VelocityFilter() : is_allowed(false), current_angle(0.0) {
        vel_sub = n.subscribe("/vel_interrupt", 1000, &VelocityFilter::velCallback, this);
        allow_sub = n.subscribe("/interrupt", 1000, &VelocityFilter::allowCallback, this);
        angle_sub = n.subscribe("/obstacle_angle", 1000, &VelocityFilter::angleCallback, this);
        vel_pub = n.advertise<geometry_msgs::Twist>("/sirar/ridgeback/cmd_vel", 1000);
    }

    void angleCallback(const std_msgs::Float64::ConstPtr& msg) {
        current_angle = msg->data;
    }

    void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        last_vel = *msg;
        
        if (is_allowed && validVelocity(msg->linear.x, msg->linear.y, msg->angular.z)) {
            vel_pub.publish(last_vel);
        } else {
            geometry_msgs::Twist zero_vel;
            vel_pub.publish(zero_vel);
        }
    }

    // void velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //     last_vel = *msg;
    //     double lin_x = last_vel.linear.x;
    //     double lin_y = last_vel.linear.y;
        
    //     if ((action_allowed == -1) && validVelocity(msg->linear.x, msg->linear.y, msg->angular.z)) {
    //         vel_pub.publish(last_vel);
    //     }
    //     // Make velocity reverse, at a rate of 0.25 
    //     else if (action_allowed = 1) {
    //         geometry_msgs::Twist rebound_vel;
    //         rebound_vel.linear.x = lin_x * -0.1;
    //         rebound_vel.linear.y = lin_y * -0.1;
    //         vel_pub.publish(rebound_vel);
    //     } 
    //     // Stop if too close
    //     else if (action_allowed = 2) {
    //         geometry_msgs::Twist zero_vel;
    //         vel_pub.publish(zero_vel);
    //     } 
    //     // Slow down if heading in direction of obstacle & speed is 0.5 or above
    //     else if (action_allowed = 3) {
    //         if (abs(current_angle - (atan2(lin_y, lin_x))) > M_PI*0.5) {
    //             vel_pub.publish(last_vel);
    //         }
    //         else {
    //             geometry_msgs::Twist slow_vel;
    //             slow_vel = last_vel;
    //             if (lin_x > 0.35) {
    //                 slow_vel.linear.x = 0.35;
    //             }
    //             else if (lin_x < -0.35) {
    //                 slow_vel.linear.x = -0.35;
    //             }

    //             if (lin_y > 0.35) {
    //                 slow_vel.linear.y = 0.35;
    //             }
    //             else if (lin_y < -0.35) {
    //                 slow_vel.linear.y = -0.35;
    //             }
    //             vel_pub.publish(slow_vel);
    //         }
            
    //     }
    // }

    void allowCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_allowed = (msg->data == true);
        if (!is_allowed) {
            geometry_msgs::Twist zero_vel;
            vel_pub.publish(zero_vel);
        }
    }
    
    // void allowCallback(const std_msgs::Int32::ConstPtr& msg) {
    //     action_allowed = msg->data;
    //     if (action_allowed == 2) {
    //         geometry_msgs::Twist zero_vel;
    //         vel_pub.publish(zero_vel);
    //     }
    // }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_filter");
    VelocityFilter filter;
    ros::spin();
    return 0;
}