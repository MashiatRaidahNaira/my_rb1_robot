#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

using namespace std;
class RotateRobot {
private:
  // ROS Objects
  ros::NodeHandle nh;

  // ROS Services
  ros::ServiceServer robot_rotate_service;

  // ROS Subscribers
  ros::Subscriber sub;

  // ROS Publishers
  ros::Publisher pub;

  // ROS Messages
  geometry_msgs::Twist msg;

  // Helpful variables
  double roll, pitch, yaw;
  double target_rad;
  double kP = 0.5;
  int prev_deg;

public:
  RotateRobot() {
    robot_rotate_service =
        nh.advertiseService("/rotate_robot", &RotateRobot::my_callback, this);
    sub = nh.subscribe("/odom", 1000, &RotateRobot::subCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  }

  void rotate_robot(int degrees) {
    target_rad = degrees * (3.14159265 / 180);
    msg.angular.z = kP * (target_rad - yaw);
    pub.publish(msg);
    ROS_INFO("Target= %f  Current= %f", target_rad, yaw);
  }

  // we define the callback function of the subscriber
  void subCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
  }

  // We define the callback function of the service
  bool my_callback(my_rb1_ros::Rotate::Request &req,
                   my_rb1_ros::Rotate::Response &res) {
    // res.some_variable = req.some_variable + req.other_variable;
    ROS_INFO("The Service rotate_robot has been called");
    ROS_INFO("Request Data==> degrees=%d", req.degrees);

    target_rad = req.degrees * (3.14159265 / 180);

    while (ros::ok()) {
      if (req.degrees < 0 && yaw > target_rad) {
        rotate_robot(req.degrees);
        prev_deg = req.degrees;
      } else if (req.degrees > 0 && yaw < target_rad) {
        rotate_robot(req.degrees);
        prev_deg = req.degrees;
      } // two else if conditions for 0 cuz there are two possible previous
        // conditions one (-90 to 0) or (90 to  0)
      else if (req.degrees == 0 && prev_deg < 0) {
        if (yaw < target_rad) {
          rotate_robot(req.degrees);
        } else {
          break;
        }
      } else if (req.degrees == 0 && prev_deg > 0) {
        if (yaw > target_rad) {
          rotate_robot(req.degrees);
        } else {
          break;
        }
      } else {
        break;
      }
      ros::spinOnce();
    }

    res.result = "Successfully Completed";
    ROS_INFO("sending back response:Successfully Completed");

    ROS_INFO("Finished service rotate_robot");
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_server_node");

  RotateRobot rotate;

  ros::spin();

  return 0;
}