#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <unistd.h>

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
  double speed_d, angle_d;
  double speed_r, angle_r;
  double t0, t1;

public:
  RotateRobot() {
    robot_rotate_service =
        nh.advertiseService("/rotate_robot", &RotateRobot::my_callback, this);
    ROS_INFO("The service /rotate_robot is READY");
    sub = nh.subscribe("/odom", 1000, &RotateRobot::subCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  }

  void stop_robot() {
    ROS_INFO("shutdown time! Stop the robot");
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    pub.publish(msg);
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
    ROS_INFO("The Service /rotate_robot has been REQUESTED");
    ROS_INFO("Request Data==> degrees= %d", req.degrees);

    try {
      // Initilize velocities
      msg.linear.x = 0;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      // Convert speed and angle to radians
      speed_d = 60.0;
      angle_d = req.degrees;
      speed_r = speed_d * (3.14159265 / 180);
      angle_r = angle_d * (3.14159265 / 180);

      // Check the direction of the rotation
      if (req.degrees < 0) {
        msg.angular.z = -abs(speed_r);
      } else {
        msg.angular.z = abs(speed_r);
      }

      // t0 is the current time
      t0 = ros::Time::now().toSec();

      ros::Rate loop_rate(10);

      double current_angle = 0.0;

      // loop to publish the velocity estimate, current_distance = velocity *
      // (t1 - t0)
      while (current_angle < abs(angle_r)) {
        // Publish the velocity
        pub.publish(msg);
        // t1 is the current time
        t1 = ros::Time::now().toSec();
        // Calculate current angle
        current_angle = speed_r * (t1 - t0);
        ROS_DEBUG("Current angle: %f", current_angle);
        loop_rate.sleep();
      }
      // set velocity to zero to stop the robot
      stop_robot();
      res.result = "Service /rotate_robot: SUCCESS";
      ROS_INFO(
          "sending back response: Service /rotate_robot: SUCCESS: %d degrees",
          req.degrees);
    } catch (...) {
      res.result = "Service /rotate_robot: FAILED";
      ROS_INFO(
          "sending back response:Service /rotate_robot: FAILED: %d degrees",
          req.degrees);
    }

    ROS_INFO("/rotate_robot Service COMPLETED");
    return true;
  }
};

int main(int argc, char **argv) {

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "rotate_server_node");

  RotateRobot rotate;

  ros::spin();

  return 0;
}