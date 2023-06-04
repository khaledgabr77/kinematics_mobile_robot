#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

class TurtleGTG
{
public:
  TurtleGTG(const std::string& x_str, const std::string& y_str, const std::string& theta_str)
    : nh_(""), pose_sub_(), cmd_vel_pub_(), goal_x_(std::stof(x_str)), goal_y_(std::stof(y_str)), goal_theta_(std::stof(theta_str))
  {
    pose_sub_ = nh_.subscribe("/turtle1/pose", 10, &TurtleGTG::poseCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TurtleGTG::goToGoal, this);
  }

  void poseCallback(const turtlesim::PoseConstPtr& pose_msg)
  {
    pose_ = *pose_msg;

  }

  void goToGoal(const ros::TimerEvent& event)
  {
    geometry_msgs::Twist new_vel;

    // Euclidean Distance
    double distance_to_goal = std::sqrt(std::pow(goal_x_ - pose_.x, 2) + std::pow(goal_y_ - pose_.y, 2));
    // Angle to Goal
    double angle_to_goal = std::atan2(goal_y_ - pose_.y, goal_x_ - pose_.x);
    ROS_INFO("DistanceToGoal: %f, AngleToGoal: %f,", distance_to_goal, angle_to_goal);

    cmd_vel_pub_.publish(new_vel);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Timer timer_;
  turtlesim::Pose pose_;
  float goal_x_;
  float goal_y_;
  float goal_theta_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_to_goal_node");

  if (argc < 4)
  {
    ROS_ERROR("Not enough command-line arguments provided. Usage: rosrun <package_name> <node_name> <x> <y> <theta>");
    return 1;
  }

  TurtleGTG turtle_gtg(argv[1], argv[2], argv[3]);
  ros::spin();
  return 0;
}

