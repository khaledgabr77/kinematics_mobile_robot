#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    // Check if the required number of command line arguments are provided
    if (argc < 3)
    {
        ROS_ERROR("Insufficient command line arguments. Usage: move_turtlesim <linear_velocity> <angular_velocity>");
        return 1;
    }

    // Parse the command line arguments
    float linear_velocity = std::stof(argv[1]);
    float radius = std::stof(argv[2]);

    // Initialize the ROS node
    ros::init(argc, argv, "move_turtlesim");
    ros::NodeHandle nh;

    // Create a publisher to publish Twist messages on the "/turtle1/cmd_vel" topic
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Create a Twist message to store the linear and angular velocities
    geometry_msgs::Twist twist;
    // w = v / r
    // Set the linear and angular velocities of the Twist message

    twist.linear.x = linear_velocity;
    twist.angular.z = linear_velocity / radius;
            


    // Publish the Twist message repeatedly
    ros::Rate rate(10);  // Publish at 10Hz
    while (ros::ok())
    {
        cmdVelPub.publish(twist);
        rate.sleep();
    }

    return 0;
}
