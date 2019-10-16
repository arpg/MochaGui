#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <carplanner_msgs/Command.h>

const float force_rng = 0.469000000*500;
const float max_lin_x = 1.5;

const float phi_rng = 0.480091000*500;
const float max_ang_z = 6.28/20;

geometry_msgs::Twist::Ptr cmd_vel;

ros::Publisher cmd_vel_pub;

void command_cb(const carplanner_msgs::Command::ConstPtr msg)
{
  // cmd_vel->linear.x = std::min( (msg->force/mass)*msg->dt+prev_lin_x , max_lin_x );
  cmd_vel->linear.x = (msg->force/force_rng)*max_lin_x;
  cmd_vel->linear.y = 0;
  cmd_vel->linear.z = 0;
  cmd_vel->angular.x = 0;
  cmd_vel->angular.y = 0;
  cmd_vel->angular.z = (msg->phi/phi_rng)*max_ang_z;
  cmd_vel_pub.publish(*cmd_vel);
  ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "command2cmdvel");

    ros::NodeHandle nh;
    ros::Subscriber command_sub = nh.subscribe<carplanner_msgs::Command>("/mochagui/command", 1, command_cb);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    printf("%sInitialized.\n","[command2cmdvel] ");

    ros::spin();
}
