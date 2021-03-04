#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <carplanner_msgs/Command.h>
#include <actionlib/client/simple_action_client.h>
#include <carplanner_msgs/UpdateStateAction.h>
#include <algorithm>    
#include <tf/transform_listener.h>

const float max_lin_x = 0.25;
const float max_ang_z = 6.28/8;
const float rate = 100;
double ang_vel = 0;
double lin_vel = 0;

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr msg)
{    
    lin_vel = msg->linear.x; 
    // msg->linear.y;
    // msg->linear.z;
    // msg->angular.x;
    // msg->angular.y;
    ang_vel = msg->angular.z;

    // cmd_vel.linear.x = std::min((float)cmd_vel.linear.x, (float)max_lin_x);
    // cmd_vel.linear.y = std::min((float)cmd_vel.linear.y, (float)max_lin_x);
    // cmd_vel.linear.z = std::min((float)cmd_vel.linear.z, (float)max_lin_x);
    // cmd_vel.angular.x = std::min((float)cmd_vel.angular.x, (float)max_ang_z);
    // cmd_vel.angular.y = std::min((float)cmd_vel.angular.y, (float)max_ang_z);
    // cmd_vel.angular.z = std::min((float)cmd_vel.angular.z, (float)max_ang_z);

    // ROS_INFO("Commanded vel %f curv %f", lin_vel, ang_vel);

    ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "mocha_simulator");

    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, cmd_vel_cb);

    actionlib::SimpleActionClient<carplanner_msgs::UpdateStateAction> actionUpdateState_client("plan_car/update_state",true);
    actionUpdateState_client.waitForServer();

    tf::TransformListener tflistener;

    ROS_INFO("Initialized.");

    ros::Time last_time = ros::Time::now();
    float last_lin_vel;
    while(ros::ok())
    {
        ros::spinOnce();

        // ROS_INFO("UpdateState: %s", actionUpdateState_client.getState().toString().c_str());
        if (actionUpdateState_client.getState() == actionlib::SimpleClientGoalState::ACTIVE) 
        { 
            // ros::Rate(rate).sleep();
            continue; 
        }

        float dt = ros::Time::now().toSec() - last_time.toSec();
        last_time = ros::Time::now();

        // float dt = 1/rate;

        tf::StampedTransform Twv;
        try
        {
            // ROS_INFO("looking up transform");
            tflistener.waitForTransform("map", "base_link/0", ros::Time::now(), ros::Duration(0.5));
            tflistener.lookupTransform("map", "base_link/0", ros::Time(0), Twv);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            continue;
        }

        carplanner_msgs::UpdateStateGoal goal;
        carplanner_msgs::UpdateStateResultConstPtr result;


        if (dt > 0.1 || dt < 0.0001) { continue; }
        ROS_INFO("Commanded vel %f curv %f", lin_vel, ang_vel);
        
        goal.worldId = 0;
        goal.command.force = (lin_vel - last_lin_vel)/dt;
        goal.command.curvature = ang_vel;
        // goal.command.torques.push_back(0);
        // goal.command.torques.push_back(0);
        // goal.command.torques.push_back(0);
        goal.command.dt = dt;
        goal.command.dphi = -ang_vel;
        // goal.command.time
        goal.forceDt = dt;
        goal.noDelay = true;
        goal.noUpdate = false;
        actionUpdateState_client.sendGoal(goal);

        float timeout(0.5);
        bool finished_before_timeout = actionUpdateState_client.waitForResult(ros::Duration(timeout));
        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = actionUpdateState_client.getState();
            // ROS_INFO("UpdateState finished: %s", state.toString());

            result = actionUpdateState_client.getResult();
            // last_lin_vel = pow(pow(result->state.lin_vel.x,2) + pow(result->state.lin_vel.y,2) + pow(result->state.lin_vel.z,2),0.5);

            tf::Vector3 last_lin_vel_ws(result->state.lin_vel.x,result->state.lin_vel.y,result->state.lin_vel.z);
            tf::Vector3 last_lin_vel_cs = last_lin_vel_ws * Twv.getBasis();
            last_lin_vel = last_lin_vel_cs[0];
        }
        else
        {
            // ROS_INFO("UpdateState did not finish before the %f s time out.",timeout);
        }

        // ros::Rate(rate).sleep();
    }
}
