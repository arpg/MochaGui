#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <glog/logging.h>
#include <random>

#include <actionlib/client/simple_action_client.h>
#include <carplanner_msgs/ApplyVelocitiesAction.h>

#include "MochaVehicle.h"

#include <tf/transform_listener.h>

namespace mochapc {

/* FakeApplyVelocitiesNodelet
 * Nodelet that sends test goals to every world in the MochaVehicle  
 * ApplyVelocities server to stress test multithreading capabilities. 
 * Launches separate requests to each server, and a total of 11 requests 
 * to world 0 on each iteration. 
 */
class FakeApplyVelocitiesNodelet : public nodelet::Nodelet
{
public:

    virtual inline void onInit()
    {
        NODELET_INFO("Initializing...\n");

        ros::NodeHandle& nh = this->getMTNodeHandle();
        ros::NodeHandle& private_nh = this->getMTPrivateNodeHandle();

        m_applyVelocitiesTopic = "vehicle";

        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/0/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/1/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/2/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/3/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/4/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/5/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/6/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/7/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/8/apply_velocities", true));
        m_actionClients.push_back(std::make_unique<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>("vehicle/9/apply_velocities", true));

       // m_actionClients[0].waitForServer();
        NODELET_INFO_NAMED("[FakeApplyVelocitiesNodelet]", "Initialized, ready to send goals to %s", m_applyVelocitiesTopic.c_str());

        m_loopTimer = nh.createTimer(ros::Duration(m_timeout_s), [ this ](const ros::TimerEvent &time) {
            // Send goals for all clients
            for (unsigned int i = 0; i < m_actionClients.size(); ++i) {
                carplanner_msgs::ApplyVelocitiesGoal goal = generateRandomGoal(i);
                goal.world_id = i;
                m_actionClients[i]->sendGoal(goal);
                NODELET_INFO_NAMED("[FakeApplyVelocitiesNodelet]", "Sent goal for world %d", i);
            }
            ros::spinOnce();

            // Send goals for all world 0 clients
            /*
            for (const auto& client: m_actionClientsWorld0) {
                carplanner_msgs::ApplyVelocitiesGoal goal = generateRandomGoal(0);
                client->sendGoal(goal);
                NODELET_INFO_NAMED("[FakeApplyVelocitiesNodelet]", "Sent goal for world 0" );
            }
            */
            // Receive goals for all clients
            for (unsigned int i = 0; i < m_actionClients.size(); ++i) {
                m_actionClients[i]->waitForResult(ros::Duration(30.0));
                m_actionClients[i]->getResult();
                NODELET_INFO_NAMED("[FakeApplyVelocitiesNodelet]", "Received result for world %d", i);
            }

            // Receive goals for all world 0 clients
            /*
            for (const auto& client: m_actionClientsWorld0) {
                client->getResult();
                NODELET_INFO_NAMED("[FakeApplyVelocitiesNodelet]", "Received result for world 0");
            }
            */
        });
    }

private:

    // Uses a normal distribution to generate a random control 
    ControlCommand generateRandomControl() {
        double force = m_normal(m_twister);
        double curvature = m_normal(m_twister);
        Eigen::Vector3d torques(Eigen::Vector3d::Random());
        double dt = m_normal(m_twister);
        double dPhi = m_normal(m_twister);
        return ControlCommand(force, curvature, torques, dt, dPhi);
    }
    
    // Creates a goal with 200 random states and controls 
    carplanner_msgs::ApplyVelocitiesGoal generateRandomGoal(int worldId) {
        carplanner_msgs::ApplyVelocitiesGoal goal;
        Sophus::SE3d dTwv(Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 1>::Zero());
        VehicleState initial_state(dTwv);

        MotionSample sample;
        for (unsigned int j = 0; j < 200; ++j) {
            sample.m_vStates.push_back({ Sophus::SE3d(Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 1>::Random()) });
            sample.m_vCommands.push_back(generateRandomControl());
        }

        goal.initial_state = initial_state.toROS();
        goal.world_id = worldId;
        goal.initial_motion_sample = sample.toROS();

        return goal;
    }


    ros::Timer m_loopTimer;
    unsigned int m_worlds{10};

    std::normal_distribution<> m_normal{5, 2};
    std::random_device m_device{};
    std::mt19937 m_twister{m_device()};

    std::vector<std::unique_ptr<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>> m_actionClients;
    std::vector<std::unique_ptr<actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>>> m_actionClientsWorld0;

    std::string m_applyVelocitiesTopic{ "vehicle/apply_velocities" };
    float m_timeout_s{ 0.1f };
};

} // namespace mochapc

PLUGINLIB_EXPORT_CLASS(mochapc::FakeApplyVelocitiesNodelet, nodelet::Nodelet);
