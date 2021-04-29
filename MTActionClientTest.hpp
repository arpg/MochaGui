// #include <mt_actionlib/server/mt_simple_action_server.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <actionlib/client/simple_action_client.h>
#include <carplanner_msgs/MTTestAction.h>
#include "threadpool.hpp"
#include <boost/thread.hpp>

#define NUM_THREADS 10

class MTActionClientTest
{
public:
    typedef actionlib::SimpleActionClient<carplanner_msgs::MTTestAction> MTTestActionClient;

public:
    inline MTActionClientTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh) :
        m_private_nh(private_nh),
        m_nh(nh)
    {
        m_threadPool.size_controller().resize(NUM_THREADS);
        m_private_nh.param("call_rate", m_callRate, 1.0);
        m_callLoop = m_private_nh.createTimer(ros::Duration(1.0/m_callRate), &MTActionClientTest::callLoopFunc, this);
        ROS_INFO("[Client] Constructed.");
    }
    
    inline ~MTActionClientTest()
    {
        ROS_INFO("[Client] Destroyed.");
    }

    inline bool MTTestActionCall(uint threadId)
    {
        ROS_INFO("[Client] Calling TestAction.");

        MTTestActionClient client("test_server/"+std::to_string(threadId)+"/test_action",true);
        client.waitForServer();

        carplanner_msgs::MTTestActionGoal goal;
        carplanner_msgs::MTTestActionResultConstPtr result;

        goal.thread_id = threadId;

        client.sendGoal(goal
            // , boost::bind(&MochaProblem::ApplyVelocitiesDoneCb, this, _1, _2)
            // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
            // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
            );

        if (client.waitForResult(ros::Duration(1.0)))
        {
            result = client.getResult();
            return true;
        }
        else
        {
            // ROS_ERROR("[Problem] << (%d) MTTestAction did not finish before the %fs timeout.", threadId, timeout);
            return false;
        }
    }

    inline void callLoopFunc(const ros::TimerEvent& event)
    {
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            dampings[ii] = damping;
            Eigen::VectorXd delta = dDeltaP *damping;
            vCubicProblems[ii] = std::make_shared<MochaProblem>(*this);
            vCubicProblems[ii]->UpdateOptParams(vCubicProblems[ii]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vFunctors[ii] = std::make_shared<ApplyCommandsThreadFunctor>(vCubicProblems[ii],
                                                                         ii,
                                                                         pDampingStates[ii],
                                                                         pDampingErrors[ii],
                                                                         vCubicProblems[ii]->m_CurrentSolution.m_Sample,
                                                                         true);
            m_ThreadPool.schedule(*vFunctors[ii]);
            damping/= DAMPING_DIVISOR;
        }
        m_ThreadPool.wait();
    }

private:
    ros::NodeHandle m_private_nh;
    ros::NodeHandle m_nh;
    // MTTestActionClient m_client;
    double m_callRate;
    ros::Timer m_callLoop;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

