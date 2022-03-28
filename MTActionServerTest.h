// #include <mt_actionlib/server/mt_simple_action_server.h>

#include "MTActionTest.h"
#include <actionlib/server/simple_action_server.h>

#ifndef USE_SIMPLE_ACTIONS
class TestActionFunctionObj : public ros::CallbackInterface
{
public:
    TestActionFunctionObj(const boost::function<void()> &callback) :
        m_Callback(callback)
    {}

    virtual ros::CallbackInterface::CallResult call() override {
        m_Callback();
        return ros::CallbackInterface::CallResult::Success;
    }

    virtual bool ready() override {
        return true;
    }

private:
    boost::function<void()> m_Callback;
};
#endif

class MTActionServerTest
{
public:
#ifdef USE_SIMPLE_ACTIONS
    typedef actionlib::SimpleActionServer<carplanner_msgs::MTTestAction> MTTestActionServer;
#else
    typedef actionlib::ActionServer<carplanner_msgs::MTTestAction> MTTestActionServer;
#endif

public:
    MTActionServerTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh);
    ~MTActionServerTest();

#ifdef USE_SIMPLE_ACTIONS
    void MTTestActionService(const carplanner_msgs::MTTestGoalConstPtr &goal);
#else
    void MTTestActionService(actionlib::ServerGoalHandle<carplanner_msgs::MTTestAction> goalHandle);
#endif

private:
    ros::NodeHandle m_private_nh;
    ros::NodeHandle m_nh;
#ifndef USE_MANY_SERVERS
    MTTestActionServer m_server;
#else
    MTTestActionServer m_server0;
    MTTestActionServer m_server1;
    MTTestActionServer m_server2;
    MTTestActionServer m_server3;
    MTTestActionServer m_server4;
    MTTestActionServer m_server5;
    MTTestActionServer m_server6;
    MTTestActionServer m_server7;
    MTTestActionServer m_server8;
    MTTestActionServer m_server9;
#endif

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

