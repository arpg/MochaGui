// #include <mt_actionlib/server/mt_simple_action_server.h>

#include "MTActionTest.h"
#include <actionlib/server/simple_action_server.h>

class MTActionServerTest
{
public:
    typedef actionlib::SimpleActionServer<carplanner_msgs::MTTestAction> MTTestActionServer;

public:
    MTActionServerTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh);
    ~MTActionServerTest();

    void MTTestActionService(const carplanner_msgs::MTTestGoalConstPtr &goal);

private:
    ros::NodeHandle m_private_nh;
    ros::NodeHandle m_nh;
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

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

