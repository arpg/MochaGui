// #include <mt_actionlib/server/mt_simple_action_server.h>

#include "MTActionServerTest.h"

MTActionServerTest::MTActionServerTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh) :
    m_private_nh(private_nh),
    m_nh(nh),
    m_server0(m_nh, "test_server/0/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*0*/, this, _1), false),
    m_server1(m_nh, "test_server/1/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*1*/, this, _1), false),
    m_server2(m_nh, "test_server/2/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*2*/, this, _1), false),
    m_server3(m_nh, "test_server/3/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*3*/, this, _1), false),
    m_server4(m_nh, "test_server/4/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*4*/, this, _1), false),
    m_server5(m_nh, "test_server/5/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*5*/, this, _1), false),
    m_server6(m_nh, "test_server/6/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*6*/, this, _1), false),
    m_server7(m_nh, "test_server/7/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*7*/, this, _1), false),
    m_server8(m_nh, "test_server/8/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*8*/, this, _1), false),
    m_server9(m_nh, "test_server/9/test_action", boost::bind(&MTActionServerTest::MTTestActionService/*9*/, this, _1), false)
{
    m_server0.start();
    m_server1.start();
    m_server2.start();
    m_server3.start();
    m_server4.start();
    m_server5.start();
    m_server6.start();
    m_server7.start();
    m_server8.start();
    m_server9.start();
    ROS_INFO("[Server] Constructed.");
}
    
MTActionServerTest::~MTActionServerTest()
{
    ROS_INFO("[Server] Destroyed.");
}

void MTActionServerTest::MTTestActionService(const carplanner_msgs::MTTestGoalConstPtr &goal)
{
    ROS_INFO("[Server] Got TestAction call for %d.", goal->thread_id);

    double wait = MAX_SERVER_TIME*((double) rand() / (RAND_MAX));
    ROS_INFO("[Server] TestAction %d waiting for %f sec...", goal->thread_id, wait);
    double start = Tic();
    while (Toc(start)<wait);

    // carplanner_msgs::MTTestActionFeedback feedback;
    carplanner_msgs::MTTestResult result;

    result.thread_id = goal->thread_id;

    switch (goal->thread_id)
    {
        case 0:
            m_server0.setSucceeded(result);
            break;
        case 1:
            m_server1.setSucceeded(result);
            break;
        case 2:
            m_server2.setSucceeded(result);
            break;
        case 3:
            m_server3.setSucceeded(result);
            break;
        case 4:
            m_server4.setSucceeded(result);
            break;
        case 5:
            m_server5.setSucceeded(result);
            break;
        case 6:
            m_server6.setSucceeded(result);
            break;
        case 7:
            m_server7.setSucceeded(result);
            break;
        case 8:
            m_server8.setSucceeded(result);
            break;
        case 9:
            m_server9.setSucceeded(result);
            break;
        default:
            break;
    }    
}

