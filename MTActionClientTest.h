// #include <mt_actionlib/server/mt_simple_action_server.h>

#include "MTActionTest.h"
#include <actionlib/client/simple_action_client.h>

class MTActionClientTest;

struct MTTestActionThreadFunctor 
{
    MTTestActionThreadFunctor(std::shared_ptr<MTActionClientTest> clientTest, uint threadId, bool& success);

    void operator()();

    std::shared_ptr<MTActionClientTest> m_clientTest;
    uint m_threadId;
    bool* m_successPtr;

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MTActionClientTest
{
public:
    typedef actionlib::SimpleActionClient<carplanner_msgs::MTTestAction> MTTestActionClient;

public:
    MTActionClientTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh);
    ~MTActionClientTest();

    bool TestActionCall(uint threadId);

    void callLoopFunc(const ros::TimerEvent& event);

private:
    ros::NodeHandle m_private_nh;
    ros::NodeHandle m_nh;
    double m_callRate;
    ros::Timer m_callLoop;
    boost::threadpool::pool m_threadpool;
    // MTTestActionClient m_client;

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

