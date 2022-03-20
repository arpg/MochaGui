// #include <mt_actionlib/server/mt_simple_action_server.h>

#include "MTActionClientTest.h"

MTTestActionThreadFunctor::MTTestActionThreadFunctor(std::shared_ptr<MTActionClientTest> clientTest, uint threadId, unsigned char& success) :
    m_clientTest(clientTest),
    m_threadId(threadId),
    m_successPtr(&success)
{
    // ROS_INFO("Constructing MTTestActionThreadFunctor #%d", m_threadId);
}

MTTestActionThreadFunctor::~MTTestActionThreadFunctor()
{
    // ROS_INFO("[Functor %d] Destroyed.", m_threadId);
}

void MTTestActionThreadFunctor::operator()()
{
    // ROS_INFO("Running MTTestActionThreadFunctor for %s idx %d", m_Problem->m_problemName.c_str(), m_index);
    SetThreadName((boost::format("MTTestActionThreadFunctor #%d") % m_threadId).str().c_str());

    double wait = MAX_SERVER_TIME*((double) rand() / (RAND_MAX));

    // ROS_INFO("[Functor %d] Calling TestAction. Will wait %f sec.", m_threadId, wait);

    double start = Tic();
    (*m_successPtr) = (unsigned char)(m_clientTest->TestActionCall(m_threadId, wait));
    double dur = Toc(start);

    ROS_INFO("[Functor %d] Called TestAction: %s. Supposed to take %f sec. Took %f sec.", m_threadId, (*m_successPtr?"success":"fail"), wait, dur);
    
    return;
}

MTActionClientTest::MTActionClientTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh) :
    m_private_nh(private_nh),
    m_nh(nh)
{
    m_threadpool.size_controller().resize(NUM_THREADS);
    m_private_nh.param("call_rate", m_callRate, 0.2);
    m_callLoop = m_private_nh.createTimer(ros::Duration(1.0/m_callRate), &MTActionClientTest::callLoopFunc, this);
    // ROS_INFO("[Client] Constructed.");
}
    
MTActionClientTest::~MTActionClientTest()
{
    // ROS_INFO("[Client] Destroyed.");
}

bool MTActionClientTest::TestActionCall(uint threadId, double delay)
{
    ROS_INFO("[Client] Calling TestAction #%d.", threadId);

    MTTestActionClient client("test_server/"+std::to_string(threadId)+"/test_action",true); // seeing how it works to create new client each time
    client.waitForServer();

    carplanner_msgs::MTTestGoal goal;
    carplanner_msgs::MTTestResultConstPtr result;

    goal.thread_id = threadId;
    goal.delay = delay;

    client.sendGoal(goal
        // , boost::bind(&MochaProblem::ApplyVelocitiesDoneCb, this, _1, _2)
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
        );

    double timeout = MAX_SERVER_TIME*2;
    if (client.waitForResult(ros::Duration(timeout)))
    {
        result = client.getResult();
        uint result_id = result->thread_id;
        ROS_INFO("[Client] TestAction #%d finished.", threadId);
        return true;
    }
    else
    {
        ROS_INFO("[Client] TestAction #%d failed to finish after %d timeout.", threadId, timeout);
        return false;
    }
}

void MTActionClientTest::callLoopFunc(const ros::TimerEvent& event)
{
    std::vector<std::shared_ptr<MTTestActionThreadFunctor>> vFunctors;
    vFunctors.resize(NUM_THREADS);
    std::vector<std::shared_ptr<MTActionClientTest>> vClientTests;
    vClientTests.resize(NUM_THREADS);
    std::vector<unsigned char> vSuccesses;
    vSuccesses.resize(NUM_THREADS);

    ROS_INFO("[Client] Starting threads.");
    double start = Tic();

    for(int ii = 0 ; ii < NUM_THREADS ; ii++) 
    {
        vClientTests[ii] = std::make_shared<MTActionClientTest>(*this);
        vFunctors[ii] = std::make_shared<MTTestActionThreadFunctor>(vClientTests[ii], ii, vSuccesses[ii]);
        m_threadpool.schedule(*vFunctors[ii]);
    }
    m_threadpool.wait();

    double dur = Toc(start);
    ROS_INFO("[Client] Threads done. Took %f sec.", dur);

    for(int ii = 0 ; ii < NUM_THREADS ; ii++) 
    {
        ROS_INFO("ClientTest #%d %s.", ii, (vSuccesses[ii]?"succeeded":"failed"));
    }
}
