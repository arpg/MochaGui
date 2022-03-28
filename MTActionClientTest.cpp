// #include <mt_actionlib/server/mt_simple_action_server.h>

#include "MTActionClientTest.h"

MTTestActionThreadFunctor::MTTestActionThreadFunctor(std::shared_ptr<MTActionClientTest> clientTest, uint threadId, std::string& result, double& duration, double& wait) :
    m_clientTest(clientTest),
    m_threadId(threadId),
    m_resultPtr(&result),
    m_durationPtr(&duration),
    m_waitPtr(&wait)
{
    ROS_DEBUG("Constructing MTTestActionThreadFunctor #%d", m_threadId);
}

MTTestActionThreadFunctor::~MTTestActionThreadFunctor()
{
    ROS_DEBUG("[Functor %d] Destroyed.", m_threadId);
}

void MTTestActionThreadFunctor::operator()()
{
    ROS_DEBUG("Running MTTestActionThreadFunctor #%d", m_threadId);
    SetThreadName((boost::format("MTTestActionThreadFunctor #%d") % m_threadId).str().c_str());

    (*m_waitPtr) = MAX_SERVER_TIME*((double) rand() / (RAND_MAX));

    ROS_DEBUG("[Functor %d] Calling TestAction. Will wait %f sec.", m_threadId, (*m_waitPtr));

    double start = Tic();
    bool finished_in_time = m_clientTest->TestActionCall(m_threadId, (*m_waitPtr), (*m_resultPtr));
    (*m_durationPtr) = Toc(start);

    ROS_DEBUG("[Functor %d] Called TestAction: %s. Supposed to take %f sec. Took %f sec.", m_threadId, (*m_resultPtr).c_str(), (*m_waitPtr), (*m_durationPtr));
    
    return;
}

MTActionClientTest::MTActionClientTest(ros::NodeHandle& private_nh, ros::NodeHandle& nh) :
    m_private_nh(private_nh),
    m_nh(nh)
{
    m_threadpool.size_controller().resize(NUM_THREADS);
    m_private_nh.param("call_rate", m_callRate, 0.2);
    m_callLoop = m_private_nh.createTimer(ros::Duration(1.0/m_callRate), &MTActionClientTest::callLoopFunc, this);
    ROS_DEBUG("[Client] Constructed.");
}
    
MTActionClientTest::~MTActionClientTest()
{
    ROS_DEBUG("[Client] Destroyed.");
}

bool MTActionClientTest::TestActionCall(uint threadId, double delay, std::string& result_str)
{
    result_str = "INITIALIZED";
    ROS_DEBUG("[Client] Calling TestAction #%d.", threadId);

    #ifndef USE_MANY_SERVERS
    MTTestActionClient client("test_server/test_action",true); // seeing how it works to create new client each time
    #else
    MTTestActionClient client("test_server/"+std::to_string(threadId)+"/test_action",true); // seeing how it works to create new client each time
    #endif
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
        actionlib::SimpleClientGoalState state = client.getState();
        result_str = state.toString().c_str();
        ROS_DEBUG("[Client] TestAction #%d finished: %s", threadId, result_str);
        return true;
    }
    else
    {
        result_str = "TIMEOUT";
        ROS_DEBUG("[Client] TestAction #%d failed to finish after %d timeout.", threadId, timeout);
        return false;
    }
}

void MTActionClientTest::callLoopFunc(const ros::TimerEvent& event)
{
    std::vector<std::shared_ptr<MTTestActionThreadFunctor>> vFunctors;
    vFunctors.resize(NUM_THREADS);
    std::vector<std::shared_ptr<MTActionClientTest>> vClientTests;
    vClientTests.resize(NUM_THREADS);
    std::vector<std::string> vResults;
    vResults.resize(NUM_THREADS);
    std::vector<double> vDurations;
    vDurations.resize(NUM_THREADS);
    std::vector<double> vWaits;
    vWaits.resize(NUM_THREADS);

    ROS_INFO("[Client] Starting threads.");
    double start = Tic();

    for(int ii = 0 ; ii < NUM_THREADS ; ii++) 
    {
        vClientTests[ii] = std::make_shared<MTActionClientTest>(*this);
        vFunctors[ii] = std::make_shared<MTTestActionThreadFunctor>(vClientTests[ii], ii, vResults[ii], vDurations[ii], vWaits[ii]);
        m_threadpool.schedule(*vFunctors[ii]);
    }
    m_threadpool.wait();

    double dur = Toc(start);
    double para_dur = std::accumulate(vDurations.begin(), vDurations.end(), decltype(vDurations)::value_type(0.0f));
    double para_wait = std::accumulate(vWaits.begin(), vWaits.end(), decltype(vWaits)::value_type(0.0f));
    ROS_INFO("[Client] Threads done. Took %f sec to run %f sec with delays of %f sec.", dur, para_dur, para_wait);

    for(int ii = 0 ; ii < NUM_THREADS ; ii++) 
    {
        ROS_INFO("ClientTest #%d: wait %.2f, dur %.2f, %s.", ii, vWaits[ii], vDurations[ii], (vResults[ii]).c_str());
    }
}

