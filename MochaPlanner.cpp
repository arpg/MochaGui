#include <CarPlanner/CVarHelpers.h>
#include "MochaPlanner.h"

static bool& g_bUseCentralDifferences = CVarUtils::CreateGetUnsavedCVar("debug.UseCentralDifferences",true);
static double& g_dTimeTarget = CVarUtils::CreateGetUnsavedCVar("debug.TimeTarget",0.00);
//static bool& g_bUseGoalPoseStepping = CVarUtils::CreateGetCVar("debug.UseGoalPoseStepping",false);
static bool& g_bDisableDamping = CVarUtils::CreateGetUnsavedCVar("debug.DisableDamping",false);
static bool& g_bMonotonicCost(CVarUtils::CreateGetUnsavedCVar("debug.MonotonicCost", true,""));
static bool& g_bVerbose(CVarUtils::CreateGetUnsavedCVar("debug.Verbose", false,""));
static bool& g_bTrajectoryCost(CVarUtils::CreateGetUnsavedCVar("debug.TrajectoryCost", true,""));
static int& g_nTrajectoryCostSegments(CVarUtils::CreateGetUnsavedCVar("debug.TrajectoryCostSegments", 10,""));
static int& g_nIterationLimit = CVarUtils::CreateGetUnsavedCVar("planner.IterationLimit", 10, "");
static double& g_dSuccessNorm = CVarUtils::CreateGetUnsavedCVar("debug.SuccessNorm",4.0);

namespace mochapc {
struct ApplyCommandsThreadFunctor {
    ApplyCommandsThreadFunctor(MochaPlanner *pPlanner, Problem& problem,const int index , Eigen::Vector6d& poseOut,Eigen::VectorXd& errorOut,
                               MotionSample& sample, const bool bSolveBoundary = false) :
        m_pPlanner(pPlanner),
        m_Problem(problem),
        m_index(index),
        m_poseOut(poseOut),
        m_dErrorOut(errorOut),
        m_Sample(sample),
        m_bSolveBoundary(bSolveBoundary)

    {}

    void operator()()
    {
        SetThreadName((boost::format("Bullet Simulation Thread #%d") % m_index).str().c_str());

        if(m_bSolveBoundary){
            m_Problem.m_pBoundarySovler->Solve(&m_Problem.m_BoundaryProblem);
        }
        m_poseOut = m_pPlanner->SimulateTrajectory(m_Sample, m_Problem, m_index);
        m_dErrorOut = m_pPlanner->_CalculateSampleError(m_Sample, m_Problem, m_Problem.m_CurrentSolution.m_dMinTrajectoryTime);
        double norm = m_pPlanner->_CalculateErrorNorm(m_Problem, m_dErrorOut);

        // DLOG(INFO) << "Ran ACF." 
        //     << " Index " << m_index << "."
        // //     // << "|  SolveBoundary " << (m_bSolveBoundary ? "true" : "false") << ".\n"
        // //     // << "|  Problem OptParams " << m_Problem.m_CurrentSolution.m_dOptParams.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << ".\n"
        //     << "\n PoseOut (of " << std::to_string(m_Sample.m_vStates.size()) << ") " << m_poseOut.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << "."
        //     << "\n ErrorsOut " << m_dErrorOut.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << "."
        //     ;

        ROS_INFO("Ran ACF, idx %d, final pose [%s], error [%s], norm %f",
            m_index,
            convertEigenMatrix2String(m_poseOut.transpose()).c_str(), 
            convertEigenMatrix2String(m_dErrorOut.transpose()).c_str(),
            norm);

        return;
    }

    MochaPlanner *m_pPlanner;
    //const double m_startingCurvature;
    //const double m_dt;
    Problem& m_Problem;
    const int m_index;
    Eigen::Vector6d& m_poseOut;
    Eigen::VectorXd& m_dErrorOut;
    MotionSample& m_Sample;
    bool m_bSolveBoundary;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

inline Eigen::VectorXd GetPointLineError(const Eigen::Vector6d& line1,const Eigen::Vector6d& line2, const Eigen::Vector6d& point, double& dInterpolationFactor)
{
    Eigen::Vector3d vAB = (line2.head(3) - line1.head(3));
    Eigen::Vector3d vAC = (point.head(3) - line1.head(3));
    double dAB = vAB.norm();
    double dProjection = vAC.dot(vAB.normalized());
//    if(dProjection < 0){
//        DLOG(INFO) << "Negative vector projection of " << dProjection << " when calculating point line error...";
//    }
    dInterpolationFactor = std::max(std::min(dProjection/dAB,1.0),0.0);

//    if(interpolation > 1.0 || interpolation < 0){
//        DLOG(INFO) << "Point/line interpolation factor out of bounds at " << interpolation << "Interpolating between [" << line1.head(3).transpose() << "], [" << line2.head(3).transpose() << "] and [" << point.head(3).transpose();
//    }
    //now calculate the interpolated value
    Eigen::Vector6d intVal = (1.0-dInterpolationFactor)*line1 + (dInterpolationFactor)*line2;
    intVal[3] = rpg::AngleWrap(intVal[3]);

    //and now we can calculate the error to the interpolated pose
    Eigen::VectorXd intError(TRAJ_UNIT_ERROR_TERMS);
    intError.head(3) = intVal.head(3) - point.head(3);
    intError[3] = rpg::AngleWrap(intVal[3] - point[3]);
    intError[4] = intVal[5] - point[5];
    return intError;
}

// MochaPlanner::MochaPlanner() :
//     m_dEps(CVarUtils::CreateUnsavedCVar("planner.Epsilon", 1e-6, "Epsilon value used in finite differences.")),
//     m_dPointWeight(CVarUtils::CreateUnsavedCVar("planner.PointCostWeights",Eigen::MatrixXd(1,1))),
//     m_dTrajWeight(CVarUtils::CreateUnsavedCVar("planner.TrajCostWeights",Eigen::MatrixXd(1,1))),
//     m_nPlanCounter(0)
// {
    
// }

MochaPlanner::MochaPlanner(ros::NodeHandle& private_nh, ros::NodeHandle& nh) :
    m_private_nh(&private_nh),
    m_nh(&nh),
    m_dEps(CVarUtils::CreateUnsavedCVar("planner.Epsilon", 0.1, "Epsilon value used in finite differences.")),
    m_dPointWeight(CVarUtils::CreateUnsavedCVar("planner.PointCostWeights",Eigen::MatrixXd(1,1))),
    m_dTrajWeight(CVarUtils::CreateUnsavedCVar("planner.TrajCostWeights",Eigen::MatrixXd(1,1))),
    m_nPlanCounter(0),
    m_dTimeInterval(CVarUtils::CreateCVar("planner.TimeInterval", 0.01, "") ),
    m_actionApplyVelocities_client("plan_car/apply_velocities",true),
    m_actionGetInertiaTensor_client("plan_car/get_inertia_tensor",true),
    m_actionSetNoDelay_client("plan_car/set_no_delay",true),
    m_actionRaycast_client("plan_car/raycast",true),
    m_actionSetState_client("plan_car/set_state",true),
    m_actionGetState_client("plan_car/get_state",true),
    m_bServersInitialized(false),
    m_bSubToVehicleWp(true), 
    m_dWpLookupDuration(5.0)
{
    m_private_nh->param("params_file", m_config.params_file, m_config.params_file);
    m_private_nh->param("terrain_mesh_file", m_config.terrain_mesh_file, m_config.terrain_mesh_file);

    m_ThreadPool.size_controller().resize(OPT_DIM*2+1);

    bestProblem.Reset();
    bestProblem.m_nPlanId = -1;
    // bestProblem.m_CurrentSolution.m_dNorm 

    //weight matrix
    m_dPointWeight = Eigen::MatrixXd(POINT_COST_ERROR_TERMS,1);
    m_dTrajWeight = Eigen::MatrixXd(TRAJ_EXTRA_ERROR_TERMS+TRAJ_UNIT_ERROR_TERMS,1);
    m_dPointWeight.setIdentity();
    m_dTrajWeight.setIdentity();

    m_dPointWeight(0) = X_WEIGHT;
    m_dPointWeight(1) = Y_WEIGHT;
    m_dPointWeight(2) = Z_WEIGHT;
    m_dPointWeight(3) = THETA_WEIGHT;
    m_dPointWeight(4) = VEL_WEIGHT_POINT;
    // m_dPointWeight(5) = CURV_WEIGHT;
    m_dPointWeight(5) = TILT_WEIGHT;
    m_dPointWeight(6) = CONTACT_WEIGHT;

    m_dTrajWeight(0) = X_WEIGHT;
    m_dTrajWeight(1) = Y_WEIGHT;
    m_dTrajWeight(2) = Z_WEIGHT;
    m_dTrajWeight(3) = THETA_WEIGHT;
    m_dTrajWeight(4) = VEL_WEIGHT_TRAJ;
    m_dTrajWeight(5) = TIME_WEIGHT;
    m_dTrajWeight(6) = CURV_WEIGHT;
    //m_dTrajWeight(7) = BADNESS_WEIGHT;

    m_timerPubLoop = m_private_nh->createTimer(ros::Duration(0.1), &MochaPlanner::PubLoopFunc, this);

    if (m_bSubToVehicleWp) {
        // m_subVehicleWp = m_nh->subscribe("vehicle_waypoint", 10, boost::bind(&MochaPlanner::vehicleWpCb, this, _1));
        m_subVehicleWp = m_private_nh->subscribe("/plan_car/vehicle_waypoint", 1, &MochaPlanner::vehicleWpCb, this);
    } else {
        m_timerWpLookup = m_private_nh->createTimer(ros::Duration(m_dWpLookupDuration), boost::bind(&MochaPlanner::vehicleWpLookupFunc, this, _1));
    }

    // m_subGoalWp = m_nh->subscribe("goal_waypoint", 10, boost::bind(&MochaPlanner::goalWpCb, this, _1));
    m_subGoalWp = m_private_nh->subscribe("/plan_car/goal_waypoint", 1, &MochaPlanner::goalWpCb, this);

    m_subWaypoints = m_private_nh->subscribe("/plan_car/input_waypoints", 1, &MochaPlanner::waypointsCb, this);

    // m_pubPlan = m_private_nh->advertise<nav_msgs::Path>("opt_plan", 10);

    m_pubWaypoints = m_nh->advertise<geometry_msgs::PoseArray>("/plan_car/waypoints",1);
    m_pubSimPath = m_private_nh->advertise<nav_msgs::Path>("/plan_car/sim_traj", 1);
    m_pubActualTraj = m_private_nh->advertise<nav_msgs::Path>("/plan_car/actual_traj", 1);
    m_pubControlTraj = m_private_nh->advertise<nav_msgs::Path>("/plan_car/control_traj", 1);

    // m_actionApplyVelocities_client.waitForServer();

    // m_actionGetControlDelay_client = new actionlib::SimpleActionClient<carplanner_msgs::GetControlDelayAction>("plan_car/get_control_delay",true);
    // m_actionGetControlDelay_client->waitForServer();

    // m_actionGetInertiaTensor_client = new actionlib::SimpleActionClient<carplanner_msgs::GetInertiaTensorAction>("plan_car/get_inertia_tensor",true);
    // m_actionGetInertiaTensor_client.waitForServer();

    // // m_actionSetNoDelay_client = new actionlib::SimpleActionClient<carplanner_msgs::SetNoDelayAction>("plan_car/set_no_delay",true);
    
    // m_actionSetNoDelay_client.waitForServer();

    m_srvEnableTerrainPlanning_server = m_nh->advertiseService("plan_car/enable_terrain_planning", &MochaPlanner::EnableTerrainPlanningSvcCb, this);

    m_timerPlanningLoop = m_private_nh->createTimer(ros::Duration(1.0), &MochaPlanner::PlanningLoopFunc, this);
    m_bPlanContinuously = false;
    m_srvEnableContinuousPlanning_server = m_nh->advertiseService("plan_car/enable_continuous_planning", &MochaPlanner::EnableContinuousPlanningSvcCb, this);

    m_srvReplay_server = m_nh->advertiseService("plan_car/replay", &MochaPlanner::ReplaySvcCb, this);

    while( !m_bServersInitialized && ros::ok()) 
    {
        m_bServersInitialized = 
        // m_actionApplyVelocities_client.isServerConnected()
        // && m_actionGetControlDelay_client->isServerConnected()  
        m_actionGetInertiaTensor_client.isServerConnected() 
        && m_actionRaycast_client.isServerConnected()
        && m_actionSetNoDelay_client.isServerConnected(); 

        // if (!m_actionApplyVelocities_client.isServerConnected()) 
        // {
        //     DLOG(INFO) << "Waiting for ApplyVelocities Server";
        // }
        // if (!m_actionGetControlDelay_client.isServerConnected()) 
        // {
        //     DLOG(INFO) << "Waiting for GetControlDelay Server";
        // }
        if (!m_actionGetInertiaTensor_client.isServerConnected()) 
        {
            DLOG(INFO) << "Waiting for GetInertiaTensor Server";
        }
        if (!m_actionRaycast_client.isServerConnected()) 
        {
            DLOG(INFO) << "Waiting for Raycast Server";
        }
        if (!m_actionSetNoDelay_client.isServerConnected()) 
        {
            DLOG(INFO) << "Waiting for SetNoDelay Server";
        }
        
        ros::Duration(1).sleep();
    }

    {
        boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
        m_vWaypoints.clear();
        // for(uint i=0; i<2; i++) {
        //     Sophus::SE3d pose;
        //     pose.translation() = *(new Eigen::Vector3d(2+i, 1, 0));
        //     pose.setQuaternion(Eigen::Quaterniond(1, 0, 0, 0));
        
        //     double vel = 1;
        //     double curv = 0;

        //     VehicleState state(pose, vel, curv);
        //     Waypoint* wp = new Waypoint(state);
        //     m_vWaypoints.push_back(wp);

        // }
        // m_vWaypoints.resize(2);
    
        {
            Sophus::SE3d pose;
            pose.translation()[0] = 0.75;
            pose.translation()[1] = -1.25;
            pose.translation()[2] = 0;
            pose.setQuaternion(Eigen::Quaterniond(0.707, 0, 0, 0.707)); // w x y z
        
            double vel = 1;
            double curv = 0;

            VehicleState state(pose, vel, curv);
            Waypoint wp(state);
            AddWaypoint(wp);
        }
        {
            Sophus::SE3d pose;
            pose.translation()[0] = 1.25;
            pose.translation()[1] = 0;
            pose.translation()[2] = 0;
            pose.setQuaternion(Eigen::Quaterniond(0.707, 0, 0, 0.707));
        
            double vel = 1;
            double curv = 0;

            VehicleState state(pose, vel, curv);
            Waypoint wp(state);
            AddWaypoint(wp);
        }
    }

    // DLOG(INFO) << "Initialized."; 

    if (!m_bPlanContinuously)
        replan();

    m_nh->param("x_weight",         m_dPointWeight(0), m_dPointWeight(0));
    m_nh->param("y_weight",         m_dPointWeight(1), m_dPointWeight(1));
    m_nh->param("z_weight",         m_dPointWeight(2), m_dPointWeight(2));
    m_nh->param("theta_weight",     m_dPointWeight(3), m_dPointWeight(3));
    m_nh->param("vel_weight",       m_dPointWeight(4), m_dPointWeight(4));
    m_nh->param("tilt_weight",      m_dPointWeight(5), m_dPointWeight(5));
    m_nh->param("contact_weight",   m_dPointWeight(6), m_dPointWeight(6));
    m_nh->param("eps",              m_dEps,            m_dEps           );

    m_dynReconfig_server.setCallback(boost::bind(&MochaPlanner::dynReconfigCb, this, _1, _2));

    ROS_INFO("Initialized.");   
}

MochaPlanner::~MochaPlanner()
{
    // if(m_pPlannerThread) {
    //     m_pPlannerThread->join();
    // }
    // delete m_pPlannerThread;
    // m_pPlannerThread = 0 ;
}

void MochaPlanner::dynReconfigCb(carplanner_msgs::MochaPlannerConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure requested.");

    m_dPointWeight(0) = config.x_weight;
    m_dPointWeight(1) = config.y_weight;
    m_dPointWeight(2) = config.z_weight;
    m_dPointWeight(3) = config.theta_weight;
    m_dPointWeight(4) = config.vel_weight;
    m_dPointWeight(5) = config.tilt_weight;
    m_dPointWeight(6) = config.contact_weight;
    m_dEps = config.eps;
}

// virtual void MochaPlanner::onInit()
// {
//     nh = getNodeHandle();
//     private_nh = getPrivateNodeHandle();

//     m_timerPlanningLoop = nh.createTimer(ros::Duration(1.0), boost::bind(& MochaPlanner::PlanningLoopFunc, this, _1));

//     if (m_bSubToVehicleWp) {
//         m_subVehicleWp = nh.subscribe("vehicle_waypoint", 10, boost::bind(& MochaPlanner::vehicleWpCb, this, _1));
//     } else {
//         m_dWpLookupDuration = 1;
//         m_timerWpLookup = nh.createTimer(ros::Duration(m_dWpLookupDuration), boost::bind(& MochaPlanner::vehicleWpLookupFunc, this, _1));
//     }

//     m_subGoalWp = nh.subscribe("goal_waypoint", 10, boost::bind(& MochaPlanner::goalWpCb, this, _1));

//     m_pubPlan = nh.advertise<nav_msgs::Path>("path", 10);

//     // m_pPlannerThread = new boost::thread(std::bind(&MochaPlanner::PlannerFunc,this));

//     ////////
//     // m_dEps = CVarUtils::CreateUnsavedCVar("planner.Epsilon", 1e-6, "Epsilon value used in finite differences."));
//     // m_dPointWeight = CVarUtils::CreateUnsavedCVar("planner.PointCostWeights",Eigen::MatrixXd(1,1)));
//     // m_dTrajWeight = CVarUtils::CreateUnsavedCVar("planner.TrajCostWeights",Eigen::MatrixXd(1,1)));
//     // m_nPlanCounter = 0;

//     // m_ThreadPool.size_controller().resize(8);

//     // //weight matrix
//     // m_dPointWeight = Eigen::MatrixXd(POINT_COST_ERROR_TERMS,1);
//     // m_dTrajWeight = Eigen::MatrixXd(TRAJ_EXTRA_ERROR_TERMS+TRAJ_UNIT_ERROR_TERMS,1);
//     // m_dPointWeight.setIdentity();
//     // m_dTrajWeight.setIdentity();

//     // m_dPointWeight(0) = XYZ_WEIGHT;
//     // m_dPointWeight(1) = XYZ_WEIGHT;
//     // m_dPointWeight(2) = XYZ_WEIGHT;
//     // m_dPointWeight(3) = THETA_WEIGHT;
//     // m_dPointWeight(4) = VEL_WEIGHT_POINT;
//     // // m_dPointWeight(5) = CURV_WEIGHT;
//     // m_dPointWeight(5) = TILT_WEIGHT;

//     // m_dTrajWeight(0) = XYZ_WEIGHT;
//     // m_dTrajWeight(1) = XYZ_WEIGHT;
//     // m_dTrajWeight(2) = XYZ_WEIGHT;
//     // m_dTrajWeight(3) = THETA_WEIGHT;
//     // m_dTrajWeight(4) = VEL_WEIGHT_TRAJ;
//     // m_dTrajWeight(5) = TIME_WEIGHT;
//     // m_dTrajWeight(6) = CURV_WEIGHT;
//     // //m_dTrajWeight(7) = BADNESS_WEIGHT;
// };

///////////////////////////////////////////////////////////////////////
void MochaPlanner::SamplePath(const Problem& problem, Eigen::Vector3dAlignedVec& vSamples, bool bBestSolution /* = true */ )
{
    vSamples.clear();
    BezierBoundaryProblem boundaryProblem = problem.m_BoundaryProblem;
    Sophus::SE2d T_start(Sophus::SO2(problem.m_dStartPose[3]),problem.m_dStartPose.head(2));
    vSamples.reserve(boundaryProblem.m_vPts.size());
    for(const Eigen::Vector2d& pt : boundaryProblem.m_vPts) {
        Eigen::Vector3d dPos(pt[0],pt[1],0);
        dPos.head(2) = T_start*dPos.head(2);

        //now transform this into the proper 3d pose
        dPos = problem.m_dT3dInv * dPos;
        vSamples.push_back(dPos);
    }
}

///////////////////////////////////////////////////////////////////////
Eigen::Vector6d MochaPlanner::_Transform3dGoalPose(const VehicleState& state, const Problem &problem) const
{
    //also transfer this pose into the 3d projected space we are working on
    const Sophus::SE3d dPose = problem.m_dT3d * state.m_dTwv;
    Eigen::Vector6d untransformedPose;
    untransformedPose << dPose.translation()[0],
                         dPose.translation()[1],
                         dPose.translation()[2],
                         atan2( dPose.matrix()(1,0), dPose.matrix()(0,0)),
                         0,
                         state.m_dV.norm();

    return _TransformGoalPose(untransformedPose,problem);
}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d  MochaPlanner::_TransformGoalPose(const Eigen::Vector6d& dGoalPose,const Problem& problem) const
{
    Eigen::Vector6d result;
    Eigen::Vector3d pt;
    pt << dGoalPose[0], dGoalPose[1], 1;
    pt = problem.m_dTinv * pt;
    result << pt[0], pt[1], dGoalPose[2], rpg::AngleWrap( dGoalPose[3] - problem.m_dStartPose[3] ), dGoalPose[4], dGoalPose[5];
    return result;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd MochaPlanner::_GetTrajectoryError(const MotionSample& sample,
                                                  const Eigen::Vector6dAlignedVec& vTransformedPoses,
                                                  const Eigen::Vector6d& endPose,
                                                  double& dMinTime) const
{
    Eigen::VectorXd error(TRAJ_UNIT_ERROR_TERMS);
    int nTrajSize = sample.m_vStates.size();
    Eigen::Vector6d minPose, minPoseBefore, minPoseAfter;
    int nMinIdx = -1;
    //now find the closes point in the trajectory to our current position
    double dMinDist = DBL_MAX;
    for(int ii = 0; ii < nTrajSize ; ii++){
        //find the closest point to our current location
        Eigen::Vector3d cartError = vTransformedPoses[ii].head(3) - endPose.head(3);
        double norm = (cartError).norm();
        if(norm < dMinDist){
            dMinTime = sample.m_vStates[ii].m_dTime;
            minPose = vTransformedPoses[ii];;
            minPoseAfter = ii < (nTrajSize-1) ? vTransformedPoses[ii+1] : minPose;
            minPoseBefore = ii > 0 ? vTransformedPoses[ii-1] : minPose;
            nMinIdx = ii;
            dMinDist = norm;
        }
    }

    //reset the error
    error.setZero();

    //calculate the distance at the minimum
    error.head(3) = minPose.head(3) - endPose.head(3);
    error[3] = rpg::AngleWrap(minPose[3] - endPose[3]);
    error[4] = minPose[5] - endPose[5];

    //now calculate the distance on both sides and find the minimum
    double dInterpolationFactor;
    Eigen::VectorXd beforeError;
    if(minPose != minPoseBefore){
        beforeError = GetPointLineError(minPoseBefore,minPose,endPose,dInterpolationFactor);
        if(beforeError.head(3).norm() < error.head(3).norm()){
            dMinTime = (1.0-dInterpolationFactor)*sample.m_vStates[nMinIdx-1].m_dTime + dInterpolationFactor*sample.m_vStates[nMinIdx].m_dTime;
            error = beforeError;
        }
    }


    Eigen::VectorXd afterError;
    if(minPose != minPoseAfter){
        afterError = GetPointLineError(minPose,minPoseAfter,endPose,dInterpolationFactor);
        if(afterError.head(3).norm() < error.head(3).norm()){
            dMinTime = (1.0-dInterpolationFactor)*sample.m_vStates[nMinIdx].m_dTime + dInterpolationFactor*sample.m_vStates[nMinIdx+1].m_dTime;
            error = afterError;
        }
    }
    //error[4] = minPose[5] - endPose[5];
//    for(int ii = 0; ii < error.rows() ; ii++){
//        error[ii] = fabs(error[ii]);
//    }
    return error;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd MochaPlanner::_GetWeightVector(const Problem& problem)
{
    int errorVecSize = problem.m_eCostMode == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd dW;
    Eigen::VectorXd trajW(errorVecSize);
    if(problem.m_eCostMode == eCostTrajectory){
        double dFactor = 1.0/((double)g_nTrajectoryCostSegments);
        for(int ii = 0; ii < g_nTrajectoryCostSegments ; ii++){
            trajW.segment(ii*TRAJ_UNIT_ERROR_TERMS,TRAJ_UNIT_ERROR_TERMS) = m_dTrajWeight.col(0).head(TRAJ_UNIT_ERROR_TERMS)*dFactor;
            dFactor *= 2;
        }
        trajW.tail(TRAJ_EXTRA_ERROR_TERMS) = m_dTrajWeight.col(0).tail(TRAJ_EXTRA_ERROR_TERMS);
        dW = trajW;
        //DLOG(INFO) << "Trjaectory weights are: " << trajW.transpose();
    }else{
        dW = m_dPointWeight.col(0).head(POINT_COST_ERROR_TERMS);
    }
    return dW;
}

///////////////////////////////////////////////////////////////////////
double MochaPlanner::_CalculateErrorNorm(const Problem& problem,const Eigen::VectorXd& dError)
{
    Eigen::VectorXd dW = _GetWeightVector(problem);
    Eigen::VectorXd error = dError;
    //DLOG(INFO) << "error vector is " << error.transpose();
    error.array() *= dW.array();
    return error.norm();
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd MochaPlanner::_CalculateSampleError(const MotionSample& sample, Problem& problem, double& dMinTrajTime) const
{
    int errorVecSize = problem.m_eCostMode == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd error;
    if(sample.m_vStates.size() == 0 ){
        // DLOG(ERROR) << problem.m_nPlanId << ":Sample with size 0 detected. Aborting.";
        ROS_INFO_THROTTLE(.1, "%d:Sample with size 0 detected. Aborting.", problem.m_nPlanId);
        error = Eigen::VectorXd(errorVecSize);
        error.setOnes();
        error *= DBL_MAX;
        return error;
    }
    //get the normalized velocity
    VehicleState state = sample.m_vStates.back();
    Eigen::Vector3d dW_goal = problem.m_GoalState.m_dTwv.so3().inverse()* state.m_dW;
    //double dPrevAngle = state.GetTheta();
    if(state.IsAirborne()){
        state.AlignWithVelocityVector();
        //DLOG(INFO) << "End state transformed from " << dPrevAngle << " to " << state.GetTheta();
    }

    Eigen::Vector6d endPose = _Transform3dGoalPose(state,problem);
    if(problem.m_eCostMode == eCostPoint){
        error =Eigen::VectorXd(errorVecSize);
        //Eigen::Vector3d cartError = T2Cart_2D(Cart2T_2D(problem.m_dTransformedGoal.head(3))*Tinv_2D(Cart2T_2D(endPose.head(3))));
        //error.head(3) = cartError;
        //transform the angular velocity into the plane of goal pose
        
        error.head(3) = problem.m_dTransformedGoal.head(3) - endPose.head(3);
        error[3] = rpg::AngleWrap(problem.m_dTransformedGoal[3] - endPose[3]);
        error[4] = problem.m_dTransformedGoal[5] - endPose[5];
        //error[5] = state.m_dV.norm()*dW_goal[2] - problem.m_GoalState.m_dCurvature;

        error[5] = sample.GetTiltCost();
        error[6] = sample.GetContactCost();
        //error[5] = -std::log(problem.m_BoundaryProblem.m_dAggressiveness);
        //error.array() *= m_dPointWeight.block<POINT_COST_ERROR_TERMS,1>(0,0).array();
        //DLOG(INFO) << "Error vector is " << error.transpose() << " weights are " << m_dPointWeight.transpose();
    }else if(problem.m_eCostMode == eCostTrajectory){
        error =Eigen::VectorXd(errorVecSize);

        if(problem.m_vTransformedTrajectory.empty()){
            int nTrajSize = problem.m_Trajectory.m_vStates.size();
            problem.m_vTransformedTrajectory.reserve(nTrajSize);
            //push back the transformed poses
            for(int ii = 0; ii < nTrajSize ; ii++){
                VehicleState tempState = problem.m_Trajectory.m_vStates[ii];
                if(tempState.IsAirborne()){
                    tempState.AlignWithVelocityVector();
                }
                problem.m_vTransformedTrajectory.push_back(_Transform3dGoalPose(tempState, problem));
            }
        }


        error.setZero();
        error.segment(TRAJ_UNIT_ERROR_TERMS*(g_nTrajectoryCostSegments-1),TRAJ_UNIT_ERROR_TERMS)  = _GetTrajectoryError(problem.m_Trajectory,
                                                                                                                         problem.m_vTransformedTrajectory,
                                                                                                                         endPose,
                                                                                                                         dMinTrajTime);

        //now that we have the minimum point, calculate trajectory error points along the way
        int counter = 0;
        int nStartIndex = 0;

        for(double ii = dMinTrajTime/(double)g_nTrajectoryCostSegments ;
                   ii < dMinTrajTime && counter < ((double)g_nTrajectoryCostSegments-1) ;
                   ii += dMinTrajTime/(double)g_nTrajectoryCostSegments ){
            //get the poses at this point in time
            //Eigen::Vector6d poseTraj = _Transform3dGoalPose(VehicleState::GetInterpolatedState(problem.m_Trajectory.m_vStates,nStartIndex,ii,nStartIndex),problem);
            Eigen::Vector6d poseSample = _Transform3dGoalPose(VehicleState::GetInterpolatedState(sample.m_vStates,nStartIndex,ii,nStartIndex),problem);

            double dMinTime;
            error.segment(TRAJ_UNIT_ERROR_TERMS*counter,TRAJ_UNIT_ERROR_TERMS) = _GetTrajectoryError(problem.m_Trajectory,
                                                                                                     problem.m_vTransformedTrajectory,
                                                                                                     poseSample,
                                                                                                    dMinTime);
            counter++;
        }
        error /= (double)g_nTrajectoryCostSegments;


        //segment cost

        error[error.rows()-2] = g_dTimeTarget-dMinTrajTime;
        error[error.rows()-1] = state.m_dV.norm()*dW_goal[2] - problem.m_GoalState.m_dCurvature; // sample.GetBadnessCost();

        //error[7] = sample.GetBadnessCost();
        //error[6] = -std::log(problem.m_BoundaryProblem.m_dAggressiveness);
        //error.array() *= m_dTrajWeight.block<TRAJ_COST_ERROR_TERMS,1>(0,0).array();
    }
    return error;
}

///////////////////////////////////////////////////////////////////////
bool MochaPlanner::_CalculateJacobian(Problem& problem,
                                      Eigen::VectorXd& dCurrentErrorVec,
                                      Solution& coordinateDescent,
                                      Eigen::MatrixXd& J)
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd errors[OPT_DIM*2],dCurrentError;
    std::vector<std::shared_ptr<Problem > > vCubicProblems;
    std::vector<std::shared_ptr<mochapc::ApplyCommandsThreadFunctor > > vFunctors;
    vCubicProblems.resize(OPT_DIM*2);
    vFunctors.resize(OPT_DIM*2);
    Eigen::Vector6d pPoses[OPT_DIM*2],dCurrentPose;

    const double dEps = m_dEps;// * problem.m_CurrentSolution.m_dNorm;
    // DLOG(INFO) << "Calcing Jacobian";
    ROS_INFO("Calculating Jacobian.");
    ROS_INFO("Running ACF on %d threads for each +OPT_DIM, -OPT_DIM, current.", OPT_DIM*2+1);

    //g_bUseCentralDifferences = false;
    for( int ii = 0; ii < OPT_DIM; ii++ )
    {
        int plusIdx = ii*2, minusIdx = ii*2+1;
        vCubicProblems[plusIdx] = std::make_shared<Problem>(problem);
        Eigen::VectorXd delta(OPT_DIM);
        delta.setZero();
        delta(ii) += dEps;
        vCubicProblems[plusIdx]->UpdateOptParams(vCubicProblems[plusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
        vFunctors[plusIdx] = std::make_shared<mochapc::ApplyCommandsThreadFunctor>(this,
                                                                          *vCubicProblems[plusIdx],
                                                                          plusIdx,
                                                                          pPoses[plusIdx],
                                                                          errors[plusIdx],
                                                                          (vCubicProblems[plusIdx]->m_CurrentSolution.m_Sample),
                                                                          true);
        m_ThreadPool.schedule(*vFunctors[plusIdx]);

        if(g_bUseCentralDifferences == true){
            vCubicProblems[minusIdx] = std::make_shared<Problem>(problem);
            Eigen::VectorXd delta(OPT_DIM);
            delta.setZero();
            delta(ii) -= dEps;
            vCubicProblems[minusIdx]->UpdateOptParams(vCubicProblems[minusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vFunctors[minusIdx] = std::make_shared<mochapc::ApplyCommandsThreadFunctor>(this,
                                                                               *vCubicProblems[minusIdx],
                                                                               minusIdx,
                                                                               pPoses[minusIdx],
                                                                               errors[minusIdx],
                                                                               (vCubicProblems[minusIdx]->m_CurrentSolution.m_Sample),
                                                                               true);
            m_ThreadPool.schedule(*vFunctors[minusIdx]);
        }
    }

    std::shared_ptr<Problem >currentProblem = std::make_shared<Problem>(problem);
    std::shared_ptr<mochapc::ApplyCommandsThreadFunctor > currentFunctor =
        std::make_shared<mochapc::ApplyCommandsThreadFunctor>(this,
                                                     *currentProblem,
                                                     OPT_DIM*2+1,
                                                     dCurrentPose,
                                                     dCurrentError,
                                                     (currentProblem->m_CurrentSolution.m_Sample),
                                                     true);
    m_ThreadPool.schedule(*currentFunctor);


    //wait for all simulations to finish
    // DLOG(INFO) << "Waiting for threads to finish...";
    ROS_INFO("Waiting for ACF threads to finish...");
    // boost::xtime t = boost::xtime::system_time();
    // // t.system_time();
    // t.sec += 1;
    m_ThreadPool.wait();
    // DLOG(INFO) << "Done waiting.";
    ROS_INFO("Done waiting.");
    ROS_INFO("Calculating coordinate descent solution.");

    dCurrentErrorVec = dCurrentError;

    std::shared_ptr<Problem> pCoordinateDescent;
    double dBestNorm = DBL_MAX;

    for( int ii = 0; ii < OPT_DIM ; ii++ ){
        int plusIdx = ii*2, minusIdx = ii*2+1;
        double norm = _CalculateErrorNorm(*vCubicProblems[plusIdx],errors[plusIdx]);
        if(std::isfinite(norm)){
            if( norm < dBestNorm ) {
                dBestNorm = norm;
                pCoordinateDescent = (vCubicProblems[plusIdx]);
            }
        }


        if(g_bUseCentralDifferences == true){
            norm = _CalculateErrorNorm(*vCubicProblems[minusIdx],errors[minusIdx]);
            if(std::isfinite(norm)){
                if( norm < dBestNorm ) {
                    dBestNorm = norm;
                    pCoordinateDescent = (vCubicProblems[plusIdx]);
                }
            }
        }

        if(g_bVerbose){
            // DLOG(INFO) << "Dimension " << ii << " norm " << norm << " error-> [" << errors[plusIdx].transpose().format(CleanFmt) << "] vs. ["  << dCurrentErrorVec.transpose().format(CleanFmt);
        }

        //now that we have all the error terms, we can set this column of the jacobians

        Eigen::VectorXd col = g_bUseCentralDifferences ? ((errors[plusIdx]) - (errors[minusIdx]))/(2.0*dEps) : ((errors[plusIdx]) - dCurrentErrorVec)/(dEps);

        // std::string str = "Col "+std::to_string(ii)+" ";
        // for(uint i=0; i<col.size(); i++){
        //     str += std::to_string(col(ii)) + " ";
        // }
        // // DLOG(INFO) << str;
        // if (g_bUseCentralDifferences)
        // {
        //     str += "with centraldiff. errors+ ";
        //     str << errors[plusIdx].format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]"));
        //     str += " errors- ";
        //     str << errors[minusIdx].format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]"));
        //     str += " dEps " + std::to_string(dEps);
        // } 
        // else
        // {
        //     str += "without centraldiff. errors+ " ;
        //     str << errors[plusIdx].format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) ;
        //     str += " currerr " ;
        //     str << dCurrentErrorVec.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) ;
        //     str += " dEps " + std::to_string(dEps) ;
        // }
        // ROS_DEBUG(str);

        J.col(ii) = -col;
        //if this term is NAN, sound the alarm
        if(std::isfinite(col[0]) == false || std::isfinite(col[1]) == false ||
           std::isfinite(col[2]) == false || std::isfinite(col[3]) == false){
            problem.m_eError = eJacobianColumnNan;
            return false;
        }

        //if this column is zero
//        if( col.norm() == 0){
//            problem.m_eError = eJacobianColumnZero;
//            return false;
//        }
    }

    coordinateDescent = pCoordinateDescent->m_CurrentSolution;
    coordinateDescent.m_dNorm = dBestNorm;
    
    ROS_INFO("Done calculating Jacobian\n%s\nwith curr norm %f, CD norm %f\nerrors [%s]", convertEigenMatrix2String(J,2,", ","\n","\t").c_str(), problem.m_CurrentSolution.m_dNorm, dBestNorm, convertEigenMatrix2String(dCurrentErrorVec.transpose()).c_str());

    if(g_bVerbose){
        DLOG(INFO) << "Jacobian:" << J.format(CleanFmt) << std::endl;
    }
    return true;
}


///////////////////////////////////////////////////////////////////////
double MochaPlanner::_DistanceTraveled( const double& t,const AccelerationProfile& profile ) const
{
    double totalDist = 0;
    double lastTime = 0;
    //go through each profile segment until we find the one that we're in
    for(size_t ii = 0 ; ii < profile.size() ; ii++){
        if(profile[ii].m_dEndTime < t && ii != profile.size()-1){
            totalDist += profile[ii].m_dEndDist;
            lastTime = profile[ii].m_dEndTime;
            continue;
        }else{
            double dt = t - lastTime;
            totalDist += profile[ii].m_dVStart * dt + (profile[ii].m_dVEnd - profile[ii].m_dVStart)*(dt * dt) / (2.0 * (profile[ii].m_dEndTime-lastTime));
            break;
        }
    }
    return totalDist;
}

///////////////////////////////////////////////////////////////////////
void MochaPlanner::SampleAcceleration(std::vector<ControlCommand>& vCommands, Problem& problem) const
{
    vCommands.clear();

    //double st;// = SegmentTime(p);
    //AccelerationProfile accelProfile;
    _GetAccelerationProfile(problem);
    //problem.m_dSegmentTime += problem.m_dSegmentTimeDelta;
    if(std::isfinite(problem.m_dSegmentTime) == false ){
        // DLOG(ERROR) << problem.m_nPlanId << ":Segment time of " << problem.m_dSegmentTime << " was not finite.";
        ROS_INFO_THROTTLE(.1,"%d:Segment time of %d was not finite.", problem.m_nPlanId, problem.m_dSegmentTime);
        return;
    }
    double t;
    problem.m_dSegmentTime = std::min(problem.m_dSegmentTime,problem.m_dMaxSegmentTime);
    int numSamples = (int)(problem.m_dSegmentTime/problem.m_dT + 1.0);
    vCommands.reserve(numSamples);

    size_t accelIndex = 0;
    for( t = 0; t < (problem.m_dSegmentTime) ; t+= problem.m_dT  ){
        double curvature = problem.m_pBoundarySovler->GetCurvature(&problem.m_BoundaryProblem,
                                                                   _DistanceTraveled(t,problem.m_vAccelProfile));

        double step = problem.m_dSegmentTime - t;
        double actualDt = std::min(problem.m_dT,step);

        if(problem.m_vAccelProfile[accelIndex].m_dEndTime < t){
            accelIndex++;
        }

        if(accelIndex >= problem.m_vAccelProfile.size()){
            DLOG(ERROR) << problem.m_nPlanId << ":Exceeded bounds of acceleration profile.";
            return;
        }

        //if needed, add torques
        double endTime = problem.m_dTorqueStartTime + problem.m_dTorqueDuration;
        Eigen::Vector3d dTorques = Eigen::Vector3d::Zero();
        if(t >= problem.m_dTorqueStartTime && problem.m_dTorqueStartTime != -1 && t <= (endTime)){
            dTorques(1) = problem.m_dCoefs(0) + problem.m_dCoefs(1)*(t-problem.m_dTorqueStartTime) +
                          problem.m_dCoefs(2)*powi((t-problem.m_dTorqueStartTime),2) +
                          problem.m_dCoefs(3)*powi((t-problem.m_dTorqueStartTime),3);
        }
        double force = problem.m_vAccelProfile[accelIndex].m_dAccel+(problem.m_CurrentSolution.m_dOptParams[OPT_ACCEL_DIM]/problem.m_dSegmentTime);
        vCommands.push_back(ControlCommand(force,curvature,dTorques,actualDt,0));
    }

    //if there is control delay, add empty commands to the end, so that the control delay queue can be flushed out
//    double totalDelay = problem.m_pFunctor->GetCarModel()->GetParameters(iWorld)[CarParameters::ControlDelay];
//    if(totalDelay > 0 && problem.m_pFunctor->GetPreviousCommand().size() != 0){
//        CommandList::iterator it  = problem.m_pFunctor->GetPreviousCommand().begin();
//        while(totalDelay > 0 && it != problem.m_pFunctor->GetPreviousCommand().end()){
//            vCommands.push_back(ControlCommands(0,0,Eigen::Vector3d::Zero(),problem.m_dT,0));
//            totalDelay -= problem.m_dT;
//            ++it;
//        }
//    }
}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d MochaPlanner::SimulateTrajectory(MotionSample& sample,
                                                 Problem& problem,
                                                 const int iWorld /*= 0*/,
                                                 const bool& bBestSolution /* = false */)
{
    sample.Clear();
    bool bUsingBestSolution = false;
    if(bBestSolution && problem.m_pBestSolution != NULL && problem.m_pBestSolution->m_Sample.m_vCommands.size() != 0){
        bUsingBestSolution = true;
        sample.m_vCommands = problem.m_pBestSolution->m_Sample.m_vCommands;
    }else{
        // calc sample.commands from problem velocity profile
        SampleAcceleration(sample.m_vCommands, problem);
    }

    VehicleState vState;
    if(sample.m_vCommands.size() == 0){
        vState = problem.m_StartState;
    }else {
        // augment sample.commands with compensation (gravity, steering, friction) for RaycastVehicle 
        // and use with BulletCarModel::UpdateState to populate sample.states

        // DLOG(INFO) << "Commands(" << sample.m_vCommands.size() << "):";
        // for(uint i=0; i<sample.m_vCommands.size(); i++) {
        //     DLOG(INFO) << "  " << sample.m_vCommands[i].m_dForce << " " << sample.m_vCommands[i].m_dCurvature << " " << sample.m_vCommands[i].m_dT << " " << sample.m_vCommands[i].m_dPhi << " " << sample.m_vCommands[i].m_dTorque.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << " " << sample.m_vCommands[i].m_dTime;
        // }

        ApplyVelocities( problem.m_StartState, sample, iWorld, bUsingBestSolution);


        // DLOG(INFO) << "States(" << sample.m_vStates.size() << "):";
        // for(uint i=0; i<sample.m_vStates.size(); i++) {
        //     DLOG(INFO) << "  " << sample.m_vStates[i].ToXYZTCV().format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]"));
        // }

        if (sample.m_vStates.size()>0)
            vState = sample.m_vStates.back();
        else
            vState = VehicleState();
    }
    //transform the result back
    Eigen::Vector6d dRes = _Transform3dGoalPose(vState,problem);
    return dRes;
}

///////////////////////////////////////////////////////////////////////
void MochaPlanner::_GetAccelerationProfile(Problem& problem) const
{
    double totalDist = problem.m_BoundaryProblem.m_dDistance;
    double currentDist = 0;
    double totalTime = 0;

    //must have at least two nodes
    assert(problem.m_vVelProfile.size()>1);

    //prepare the accel profile
    problem.m_vAccelProfile.clear();
    problem.m_vAccelProfile.reserve(problem.m_vVelProfile.size());

    //first calcualte the segment time
    for(size_t ii = 1; ii < problem.m_vVelProfile.size(); ii++){
        //calculate the distance in this segment
        double segDist = (problem.m_vVelProfile[ii].m_dDistanceRatio - problem.m_vVelProfile[ii-1].m_dDistanceRatio)*totalDist;
        double segTime = segDist / (problem.m_vVelProfile[ii-1].m_dVel + 0.5 * (problem.m_vVelProfile[ii].m_dVel - problem.m_vVelProfile[ii-1].m_dVel));
        totalTime += segTime;
        currentDist += segDist;

        //push back the accel profile
        double accel = (problem.m_vVelProfile[ii].m_dVel - problem.m_vVelProfile[ii-1].m_dVel)/segTime;
        problem.m_vAccelProfile.push_back(AccelerationProfileNode(totalTime,accel,currentDist,problem.m_vVelProfile[ii-1].m_dVel,problem.m_vVelProfile[ii].m_dVel));
    }
    problem.m_dSegmentTime = totalTime;
    //problem.m_dMaxSegmentTime = DBL_MAX;
}

// void MochaPlanner::ApplyVelocitiesDoneCb(const actionlib::SimpleClientGoalState& state,
//     const carplanner_msgs::ApplyVelocitiesResultConstPtr& result)
// {
//     DLOG(INFO) << "AV done:" 
//             << " world " << std::to_string(result->world_id) 
//             << " " << state.toString()
//             // << " poseOut " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x) 
//             //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x)  
//             //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.y)
//             //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.z) 
//             ;

//     if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//         {
//         boost::mutex::scoped_lock lock(m_mutexApplyVelocitiesInfo);
//         for (uint i=0; i<m_vApplyVelocitiesGoalIds.size(); i++)
//         {
//             if (m_vApplyVelocitiesGoalIds[i]==state)
//         }
//         }
//     }
//     else
//         DLOG(INFO) << "ApplyVelocities " << std::to_string(nWorldId) << ") did not finish before the time out.";

//     ros::shutdown();
// }

void MochaPlanner::ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int nWorldId /*= 0*/,
                                                      bool noCompensation /*= false*/) {
    // ApplyVelocities(startState,
    //                 sample.m_vCommands,
    //                 sample.m_vStates,
    //                 0,
    //                 sample.m_vCommands.size(),
    //                 nWorldId,
    //                 noCompensation,
    //                 NULL);
    // return sample.m_vStates.back();

    // boost::thread spin_thread(&spinThread);

    actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction> actionApplyVelocities_client("plan_car/apply_velocities/"+std::to_string(nWorldId),true);
    actionApplyVelocities_client.waitForServer();

    carplanner_msgs::ApplyVelocitiesGoal goal;
    carplanner_msgs::ApplyVelocitiesResultConstPtr result;
    
    goal.initial_state = startState.toROS();
    goal.initial_motion_sample = sample.toROS();
    goal.world_id = nWorldId;
    goal.no_compensation = noCompensation;
    actionApplyVelocities_client.sendGoal(goal
        // , boost::bind(&MochaPlanner::ApplyVelocitiesDoneCb, this, _1, _2)
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
        );

    // {
    // boost::mutex::scoped_lock lock(m_mutexApplyVelocitiesInfo);
    // m_vApplyVelocitiesGoalIds.push_back(goal.goal_id);
    // m_vApplyVelocitiesMotionSamples.push_back(&sample);
    // }

    DLOG(INFO) << "AV called:" 
        << " world " << std::to_string(goal.world_id) 
      // "\nstart " << std::to_string(VehicleState::fromROS(goal->initial_state))
      ;

    float timeout(15.0);
    bool finished_before_timeout = actionApplyVelocities_client.waitForResult(ros::Duration(timeout));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = actionApplyVelocities_client.getState();
        // DLOG(INFO) << "ApplyVelocities finished: " << state.toString();

        result = actionApplyVelocities_client.getResult();
        sample.fromROS(result->motion_sample);
    }
    else
        DLOG(INFO) << "ApplyVelocities (" << std::to_string(nWorldId) << ") did not finish before the " << std::to_string(timeout) << "s time out.";

    // spin_thread.join();

    // while (actionApplyVelocities_client.getState() == actionlib::SimpleClientGoalState::PENDING
    //     || actionApplyVelocities_client.getState() == actionlib::SimpleClientGoalState::ACTIVE
    //     )
    // {  
    //     DLOG(INFO) << std::to_string(goal.world_id) << ": " << actionApplyVelocities_client.getState().toString();
    //     ros::Rate(10).sleep();
    // }

    // result = actionApplyVelocities_client.getResult();
    // sample.fromROS(result->motion_sample);

    DLOG(INFO) << "AV done:" 
        << " world " << std::to_string(goal.world_id) 
        << " " << actionApplyVelocities_client.getState().toString()
        // << " poseOut " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x) 
        //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x)  
        //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.y)
        //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.z) 
        ;

    return;
}

double MochaPlanner::GetControlDelay(int nWorldId)
{
    double valOut;

    carplanner_msgs::GetControlDelayGoal goal;
    carplanner_msgs::GetControlDelayResultConstPtr result;
    
    goal.worldId = nWorldId;
    m_actionGetControlDelay_client->sendGoal(goal);

    bool finished_before_timeout = m_actionGetControlDelay_client->waitForResult(ros::Duration(0.5));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_actionGetControlDelay_client->getState();
        // ROS_INFO("GetControlDelay Action finished: %s", state.toString().c_str());

        result = m_actionGetControlDelay_client->getResult();
        valOut = result->val;
    }
    else
        ROS_INFO("GetControlDelay Action did not finish before the time out.");

    return valOut;

    // problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
}

const Eigen::Vector3d MochaPlanner::GetInertiaTensor(int nWorldId)
{
    carplanner_msgs::GetInertiaTensorGoal goal;
    carplanner_msgs::GetInertiaTensorResultConstPtr result;

    goal.world_id = nWorldId;
    m_actionGetInertiaTensor_client.sendGoal(goal);

    bool finished_before_timeout = m_actionGetInertiaTensor_client.waitForResult(ros::Duration(0.5));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_actionGetInertiaTensor_client.getState();
        // ROS_INFO("GetInertiaTensor Action finished: %s", state.toString().c_str());

        result = m_actionGetInertiaTensor_client.getResult();
    }
    else
    {
        ROS_INFO("GetInertiaTensor Action did not finish before the time out.");
        return Eigen::Vector3d();
    }

    const Eigen::Vector3d valsOut(result->vals[0], result->vals[1], result->vals[2]);
    return valsOut;
}

void MochaPlanner::SetNoDelay(bool no_delay)
{
    carplanner_msgs::SetNoDelayGoal goal;
    carplanner_msgs::SetNoDelayResultConstPtr result;

    goal.no_delay = no_delay;
    m_actionSetNoDelay_client.sendGoal(goal);

    bool finished_before_timeout = m_actionSetNoDelay_client.waitForResult(ros::Duration(0.5));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_actionSetNoDelay_client.getState();
        // ROS_INFO("SetNoDelay Action finished: %s", state.toString().c_str());

    }
    else
    {
        ROS_INFO("SetNoDelay Action did not finish before the time out.");
    }
    return;
}

bool MochaPlanner::EnableTerrainPlanningSvcCb(carplanner_msgs::EnableTerrainPlanning::Request &req, carplanner_msgs::EnableTerrainPlanning::Response &res)
{   
    DLOG(INFO) << "EnableTerrainPlanning service called.";

    {
    boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    for(uint i=0; i<m_vWaypoints.size(); i++)
    {
        m_vWaypoints[i]->isDirty = true;
    }
    }

    EnableTerrainPlanning(req.do_plan);

    if (!m_bPlanContinuously)
        replan();

    return true;
}

bool MochaPlanner::EnableContinuousPlanningSvcCb(carplanner_msgs::EnableContinuousPlanning::Request &req, carplanner_msgs::EnableContinuousPlanning::Response &res)
{   
    DLOG(INFO) << "EnableContinuousPlanning service called.";

    EnableContinuousPlanning(req.plan_continuously);

    return true;
}

bool MochaPlanner::Raycast(const Eigen::Vector3d& dSource, const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool& biDirectional, int index /*= 0*/)
{
    carplanner_msgs::RaycastGoal goal;
    carplanner_msgs::RaycastResultConstPtr result;

    goal.source.x = dSource[0];
    goal.source.y = dSource[1];
    goal.source.z = dSource[2];
    goal.ray.x = dRayVector[0];
    goal.ray.y = dRayVector[1];
    goal.ray.z = dRayVector[2];
    goal.bidirectional = biDirectional;
    goal.index = index;

    // while (m_actionRaycast_client.getState() == actionlib::SimpleClientGoalState::PENDING
    //     ||  m_actionRaycast_client.getState() == actionlib::SimpleClientGoalState::ACTIVE
    //     )
    // {
    //     ros::Duration(0.1).sleep();
    // }
    
    m_actionRaycast_client.sendGoal(goal);

    bool finished_before_timeout = m_actionRaycast_client.waitForResult(ros::Duration(1.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_actionRaycast_client.getState();
        ROS_INFO("Raycast Action finished: %s", state.toString().c_str());

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            result = m_actionRaycast_client.getResult();
            dIntersect = Eigen::Vector3d(result->intersect.x, result->intersect.y, result->intersect.z);
            // dIntersect[2] -= 0.25; 

            return true;
        }
        else
        {
            ROS_INFO("Raycast Action failed.");
            return false;
        }

    }
    else
    {
        ROS_INFO("Raycast Action did not finish before the time out.");
        return false;
    }
}

bool MochaPlanner::ReplaySvcCb(carplanner_msgs::Replay::Request &req, carplanner_msgs::Replay::Response &res)
{
    DLOG(INFO) << "replay service called.";

    replay(req.rate);

    return true;
}

void MochaPlanner::replay(float rate)
{
    if (rate==0.0) {rate = 1.0;}

    MotionSample last_segment = m_vSegmentSamples[m_vSegmentSamples.size()-1];
    VehicleState last_state_of_last_segment = last_segment.m_vStates[last_segment.m_vStates.size()-1];
    double est_time = last_state_of_last_segment.m_dTime*(1.f/rate);
    ROS_INFO("Replaying motion trajectories at %f Hz, est time %f sec.", rate, est_time);

    VehicleState origState = GetVehicleState(0);

    ros::Time t0, t1;

    double sim_dt=0.0, actual_dt=0.0, des_dt=0.0;
    uint skip=0;
    for (uint i_seg=0; i_seg<m_vSegmentSamples.size(); i_seg++)
    {
        MotionSample seg = m_vSegmentSamples[i_seg];
        assert(seg.m_vStates.size()>1);
        for (uint i_motion=0; i_motion<seg.m_vStates.size(); i_motion++)
        {
            if (skip > 0)
            {
                skip--;
                continue;
            }

            sim_dt = seg.m_vCommands[i_motion].m_dT;
            des_dt = sim_dt/rate;

            {
                t0 = ros::Time::now();
                
                VehicleState stateIn = seg.m_vStates[i_motion];
                SetVehicleState(stateIn, 0);
            
                t1 = ros::Time::now();
                actual_dt = t1.toSec() - t0.toSec();
            }

            if (des_dt > actual_dt)
            {
                ros::Duration(des_dt - actual_dt).sleep();
            }
            else
            {
                skip = round(actual_dt / des_dt);
            }

            // ROS_INFO("actual_dt %f sim_dt %f des_dt %f skip %d", actual_dt, sim_dt, des_dt, skip);
            ros::spinOnce();
        }
    }

    SetVehicleState(origState, 0);
}

VehicleState MochaPlanner::SetVehicleState(VehicleState stateIn, int worldId)
{
    carplanner_msgs::SetStateGoal goal;
    carplanner_msgs::SetStateResultConstPtr result;
    VehicleState stateOut;

    goal.world_id = worldId;
    goal.state_in = stateIn.toROS();
    m_actionSetState_client.sendGoal(goal);

    bool finished_before_timeout = m_actionSetState_client.waitForResult(ros::Duration(0.5));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_actionSetState_client.getState();
        // ROS_INFO("SetNoDelay Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            result = m_actionSetState_client.getResult();
            stateOut.fromROS(result->state_out);
            return stateOut;
        }
        else
        {
            ROS_INFO("SetState Action failed.");
            return stateOut;
        }
    }
    else
    {
        ROS_INFO("SetState Action did not finish before the time out.");
        return stateOut;
    }
}

VehicleState MochaPlanner::GetVehicleState(int worldId)
{
    carplanner_msgs::GetStateGoal goal;
    carplanner_msgs::GetStateResultConstPtr result;
    VehicleState stateOut;

    goal.world_id = worldId;
    m_actionGetState_client.sendGoal(goal);

    bool finished_before_timeout = m_actionGetState_client.waitForResult(ros::Duration(0.5));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_actionGetState_client.getState();
        // ROS_INFO("SetNoDelay Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            result = m_actionGetState_client.getResult();
            stateOut.fromROS(result->state_out);
            return stateOut;
        }
        else
        {
            ROS_INFO("GetState Action failed.");
            return stateOut;
        }
    }
    else
    {
        ROS_INFO("GetState Action did not finish before the time out.");
        return stateOut;
    }
}

///////////////////////////////////////////////////////////////////////
bool MochaPlanner::InitializeProblem(Problem& problem,
                                          const double dStartTime,
                                          const VelocityProfile* pVelProfile /* = NULL*/,
                                          CostMode eCostMode /*= eCostPoint */)
{
    ROS_INFO("Initializing problem.");
            boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);

    problem.Reset();
    problem.m_nPlanId = m_nPlanCounter++;

    //if there are previous commands, apply them so we may make a more educated guess
    MotionSample delaySample;
    // double totalDelay = GetControlDelay(0);
    // if(totalDelay > 0 && problem.m_pFunctor->GetPreviousCommand().size() != 0){
    //     CommandList::iterator it  = problem.m_pFunctor->GetPreviousCommand().begin();
    //     while(totalDelay > 0 && it != problem.m_pFunctor->GetPreviousCommand().end()){
    //         delaySample.m_vCommands.insert(delaySample.m_vCommands.begin(),(*it));
    //         //delaySample.m_vCommands.push_back((*it)  );
    //         totalDelay -= (*it).m_dT;
    //         ++it;
    //     }
    //     problem.m_pFunctor->ResetPreviousCommands();
    //     problem.m_pFunctor->SetNoDelay(true);
    //     problem.m_pFunctor->ApplyVelocities(problem.m_StartState,delaySample,0,true);
    //     //and now set the starting state to this new value
    //     problem.m_StartState = delaySample.m_vStates.back();
    // }

    //regardless of the delay, for local planning we always want to proceed with no delay and with no previous commands
    //as the previous section should take care of that
    // problem.m_pFunctor->ResetPreviousCommands();
    SetNoDelay(true);

    Sophus::SE3d dTranslation(Sophus::SO3d(),-problem.m_StartState.m_dTwv.translation());

    //first translate to base everything around dStartPose
    Sophus::SE3d dFixedStart = problem.m_StartState.m_dTwv;
    Sophus::SE3d dFixedGoal = problem.m_GoalState.m_dTwv;

    //now rotate everything to a frame inbetween the two
    Eigen::Quaternion<double> dStartQuat = problem.m_StartState.m_dTwv.so3().unit_quaternion();
    Eigen::Quaternion<double> dEndQuat = problem.m_GoalState.m_dTwv.so3().unit_quaternion();
    //find the halfway rotation
    Eigen::Quaternion<double> dMidQuat = dStartQuat.slerp(0.5,dEndQuat);
    //dMidQuat.setIdentity();

    //now rotate both waypoints into this intermediate frame
    Sophus::SE3d dRotation(Sophus::SO3d(dMidQuat.toRotationMatrix().transpose()),Eigen::Vector3d::Zero()); //we want the inverse rotation here
    static bool& bFlatten2Dcurves = CVarUtils::CreateGetCVar("debug.flatten2Dcurves",false,"");
    if(bFlatten2Dcurves){
        dRotation.so3() = Sophus::SO3d();
    }

    // ROS_INFO("sr %f %f %f %f", dStartQuat.w(), dStartQuat.x(), dStartQuat.y(), dStartQuat.z());
    // ROS_INFO("er %f %f %f %f", dEndQuat.w(), dEndQuat.x(), dEndQuat.y(), dEndQuat.z());
    // ROS_INFO("t %f %f %f", dTranslation.translation().x(), dTranslation.translation().y(), dTranslation.translation().z());
    // ROS_INFO("r %f %f %f %f", dRotation.unit_quaternion().w(), dRotation.unit_quaternion().x(), dRotation.unit_quaternion().y(), dRotation.unit_quaternion().z());
    
    problem.m_dT3d = (dRotation*dTranslation);
    problem.m_dT3dInv = problem.m_dT3d.inverse();

    //now rotate both points;
    dFixedStart = problem.m_dT3d * dFixedStart;
    dFixedGoal = problem.m_dT3d * dFixedGoal;

    VehicleState dStartStateFixed = problem.m_StartState;
    VehicleState dGoalStateFixed = problem.m_GoalState;
    dStartStateFixed.m_dTwv = dFixedStart;
    dGoalStateFixed.m_dTwv = dFixedGoal;
    //and now we can project to 2d and get our 2d planner points
    Eigen::Vector6d dStartPose2D = dStartStateFixed.ToXYZTCV();
    Eigen::Vector6d dGoalPose2D = dGoalStateFixed.ToXYZTCV();

    //make sure we don't have start and end velocities of zero
    if(dStartPose2D[5] == 0 && dGoalPose2D[5] == 0){
        return false;
    }

    problem.m_dStartTime = dStartTime;

    //setup the boundary problem
    problem.m_dStartPose = dStartPose2D;
    problem.m_dGoalPose = dGoalPose2D;
    problem.m_pBoundarySovler = &m_BoundarySolver;

    problem.m_dTinv = rpg::TInv( rpg::Cart2T( problem.m_dStartPose[0], problem.m_dStartPose[1], problem.m_dStartPose[3] ));
    Eigen::Vector6d dTemp = _TransformGoalPose(dStartPose2D,problem);
    problem.m_BoundaryProblem.m_dStartPose = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    dTemp  = _TransformGoalPose(dGoalPose2D,problem);
    problem.m_BoundaryProblem.m_dGoalPose = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    problem.m_BoundaryProblem.m_nDiscretization = 50;

    if(pVelProfile == NULL){
        problem.m_vVelProfile.clear();
        problem.m_vVelProfile.reserve(2);
        problem.m_vVelProfile.push_back(VelocityProfileNode(0, double(problem.m_dStartPose[5])));
        problem.m_vVelProfile.push_back(VelocityProfileNode(1, double(problem.m_dGoalPose[5])));
    }else{
        problem.m_vVelProfile = *pVelProfile;
    }

    //this puts the result in m_dP
    problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);
    _GetAccelerationProfile(problem);

    //initialize the max time
    problem.m_dMaxSegmentTime = problem.m_dSegmentTime*5;

    //reset optimization parameters
    problem.m_CurrentSolution.m_dNorm = DBL_MAX;
    problem.m_bInLocalMinimum = false;
    problem.m_CurrentSolution.m_dOptParams.head(3) = problem.m_BoundaryProblem.m_dGoalPose.head(3);
    if(OPT_DIM > OPT_AGGR_DIM){
        problem.m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM] = problem.m_BoundaryProblem.m_dAggressiveness;
    }
    problem.m_CurrentSolution.m_dOptParams[OPT_ACCEL_DIM] = 0.0; // this is the percentage of the calculated acceleration
    problem.m_dInitOptParams = problem.m_CurrentSolution.m_dOptParams;
    problem.m_pBestSolution = &problem.m_CurrentSolution;

    //the transformed final position
    problem.m_dTransformedGoal = _TransformGoalPose(problem.m_dGoalPose, problem);

    //restore the original start state for simulation purposes
    //problem.m_StartState = originalStartState;
    problem.m_eCostMode = eCostMode;

    return true;
}

///////////////////////////////////////////////////////////////////////
bool MochaPlanner::Iterate(Problem &problem )
{
    try
    {
                //get the current state and norm for the first iteration
        if(problem.m_lSolutions.size() == 0 ) {
            //here we have to re-evaluate the segment time of the trajectory

            SimulateTrajectory(problem.m_CurrentSolution.m_Sample,problem,0,false);
            //problem.m_dDistanceDelta = problem.m_pCurrentMotionSample->GetDistance() - problem.m_BoundaryProblem.m_dDistance;
            if(problem.m_bInertialControlActive){
                CalculateTorqueCoefficients(problem,&problem.m_CurrentSolution.m_Sample);
                SimulateTrajectory(problem.m_CurrentSolution.m_Sample,problem,0,false);
            }

            //calculate the distance
            double dMinLookahead;
            Eigen::VectorXd error = _CalculateSampleError(problem,dMinLookahead);
            //DLOG(INFO) << problem.m_nPlanId << ":Initial error is " << error.transpose();
            problem.m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,error);
            if(std::isfinite(problem.m_CurrentSolution.m_dNorm) == false){
                DLOG(INFO) << problem.m_nPlanId << ":Initial norm is not finite. Exiting optimization";
                return false;
            }
            problem.m_lSolutions.push_back(problem.m_CurrentSolution);
            problem.m_pBestSolution = &problem.m_lSolutions.back();
        }

        if( m_dEps > 5 || problem.m_bInLocalMinimum == true) {
            DLOG(INFO) << "Failed to plan. Norm = " << problem.m_CurrentSolution.m_dNorm;
            return false;
        }

        _IterateGaussNewton(problem);

        return false;
    }catch(...){
        return false;
    }
}

void MochaPlanner::CalculateTorqueCoefficients(Problem& problem,MotionSample* pSample)
{
    //find the last air section and record its duration
    double dAirTime = 0;
    int nStartIndex = -1;
    double dStartTime = 0;

    double dLongestAirTime = 0, dLongestStartTime = 0;
    int nLongestStartIndex = -1;

    //find the longest airborne segment in the trajectory
    for(int ii = std::min(pSample->m_vStates.size()-1,
                          pSample->m_vCommands.size()-1) ;
            ii>=0 ; --ii){
        if(pSample->m_vStates[ii].IsAirborne()){
            dAirTime += pSample->m_vCommands[ii].m_dT;
            nStartIndex = ii;
            dStartTime = pSample->m_vStates[ii].m_dTime;
        }else{
            if(dAirTime != 0){
                if(dAirTime > dLongestAirTime){
                    dLongestAirTime = dAirTime;
                    dLongestStartTime = dStartTime;
                    nLongestStartIndex = nStartIndex;

                    dAirTime = 0;
                    nStartIndex = -1;
                    dStartTime = 0;
                }
            }
        }
    }

    if(nLongestStartIndex != -1){
        dAirTime = dLongestAirTime;
        dStartTime = dLongestStartTime;
        nStartIndex = nLongestStartIndex;
    }

    //now calcualte the coefficients for this maneuver based on the angle error
    if(dAirTime > 0.05){
        //find the goal state based on the cost type
        Sophus::SO3d dGoalState;
        if(problem.m_eCostMode == eCostPoint){
            dGoalState = problem.m_GoalState.m_dTwv.so3();
        }else{
            dGoalState = problem.m_GoalState.m_dTwv.so3();

            bool bCurrentAirborne = problem.m_Trajectory.m_vStates[0].IsAirborne();
            for(size_t ii = 1 ; ii < problem.m_Trajectory.m_vStates.size() ; ii++){
                bool bTemp = problem.m_Trajectory.m_vStates[ii].IsAirborne();
                if(bTemp == false && bCurrentAirborne == true){
                    dGoalState = problem.m_Trajectory.m_vStates[ii].m_dTwv.so3();
                    break;
                }
                bCurrentAirborne = bTemp;
            }
        }
        //calculate the change in angles necessary
        const Sophus::SO3d Rw_dest = problem.m_GoalState.m_dTwv.so3();
        const Sophus::SO3d Rw_init = pSample->m_vStates[nStartIndex].m_dTwv.so3();

        //const Sophus::SO3d Rinit_w = Rw_init.inverse();
        //const Sophus::SO3d Rinit_dest = Rinit_w * Rw_dest;
        //angle in body frame
        const Eigen::Vector3d angles(0,
                                     rpg::AngleWrap(rpg::R2Cart(Rw_dest.matrix())[1]-rpg::R2Cart(Rw_init.matrix())[1]),// - (problem.m_eCostMode == eCostPoint ? M_PI*2 : 0),
                                     0);// = Rinit_dest.log();

        // const Eigen::Vector3d dInertia = problem.m_pFunctor->GetCarModel()->GetVehicleInertiaTensor(0);
        const Eigen::Vector3d dInertia = GetInertiaTensor(0);

        //Eigen::Vector3d currentV = pSample->m_vStates[nStartIndex].m_dV;
        const Sophus::SO3d Rwv = pSample->m_vStates[nStartIndex].m_dTwv.so3();
        //angular velocities in body frame
        //create the inertia tensor
//        Eigen::Matrix3d omega_w;
//        omega_w << 0, -pSample->m_vStates[nStartIndex].m_dW[2], pSample->m_vStates[nStartIndex].m_dW[1],
//                   pSample->m_vStates[nStartIndex].m_dW[2],0,-pSample->m_vStates[nStartIndex].m_dW[0],
//                   -pSample->m_vStates[nStartIndex].m_dW[1],pSample->m_vStates[nStartIndex].m_dW[0],0;
        //DLOG(INFO) << "omega " << omega_w;

        //Sophus::SO3d dRwv = Sophus::SO3d::hat(pSample->m_vStates[nStartIndex].m_dW);
        //const Eigen::Vector3d omega_v =  Sophus::SO3d::vee((Rwv.inverse()*dRwv).matrix());
        const Eigen::Vector3d omega_v = Rwv.inverse() * pSample->m_vStates[nStartIndex].m_dW;

        //calculate the coefficients
        Eigen::Matrix4d A;
        Eigen::Vector4d B;

        A << 1, 0              ,0                     ,0,
                1, dAirTime, powi(dAirTime,2), powi(dAirTime,3),
                dAirTime, powi(dAirTime,2)/2,powi(dAirTime,3)/3,powi(dAirTime,4)/4,
                powi(dAirTime,2)/2,  powi(dAirTime,3)/6,powi(dAirTime,4)/12,powi(dAirTime,5)/20;

            Eigen::Matrix4d Ainertia = A / dInertia(1);
            B << problem.m_dStartTorques(1),0, -omega_v(1), angles(1) - omega_v(1)*dAirTime;
            problem.m_dCoefs = Ainertia.inverse() * B;
            problem.m_dTorqueStartTime = dStartTime;
            problem.m_dTorqueDuration = dAirTime;
    }else{
        problem.m_dTorqueStartTime = -1;
    }
}

void MochaPlanner::StressTest(Problem &problem)
{
//    std::vector<std::shared_ptr<boost::thread> > threads;
//    for(int jj = 0; jj < 1000 ; jj++){
//        Eigen::Vector6d pHypeStates[DAMPING_STEPS];
//        Eigen::VectorXd pHypeErrors[DAMPING_STEPS];
//        std::vector<std::shared_ptr<Problem> > vCubicProblems;
//        std::vector<std::shared_ptr<ApplyCommandsThreadFunctor> >vFunctors;
//        vCubicProblems.resize(DAMPING_STEPS);
//        vFunctors.resize(DAMPING_STEPS);

////        vCubicProblems[0] = std::make_shared<Problem>(problem);
////        vFunctors[0] = std::make_shared<ApplyCommandsThreadFunctor>(this,*vCubicProblems[0],0,pHypeStates[0],m_DampingMotionSamples[0],true);
////        vFunctors[0]->operator()();

//        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
//            vCubicProblems[ii] = std::make_shared<Problem>(problem);
//            vCubicProblems[ii]->m_pBoundarySovler->Solve(&vCubicProblems[ii]->m_BoundaryProblem);
//            vFunctors[ii] = std::make_shared<ApplyCommandsThreadFunctor>(this,*vCubicProblems[ii],ii,pHypeStates[ii],pHypeErrors[ii],m_MotionSamples[ii],false);
//            threads.push_back(std::make_shared<boost::thread>(*vFunctors[ii].get()));
//            //vFunctors[0]->operator()();
//            //m_ThreadPool.schedule(*vFunctors[ii]);
//        }

//        for(size_t ii = 0 ; ii < threads.size() ; ii++){
//            threads[ii]->join();
//        }
//        threads.clear();
//        //m_ThreadPool.wait();
//    }
}


///////////////////////////////////////////////////////////////////////
bool MochaPlanner::_IterateGaussNewton( Problem& problem )
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd dDeltaP;
    //        DLOG(INFO) << "Entered gauss-newton search with e = " << m_dCurrentEps;
    double bestDamping;
    unsigned int nBestDampingIdx = 0;

    Eigen::VectorXd dWvec = _GetWeightVector(problem);

    Eigen::MatrixXd J = Eigen::MatrixXd(dWvec.rows(),OPT_DIM);
    //create an appropriate weighting matrix
    Eigen::MatrixXd dW = dWvec.asDiagonal();

    //double dMinLookahead;
    Eigen::VectorXd error;// = _CalculateSampleError(problem,dMinLookahead);

    //DLOG(INFO) << problem.m_nPlanId << ":Iteration error is" << error.transpose();
    Solution coordinateDescent;
    if(_CalculateJacobian(problem,error,coordinateDescent,J) == false){
        return false;
    }

    // ROS_INFO("Errors:");
    // for (uint i=0; i<error.size(); i++)
    // {
    //     ROS_INFO("%f",error[i]);
    // }

    // problem.m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,error);
    // if(g_bVerbose){
    //     DLOG(INFO) << "Calculated jacobian with base norm: " << problem.m_CurrentSolution.m_dNorm;
    // }
    // ROS_INFO("Calculated jacobian with base norm %f", problem.m_CurrentSolution.m_dNorm);

    //if any of the columns are zero, reguralize
    bool bZeroCols = false;
    for(int ii = 0 ; ii < J.cols() ; ii++){
        if( J.col(ii).norm() == 0){
            bZeroCols = true;
            break;
        }
    }

    //solve for the dDeltaP
    dDeltaP = J.transpose()*dW*error;
    Eigen::MatrixXd JtJ = J.transpose()*dW*J;
    //Do thikonov reguralization if there is a null column
    if(bZeroCols) {
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
    }
    (JtJ).llt().solveInPlace(dDeltaP);

    //this is a hack for now, but it should take care of large deltas
    if(dDeltaP.norm() > 100 ){
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
        dDeltaP = J.transpose()*dW*error;
        (JtJ).llt().solveInPlace(dDeltaP);
    }

    if(g_bVerbose){
        DLOG(INFO) << "Gauss newton delta: [" << dDeltaP.transpose().format(CleanFmt) << "]";
    }
    ROS_INFO("Gauss newton delta [%s]", convertEigenMatrix2String(dDeltaP.transpose()).c_str());


    if(std::isfinite(dDeltaP.norm()) == false){
        DLOG(ERROR) << problem.m_nPlanId << ":Deltas are NAN. Dump => J:" << J.format(CleanFmt) << std::endl << "b:" << error.transpose().format(CleanFmt) << std::endl;
        problem.m_eError = eDeltaNan;
        return false;
    }

    ROS_INFO("Finished GN with norm %f, opts [%s]", problem.m_CurrentSolution.m_dNorm, convertEigenMatrix2String(problem.m_CurrentSolution.m_dOptParams.transpose()).c_str());

    //if no damping is required, just pick the result of the gauss newton
    if(g_bDisableDamping == true){
//        problem.m_CurrentSolution.m_dOptParams.head(OPT_DIM) += dDeltaP.head(OPT_DIM);
//        problem.m_BoundaryProblem.m_dGoalPose.head(3) = problem.m_CurrentSolution.m_dOptParams.head(3);
//        problem.m_BoundaryProblem.m_dAggressiveness = problem.m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM];
        problem.UpdateOptParams(problem.m_CurrentSolution.m_dOptParams.head(OPT_DIM) + dDeltaP);
        problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);
        SimulateTrajectory( problem.m_CurrentSolution.m_Sample,problem);
        problem.m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,_CalculateSampleError(problem,problem.m_CurrentSolution.m_dMinTrajectoryTime));
        problem.m_lSolutions.push_back(problem.m_CurrentSolution);
        problem.m_pBestSolution = &problem.m_lSolutions.back();
        // DLOG(INFO) << "Iteration with no damping finished with norm " << problem.m_CurrentSolution.m_dNorm << " and opts "<< problem.m_CurrentSolution.m_dOptParams.transpose().format(CleanFmt);
        ROS_INFO("Finished GN without dampening.");
        return true;
    }

    //initialize the parameters here, in case non of the
    //dampings are any good
    double damping = 0;
    //damp the gauss newton response
    //DLOG(INFO) << "Gauss newton delta is: [" << dDeltaP.transpose() << "] - Damping with lambda";

    Eigen::Vector6d pDampingStates[DAMPING_STEPS];
    Eigen::VectorXd pDampingErrors[DAMPING_STEPS];
    //Eigen::VectorOpt pHypePs[DAMPING_STEPS];
    std::vector<std::shared_ptr<Problem> > vCubicProblems;
    std::vector<std::shared_ptr<mochapc::ApplyCommandsThreadFunctor> >vFunctors;
    vCubicProblems.resize(DAMPING_STEPS);
    vFunctors.resize(DAMPING_STEPS);
    double dampings[DAMPING_STEPS];
    damping = 1.0;

    ROS_INFO("Running ACF on %d threads to dampen current solution.", DAMPING_STEPS);

    Solution dampedSolution;
    dampedSolution.m_dNorm = DBL_MAX;
    if(std::isfinite(dDeltaP[0]) ){
        //create the damped problems and run the thread queue to sample them
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            dampings[ii] = damping;
            Eigen::VectorXd delta = dDeltaP *damping;
            vCubicProblems[ii] = std::make_shared<Problem>(problem);
            vCubicProblems[ii]->UpdateOptParams(vCubicProblems[ii]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vFunctors[ii] = std::make_shared<mochapc::ApplyCommandsThreadFunctor>(this,
                                                                         *vCubicProblems[ii],
                                                                         ii,
                                                                         pDampingStates[ii],
                                                                         pDampingErrors[ii],
                                                                         vCubicProblems[ii]->m_CurrentSolution.m_Sample,
                                                                         true);
            m_ThreadPool.schedule(*vFunctors[ii]);
            damping/= DAMPING_DIVISOR;
        }
        m_ThreadPool.wait();


        if(g_bVerbose){
            DLOG(INFO) << "Damping norms are: [";
        }
        //pick out the best damping by comparing the norms
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            double norm = 0;
            dout_cond("Final state is " << pDampingStates[ii].transpose().format(CleanFmt),g_bVerbose);
            norm = _CalculateErrorNorm(*vCubicProblems[ii],pDampingErrors[ii]);
            if(g_bVerbose){
                DLOG(INFO) << " " << norm;
            }
            ROS_INFO("Dampened solution step %d, norm %f, dampening %f, final state [%s], errors [%s]", ii, norm, dampings[ii], convertEigenMatrix2String(pDampingStates[ii].transpose()).c_str(), convertEigenMatrix2String(pDampingErrors[ii].transpose()).c_str());
            if(norm < dampedSolution.m_dNorm ) {
                ROS_INFO("New best damped solution with norm %f.", norm);
                dampedSolution = vCubicProblems[ii]->m_CurrentSolution;
                dampedSolution.m_dNorm = norm;
                bestDamping = dampings[ii];
                nBestDampingIdx = ii;
            }
        }

        if(g_bVerbose){
            DLOG(INFO) << "]" ;
        }

    }else{
        Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
        DLOG(ERROR) << problem.m_nPlanId << ":Deltas are NAN. Dump => J:" << J.format(CleanFmt) << std::endl;
        problem.m_eError = eDeltaNan;
        return false;
    }

    Solution newSolution;

    if(coordinateDescent.m_dNorm > problem.m_CurrentSolution.m_dNorm && g_bMonotonicCost){
        problem.m_bInLocalMinimum = true;
        // DLOG(INFO) << problem.m_nPlanId << ":In local minimum with Norm = " << problem.m_CurrentSolution.m_dNorm;
        ROS_INFO("Finished GN dampening for plan %d in local minimum, norm %f.", problem.m_nPlanId, problem.m_CurrentSolution.m_dNorm);
    }else if( dampedSolution.m_dNorm > problem.m_CurrentSolution.m_dNorm && g_bMonotonicCost) {
        // newSolution = coordinateDescent;
        problem.m_bInLocalMinimum = true;
        // DLOG(INFO) << problem.m_nPlanId << ":Accepted coordinate descent with Norm = " << problem.m_CurrentSolution.m_dNorm;
        ROS_INFO("Finished GN dampening for plan %d with coordinated descent solution, norm %f.", problem.m_nPlanId, newSolution.m_dNorm);
    }else{
        newSolution = dampedSolution;
        // DLOG(INFO) <<  problem.m_nPlanId << ":New norm from damped gauss newton = " << newSolution.m_dNorm << " with damping = " << bestDamping << " best damped traj error is " << pDampingErrors[nBestDampingIdx].transpose().format(CleanFmt);
        ROS_INFO("Finished GN dampening for plan %d with damped solution, norm %f, index %d", problem.m_nPlanId, newSolution.m_dNorm, nBestDampingIdx);
    }


    //update the problem params
    //problem.m_CurrentSolution.m_dOptParams = newSolution.m_dOptParams;


    //update the best solution if necessary
    if(problem.m_bInLocalMinimum == false){
        if(newSolution.m_dNorm < problem.m_CurrentSolution.m_dNorm){
            ROS_INFO("Updating best solution with norm %f.", newSolution.m_dNorm);
            //add this solution to the list
            problem.m_lSolutions.push_back(newSolution);
            problem.m_pBestSolution = &problem.m_lSolutions.back();
        }
        problem.m_CurrentSolution = newSolution;
    }
    //problem.m_CurrentSolution.m_Sample = newSolution.m_Sample;

    problem.UpdateOptParams(problem.m_CurrentSolution.m_dOptParams.head(OPT_DIM));
    problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);

//    if(problem.m_pCurrentMotionSample == NULL) {
//        assert(false);
//    }
    problem.m_eError = eSuccess;
    return true;
}

////////////////////////


// void MochaPlanner::timerCb(const ros::TimerEvent& event)
// {
//     // Using timers is the preferred 'ROS way' to manual threading
//     NODELET_INFO_STREAM("The time is now " << event.current_real);
// }

void MochaPlanner::vehicleWpLookupFunc(const ros::TimerEvent& event)
{
    if (!m_bServersInitialized) { DLOG(INFO) << "Aborting vehicle waypoint lookup bc servers not initialized."; return; }

    DLOG(INFO) << "Looking up vehicle waypoint from tf tree.";
    static tf::TransformListener tflistener;
    static tf::StampedTransform last_tf;
    tf::StampedTransform this_tf;
    try
    {
        tflistener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(1.0));
        tflistener.lookupTransform("map", "base_link", ros::Time(0), this_tf);
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.5).sleep();
        return;
    }
    
    Waypoint start_wp = Waypoint::tf2Waypoint(this_tf);
    {
        boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
        // while(m_vWaypoints.size()>2) m_vWaypoints.pop_back(); //erase(m_vWaypoints.back());
        // delete m_vWaypoints[0];
        m_vWaypoints.resize(2);
        m_vWaypoints[0] = new Waypoint(start_wp);
    }
    {
        Eigen::Vector3d dIntersect;
        Sophus::SE3d pose( m_vWaypoints[0]->state.ToSE3d() );
        if (Raycast(pose.translation(), GetBasisVector(pose,2)*0.2, dIntersect, true))
        {
            pose.translation() = dIntersect;
            m_vWaypoints[0]->state = VehicleState(pose, 1, m_vWaypoints[0]->state.m_dCurvature);
        }
    }

    m_vSegmentSamples.resize(m_vWaypoints.size()-1);
    
    if (!m_bPlanContinuously)
        replan();
}


// must use a ConstPtr callback to use zero-copy transport
void MochaPlanner::vehicleWpCb(const nav_msgs::Odometry& odom_msg)
{
    if (!m_bServersInitialized) { DLOG(INFO) << "Aborting vehicle waypoint callback bc servers not initialized."; return; }

    DLOG(INFO) << "Got vehicle waypoint"; 

    {
        boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
        m_vWaypoints.resize(2);
        SetWaypoint(0, Waypoint::OdomMsg2Waypoint(odom_msg));
    }

    m_vSegmentSamples.resize(m_vWaypoints.size()-1);

    if (!m_bPlanContinuously)
        replan();
}

///this used to work ithink
/*
// while(m_vWaypoints.size()>2) m_vWaypoints.pop_back(); //.erase(m_vWaypoints.back());
// delete m_vWaypoints[0];
m_vWaypoints.resize(2);
m_vWaypoints[0] = new Waypoint(start_wp);

{
    Eigen::Vector3d dIntersect;
    Sophus::SE3d pose( m_vWaypoints[0]->state.ToSE3d() );
    if (Raycast(pose.translation(), GetBasisVector(pose,2)*0.2, dIntersect, true))
    {
        pose.translation() = dIntersect;
        m_vWaypoints[0]->state = VehicleState(pose, 1, m_vWaypoints[0]->state.m_dCurvature);
    }
}
*/

void MochaPlanner::goalWpCb(const nav_msgs::Odometry& odom_msg)
{
    if (!m_bServersInitialized) { DLOG(INFO) << "Aborting vehicle waypoint callback bc servers not initialized."; return; }

    DLOG(INFO) << "Got goal waypoint"; 

    {
        boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
        m_vWaypoints.resize(2);
        SetWaypoint(1, Waypoint::OdomMsg2Waypoint(odom_msg));
    }

    m_vSegmentSamples.resize(m_vWaypoints.size()-1);

    if (!m_bPlanContinuously)
        replan();
}

void MochaPlanner::waypointsCb(const carplanner_msgs::OdometryArray& odom_arr_msg)
{
    if (!m_bServersInitialized) { DLOG(INFO) << "Aborting waypoints callback bc servers not initialized."; return; }

    DLOG(INFO) << "Got " <<  std::to_string(odom_arr_msg.odoms.size()) << " waypoints.";

    {
        boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
        m_vWaypoints.clear();
        for(uint i=0; i<odom_arr_msg.odoms.size(); i++)
        {
            AddWaypoint(Waypoint::OdomMsg2Waypoint(odom_arr_msg.odoms[i]));
        }
    }

    m_vSegmentSamples.resize(m_vWaypoints.size()-1);

    bestProblem.Reset();

    if (!m_bPlanContinuously)
        replan();
}

void MochaPlanner::AddWaypoint(Waypoint& wp)
{
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    Eigen::Vector3d dIntersect;

    Waypoint* wp_ptr = new Waypoint(wp); // in nwu
    // wp_ptr->state.FlipCoordFrame(); // now in ned

    if (Raycast(wp_ptr->state.m_dTwv.translation(), -GetBasisVector(wp_ptr->state.m_dTwv,2)*0.2, dIntersect, true))
    {
        wp_ptr->state.m_dTwv.translation() = dIntersect;// + Eigen::Vector3d(0,0,-0.1); // in ned

        // m_vWaypoints.back() = new Waypoint(wp);

        // m_vWaypoints.erase(m_vWaypoints.end());
        // m_vWaypoints.push_back(new Waypoint(wp));
    }

    m_vWaypoints.push_back(wp_ptr);
}

void MochaPlanner::SetWaypoint(int idx, Waypoint& wp)
{
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    Eigen::Vector3d dIntersect;

    Waypoint* wp_ptr = new Waypoint(wp);
    // wp_ptr->state.FlipCoordFrame();

    if (Raycast(wp_ptr->state.m_dTwv.translation(), -GetBasisVector(wp_ptr->state.m_dTwv,2)*0.2, dIntersect, true))
    {
        wp_ptr->state.m_dTwv.translation() = dIntersect;// + Eigen::Vector3d(0,0,-0.1);

        // m_vWaypoints.back() = new Waypoint(wp);

        // m_vWaypoints.erase(m_vWaypoints.end());
        // m_vWaypoints.push_back(new Waypoint(wp));
    }

    // if (m_vWaypoints[idx])
    // {
        // delete m_vWaypoints[idx];
    // }
    m_vWaypoints[idx] = wp_ptr;
}

// m_vWaypoints.resize(2);
// m_vWaypoints[0] = new Waypoint(start_wp);

// {
//     Eigen::Vector3d dIntersect;
//     Sophus::SE3d pose( m_vWaypoints[0]->state.ToSE3d() );
//     if (Raycast(pose.translation(), GetBasisVector(pose,2)*0.2, dIntersect, true))
//     {
//         pose.translation() = dIntersect;
//         m_vWaypoints[0]->state = VehicleState(pose, 1, m_vWaypoints[0]->state.m_dCurvature);
//     }
// }

// void MochaPlanner::waypointsCb(const carplanner_msgs::OdometryArray& odom_arr_msg)
// {
//     if (!m_bServersInitialized) { DLOG(INFO) << "Aborting waypoints callback bc servers not initialized."; return; }

//     DLOG(INFO) << "Got " <<  std::to_string(odom_arr_msg.odoms.size()) << " waypoints.";

//     Sophus::SE3d rot_180_x;
// 	rot_180_x.translation().setZero();
// 	rot_180_x.setQuaternion(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0));

//     {
//         boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
//         // m_vWaypoints.clear();
//         m_vWaypoints.resize(odom_arr_msg.odoms.size());
//         for(uint i=0; i<m_vWaypoints.size(); i++)
//         {
//             nav_msgs::Odometry odom_msg_nwu = odom_arr_msg.odoms[i];

//             // DLOG(INFO) << " odom_nwu " << i << ": "
//             //     << odom_msg_nwu.pose.pose.position.x << ", "
//             //     << odom_msg_nwu.pose.pose.position.y << ", "
//             //     << odom_msg_nwu.pose.pose.position.z << "; "
//             //     << odom_msg_nwu.pose.pose.orientation.x << ", "
//             //     << odom_msg_nwu.pose.pose.orientation.y << ", "
//             //     << odom_msg_nwu.pose.pose.orientation.z << ", "
//             //     << odom_msg_nwu.pose.pose.orientation.w << "; ";

//             Sophus::SE3d pose_nwu;
//             pose_nwu.translation()[0]                 = odom_msg_nwu.pose.pose.position.x;
//             pose_nwu.translation()[1]                 = odom_msg_nwu.pose.pose.position.y;
//             pose_nwu.translation()[2]                 = odom_msg_nwu.pose.pose.position.z;
//             pose_nwu.setQuaternion(Sophus::Quaterniond( odom_msg_nwu.pose.pose.orientation.w,
//                                                         odom_msg_nwu.pose.pose.orientation.x,
//                                                         odom_msg_nwu.pose.pose.orientation.y,
//                                                         odom_msg_nwu.pose.pose.orientation.z));
            
//             Sophus::SE3d pose_ned = rot_180_x*pose_nwu*rot_180_x;

//             nav_msgs::Odometry odom_msg_ned;
//             odom_msg_ned.pose.pose.position.x       = pose_ned.translation()[0];
//             odom_msg_ned.pose.pose.position.y       = pose_ned.translation()[1];
//             odom_msg_ned.pose.pose.position.z       = pose_ned.translation()[2];
//             odom_msg_ned.pose.pose.orientation.w    = pose_ned.unit_quaternion().w();
//             odom_msg_ned.pose.pose.orientation.x    = pose_ned.unit_quaternion().x();
//             odom_msg_ned.pose.pose.orientation.y    = pose_ned.unit_quaternion().y();
//             odom_msg_ned.pose.pose.orientation.z    = pose_ned.unit_quaternion().z();

//             // DLOG(INFO) << " odom_ned " << i << ": "
//             //     << odom_msg_ned.pose.pose.position.x << ", "
//             //     << odom_msg_ned.pose.pose.position.y << ", "
//             //     << odom_msg_ned.pose.pose.position.z << "; "
//             //     << odom_msg_ned.pose.pose.orientation.x << ", "
//             //     << odom_msg_ned.pose.pose.orientation.y << ", "
//             //     << odom_msg_ned.pose.pose.orientation.z << ", "
//             //     << odom_msg_ned.pose.pose.orientation.w << "; ";

//             Waypoint wp_ned = Waypoint::OdomMsg2Waypoint(odom_msg_ned);
//             m_vWaypoints[i] = new Waypoint(wp_ned);

//             // DLOG(INFO) << " wp " << i << ": "
//             //     << m_vWaypoints[i]->state.m_dTwv.translation()[0] << ", "
//             //     << m_vWaypoints[i]->state.m_dTwv.translation()[1] << ", "
//             //     << m_vWaypoints[i]->state.m_dTwv.translation()[2] << "; "
//             //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().x() << ", "
//             //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().y() << ", "
//             //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().z() << ", "
//             //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().w() << "; ";

//             {
//                 Eigen::Vector3d dIntersect;
//                 if (Raycast(pose_ned.translation(), GetBasisVector(pose_ned,2)*0.1, dIntersect, true))
//                 {
//                     pose_ned.translation() = dIntersect;// + Eigen::Vector3d(0, 0, 0.1); // ned
//                     m_vWaypoints[i]->state.m_dTwv = pose_ned;

//                     // DLOG(INFO) << " raycast wp " << i << ": "
//                     //     << m_vWaypoints[i]->state.m_dTwv.translation()[0] << ", "
//                     //     << m_vWaypoints[i]->state.m_dTwv.translation()[1] << ", "
//                     //     << m_vWaypoints[i]->state.m_dTwv.translation()[2] << "; "
//                     //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().x() << ", "
//                     //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().y() << ", "
//                     //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().z() << ", "
//                     //     << m_vWaypoints[i]->state.m_dTwv.unit_quaternion().w() << "; ";
//                 }
//             }
//         }
//     }

//     m_vSegmentSamples.resize(m_vWaypoints.size()-1);

//     replan();
// }

// void MochaPlanner::waypointsCb(const carplanner_msgs::OdometryArray& odom_arr_msg)
// {
//     if (!m_bServersInitialized) { DLOG(INFO) << "Aborting waypoints callback bc servers not initialized."; return; }

//     DLOG(INFO) << "Got waypoints.";

//     Sophus::SE3d rot_180_x;
// 	rot_180_x.translation().setZero();
// 	rot_180_x.setQuaternion(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0));

//     {
//         boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
//         // m_vWaypoints.clear();
//         m_vWaypoints.resize(odom_arr_msg.odoms.size());
//         for(uint i=0; i<m_vWaypoints.size(); i++)
//         {
//             Waypoint wp_in_nwu = Waypoint::OdomMsg2Waypoint(odom_arr_msg.odoms[i]);
//             Waypoint wp_in_ned = wp_in_nwu;
//             wp_in_ned.state.m_dTwv = rot_180_x*wp_in_nwu.state.m_dTwv*rot_180_x;
//             m_vWaypoints[i] = new Waypoint(wp_in_ned);
//             {
//                 Eigen::Vector3d dIntersect;
//                 Sophus::SE3d pose( wp_in_nwu.state.ToSE3d() );
//                 if (Raycast(pose.translation(), GetBasisVector(pose,2)*0.2, dIntersect, false))
//                 {
//                     pose.translation() = dIntersect;// + Eigen::Vector3d(0, 0, 0.1);
//                     m_vWaypoints[i]->state.m_dTwv = rot_180_x*pose*rot_180_x;
//                 }
//             }
//         }
//     }

//     m_vSegmentSamples.resize(m_vWaypoints.size()-1);

//     replan();
// }


// void MochaPlanner::PathCb(const nav_msgs::OdometryConstPtr odom_msg)
// {
//     Waypoint goal_wp = Waypoint::OdomMsg2Waypoint(waypoint);
//     while(m_vWaypoints.size()>2) m_vWaypoints.erase(m_vWaypoints.back());
//     m_vWaypoints[1] = &goal_pose;
//     replan();
// }

// void MochaPlanner::PublishPath()
// {
    
// }


void MochaPlanner::PubLoopFunc(const ros::TimerEvent& event)
{   
    // {
    //     boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
        _pubWaypoints(m_vWaypoints);
    // }
    _pubSimPath(m_vSegmentSamples);
    // _pubPlan(m_vSegmentSamples);
    _pubActualTraj(m_vActualTrajectory);
    _pubControlTraj(m_vControlTrajectory);
}

void MochaPlanner::PlanningLoopFunc(const ros::TimerEvent& event)
{   
    if (m_bPlanContinuously)
    {
        bool success = replan();
    }
}

bool MochaPlanner::replan()
{   
    try
    {
        if (!m_bServersInitialized) { DLOG(INFO) << "Aborting replan bc servers not initialized."; return false; }
        // if (m_bPlanning) { DLOG(INFO) << "Aborting replan bc already replanning."; return false; }

        std::vector<int> dirtyWaypointIds;

        {
            boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
            if (m_vWaypoints.empty())
            {
                DLOG(INFO) << "Aborting replan bc waypoints vector is empty.";
                return false;
            }
            
            for (uint i=0; i<m_vWaypoints.size()-1; i++) 
            {
                Eigen::Vector7d wp = m_vWaypoints[i]->state.ToXYZQuat();
                if (!std::isfinite(wp.norm()))
                {
                    DLOG(INFO) << "Aborting replan bc waypoint " << std::to_string(i) << " is not finite:"
                        << " " << std::to_string(wp[0])
                        << " " << std::to_string(wp[1])
                        << " " << std::to_string(wp[2])
                        << " " << std::to_string(wp[3])
                        << " " << std::to_string(wp[4])
                        << " " << std::to_string(wp[5])
                        << " " << std::to_string(wp[6]);
                    return false;
                }

                if (m_vWaypoints[i]->isDirty)
                    dirtyWaypointIds.push_back(i);
            }
        }
        if (dirtyWaypointIds.empty())
        {
            DLOG(INFO) << "Aborting replan bc all waypoints are clean.";
            return false;
        }

        m_vSegmentSamples.resize(m_vWaypoints.size()-1);

        DLOG(INFO) << "Replanning...";

        boost::mutex::scoped_lock planningMutex(m_mutexPlanning);
        m_bPlanning = true;

        // have dirty waypoints, need to replan the corresponding segments
        VehicleState startState;
        VehicleState goalState;
        for (uint i=0; i<dirtyWaypointIds.size(); i++) {
            uint dirtySegmentId = dirtyWaypointIds[i];
            Waypoint* a;
            Waypoint* b;
            {
                // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
                a = m_vWaypoints[dirtySegmentId];
                b = m_vWaypoints[dirtySegmentId+1];

                //make sure both waypoints have well defined poses (no nans)
                if(std::isfinite(a->state.ToXYZTCV().norm()) == false || std::isfinite(b->state.ToXYZTCV().norm()) == false )
                {
                    DLOG(INFO) << "Aborting replan bc poorly defined waypoint.";
                    return false;
                }

                // clamp waypoints to ground
                // Eigen::Vector3d dIntersect;
                // Eigen::Vector6d newPose;
                // ...

                // project waypoints to terrain
                // {
                //     Eigen::Vector3d dIntersect;
                //     Sophus::SE3d pose( a->state.ToSE3d() );
                //     if (Raycast(pose.translation(), GetBasisVector(pose,2)*0.2, dIntersect, true))
                //     {
                //         pose.translation() = dIntersect;
                //         a->state = VehicleState(pose, 1, a->state.m_dCurvature);
                //     }
                //     // *m_vWayPoints[iMotionStart] << a->GetPose(), a->GetVelocity(), a->GetAerial();
                // }
                // {
                //     Eigen::Vector3d dIntersect;
                //     Sophus::SE3d pose( b->state.ToSE3d() );
                //     if (Raycast(pose.translation(), GetBasisVector(pose,2)*0.2, dIntersect, true))
                //     {
                //         pose.translation() = dIntersect;
                //         b->state = VehicleState(pose, 1, b->state.m_dCurvature);
                //     }
                //     // *m_vWayPoints[iMotionStart] << a->GetPose(), a->GetVelocity(), a->GetAerial();
                // }

                //iterate the planner
                startState = a->state;
                goalState = b->state;
            }

    //                //do pre-emptive calculation of start/end curvatures by looking at the prev/next states
    //                GLWayPoint* pPrevWaypoint = &m_Gui.GetWaypoint((iSegment ==  0) ? m_Path[m_Path.size()-2] : m_Path[iSegment-1])->m_Waypoint;
    //                VehicleState prevState = VehicleState(Sophus::SE3d(pPrevWaypoint->GetPose4x4_po()),pPrevWaypoint->GetVelocity());
    //                GLWayPoint* pNextWaypoint = &m_Gui.GetWaypoint((iSegment >=  m_Path.size()-1) ? m_Path[0] : m_Path[iSegment+2])->m_Waypoint;
    //                VehicleState nextState = VehicleState(Sophus::SE3d(pNextWaypoint->GetPose4x4_po()),pNextWaypoint->GetVelocity());

    //                //now calculate curvatures for each side
    //                double startToGoalCurvature = rpg::AngleWrap(goalState.GetTheta()-startState.GetTheta())/(goalState.m_dTwv.translation().head(2) - startState.m_dTwv.translation().head(2)).norm() ;
    //                startState.m_dCurvature = (rpg::AngleWrap(startState.GetTheta()-prevState.GetTheta())/(startState.m_dTwv.translation().head(2) - prevState.m_dTwv.translation().head(2)).norm() +
    //                                          startToGoalCurvature)/2;
    //                goalState.m_dCurvature = (startToGoalCurvature +
    //                                         rpg::AngleWrap(nextState.GetTheta()-goalState.GetTheta())/(nextState.m_dTwv.translation().head(2) - goalState.m_dTwv.translation().head(2)).norm())/2;


    //                startState.m_dCurvature = std::isfinite(startState.m_dCurvature) ? startState.m_dCurvature : 0;
    //                goalState.m_dCurvature = std::isfinite(goalState.m_dCurvature) ? goalState.m_dCurvature : 0;

            //do connected planning if possibles
            // if(g_bContinuousPathPlanning == true){
            //     if( !(iSegment == 0 || m_vSegmentSamples[iSegment-1].m_vStates.empty() == true /*|| m_bIterateGN == false*/) ){
            //         startState = m_vSegmentSamples[iSegment-1].m_vStates.back();
            //         startState.m_dCurvature = m_vSegmentSamples[iSegment-1].m_vCommands.back().m_dCurvature;
            //     }
            // }

            // ApplyVelocitiesFunctor5d f(Eigen::Vector3d::Zero(), NULL);
            SetNoDelay(true);
            Problem problem(startState,goalState,m_dTimeInterval);
            InitializeProblem(problem,0,NULL,eCostPoint);
            problem.m_bInertialControlActive = true;//g_bInertialControl;
            //problem.m_lPreviousCommands = previousCommands;
            
            bool success = false;
            uint numIterations = 0;
            float last_norm=INFINITY, this_norm=INFINITY;
            float norm_eps = 0.1;
            while(success == false && ros::ok())
            {
                // DLOG(INFO) << "Iteration " << std::to_string(numIterations+1);

                if(numIterations+1 > g_nIterationLimit)
                {
                    // dout("Reached iteration limit. Skipping");
                    ROS_INFO("Reached iteration limit. Skipping.");
                    success = true;
                }
                else if(problem.m_CurrentSolution.m_dNorm < g_dSuccessNorm) 
                {
                    // DLOG(INFO) << problem.m_nPlanId << ":Succeeded to plan. Norm = " << problem.m_CurrentSolution.m_dNorm;
                    // ROS_DEBUG("Met success norm with plan %d, norm %f", problem.m_nPlanId, problem.m_CurrentSolution.m_dNorm);
                    ROS_INFO("Met success norm with plan %d, norm %f", problem.m_nPlanId, problem.m_CurrentSolution.m_dNorm);
                    success = true;
                }
                else if(fabs(this_norm - last_norm)<norm_eps)
                {
                    ROS_INFO("Norm not improving. Skipping.");
                    success = true;
                }
                else
                {
                    // get plan for current motion sample under consideration
                    // m_vActualTrajectory.clear();
                    // m_vControlTrajectory.clear();
                    last_norm = this_norm;
                    ROS_INFO("Iterating planner.");
                    success = _IteratePlanner(problem, m_vSegmentSamples[dirtySegmentId], m_vActualTrajectory, m_vControlTrajectory);
                    ROS_INFO("Planner %s after %dth iteration.", (success ? "succeeded" : "failed"), numIterations+1);
                    this_norm = problem.m_CurrentSolution.m_dNorm;
                }

                if (success)
                {
                    // DLOG(INFO) << "Success";
                    // ROS_INFO("Success");
                    numIterations = 0;
                }
                else
                {
                    numIterations++;
                }

                // std::string cmds_str = "Commands:\n";
                // cmds_str += "\tForce\t\tPhi\n";
                // for (uint i=0; i<m_vSegmentSamples[dirtySegmentId].m_vCommands.size(); )
                // {
                //     cmds_str += "\t" + std::to_string(m_vSegmentSamples[dirtySegmentId].m_vCommands[i].m_dForce) + "\t" + std::to_string(m_vSegmentSamples[dirtySegmentId].m_vCommands[i].m_dPhi) + "\n";
                //     i+=2;
                // }
                // printf(cmds_str.c_str());
            }

            if (problem.m_CurrentSolution.m_dNorm < bestProblem.m_CurrentSolution.m_dNorm && m_bIterateGN == true)
            {
                bestProblem = problem;
            }
            // DLOG(INFO) <<  bestProblem.m_nPlanId << " has Best norm = " << bestProblem.m_CurrentSolution.m_dNorm/* << " with damping = " << bestDamping << " best damped traj error is " << pDampingErrors[nBestDampingIdx].transpose().format(CleanFmt)*/;
            ROS_INFO("Best soln overall: id %d, norm %f", bestProblem.m_nPlanId, bestProblem.m_CurrentSolution.m_dNorm);
        }
        
        m_bPlanning = false;

        // *m_lPlanStates.front() = pPlan->m_Sample.m_vStates;

        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << "Replan failed: " << e.what() << ".\n";
        m_bPlanning = false;
        return false;
    }

}

bool MochaPlanner::_IteratePlanner(
        Problem& problem,
        //const GLWayPoint* pStart,
        MotionSample& sample, //< Output:
        Eigen::Vector3dAlignedVec& vActualTrajectory,
        Eigen::Vector3dAlignedVec& vControlTrajectory,
        bool only2d /*= false*/)
{
    static double dtcur, dtmax, t;
    t = Tic();

    bool res = false;

    //StressTest(problem);

    if(only2d == false) {
        if( m_bIterateGN == true ) // planner on && sim off
        {
            // run full Gauss-Newton minimization and calc best solution
            printf("Running full sim.\n");
            res = Iterate(problem);
            if(problem.m_bInertialControlActive){
                CalculateTorqueCoefficients(problem,&problem.m_CurrentSolution.m_Sample);
            }
            SimulateTrajectory(sample,problem,0,true); // recalc using best solution
        }
        else
        {
            // run simulation using best solution
            printf("Running simple sim with best solution.\n");
            res = true;
            SimulateTrajectory(sample,problem,0,true);
            if(problem.m_bInertialControlActive)
            {
                CalculateTorqueCoefficients(problem,&sample);
                SimulateTrajectory(sample,problem,0,true);
            }
        }

        // copy actual trajectory
        vActualTrajectory.clear();
        for(const VehicleState& state : sample.m_vStates) {
            vActualTrajectory.push_back(state.m_dTwv.translation());
        }
    }

    // copy sample trajectory
    // SamplePath(problem,vControlTrajectory);
    {
        vControlTrajectory.clear();
        vControlTrajectory.reserve(problem.m_BoundaryProblem.m_vPts.size());

        Sophus::SE2d T_start(Sophus::SO2(problem.m_dStartPose[3]),problem.m_dStartPose.head(2));

        for(const Eigen::Vector2d& pt : problem.m_BoundaryProblem.m_vPts) {
            Eigen::Vector3d dPos(pt[0],pt[1],0);
            dPos.head(2) = T_start*dPos.head(2);

            //now transform this into the proper 3d pose
            dPos = problem.m_dT3dInv * dPos;
            vControlTrajectory.push_back(dPos);
        }
    }

    dtcur = Toc(t);
    if (dtcur > dtmax) dtmax = dtcur;
    printf("Planning this time: %f s, max: %f s\n", dtcur, dtmax);

    return res;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void MochaPlanner::_pubWaypoints(std::vector<Waypoint*>& pts)
{
    if (pts.empty()) { pts.empty(); }
    // tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));

    geometry_msgs::PoseArray pts_msg;
    pts_msg.header.frame_id = "map";
    pts_msg.header.stamp = ros::Time::now();

    for( uint ii=0; ii<pts.size(); ii++ )
    {
        Eigen::MatrixXd pt;
        pt = pts[ii]->state.ToXYZRPY();
        double roll =   (pt)(3);
        double pitch =  (pt)(4);
        double yaw =    (pt)(5);
        Eigen::Quaterniond quat;
        convertRPY2Quat(Eigen::Vector3d(roll, pitch, yaw), &quat);

        tf::Transform pt_tf(tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()),
                            tf::Vector3((pt)(0),(pt)(1),(pt)(2)));
        // pt_tf = rot_180_x*pt_tf*rot_180_x;

        geometry_msgs::Pose pt_msg;
        pt_msg.position.x = pt_tf.getOrigin().x();
        pt_msg.position.y = pt_tf.getOrigin().y();
        pt_msg.position.z = pt_tf.getOrigin().z();
        pt_msg.orientation.x = pt_tf.getRotation().x();
        pt_msg.orientation.y = pt_tf.getRotation().y();
        pt_msg.orientation.z = pt_tf.getRotation().z();
        pt_msg.orientation.w = pt_tf.getRotation().w();

        pts_msg.poses.push_back(pt_msg);
    }

    m_pubWaypoints.publish(pts_msg);
    // m_tfcaster.sendTransform(state_msg.pose);
    ros::spinOnce();
}

void MochaPlanner::_pubSimPath(std::vector<MotionSample>& path_in)
{   
    if (path_in.empty()) { return; }
    std::vector<MotionSample> some_path = path_in;

    // carplanner_msgs::PathArray patharr_msg;
    // convertSomePath2PathArrayMsg(some_path, &patharr_msg, "map");
    // visualization_msgs::MarkerArray markarr_msg;
    // carplanner_msgs::MarkerArrayConfig markarr_config = carplanner_msgs::MarkerArrayConfig("",0.01,0,0,0.0,1.0,0.0,1.0);
    // convertPathArrayMsg2LineStripArrayMsg(patharr_msg,&markarr_msg, markarr_config);

    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(some_path, &path_msg, "map");
    m_pubSimPath.publish(path_msg);
    ros::spinOnce();
}

void MochaPlanner::_pubActualTraj(Eigen::Vector3dAlignedVec& path_in)
{   
    if (path_in.empty()) { return; }
    Eigen::Vector3dAlignedVec some_path = path_in;
    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(some_path, &path_msg, "map");
    // path_msg.header.frame_id = "map";
    m_pubActualTraj.publish(path_msg);
    ros::spinOnce();
}

void MochaPlanner::_pubControlTraj(Eigen::Vector3dAlignedVec& path_in)
{   
    if (path_in.empty()) { return; }
    Eigen::Vector3dAlignedVec some_path = path_in;
    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(some_path, &path_msg, "map");
    // path_msg.header.frame_id = "map";
    m_pubControlTraj.publish(path_msg);
    ros::spinOnce();
}
