#ifndef MOCHAPLANNER_H
#define MOCHAPLANNER_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "ros/package.h"

#include "CarPlannerCommon.h"
// #include <CarPlanner/BulletCarModel.h>
#include "BoundarySolver.h"
// #include "MochaVehicle.h"
#include "MochaProblem.h"
// #include "ApplyVelocitiesFunctor.h"
#include "BezierBoundarySolver.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include <sophus/se3.hpp>
#include <sophus/se2.hpp>
// #include "CVarHelpers.h"
// #include "cvars/CVar.h"

#include <nodelet/nodelet.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <tf/TransformListener.h>

// #include <std_srvs/Bool.h>
// #include <std_srvs/Empty.h>

// #include "tf_conversion_tools.hpp"

#include <actionlib/client/simple_action_client.h>
// #include <carplanner_msgs/GetControlDelayAction.h>
// #include <carplanner_msgs/GetInertiaTensorAction.h>
// #include <carplanner_msgs/SetNoDelayAction.h>
#include <carplanner_msgs/GetStateAction.h>
#include <carplanner_msgs/SetStateAction.h>

#include <carplanner_msgs/EnableTerrainPlanning.h>
#include <carplanner_msgs/EnableContinuousPlanning.h>
#include <carplanner_msgs/Replay.h>

#include <carplanner_msgs/MotionPlan.h>

#include <carplanner_msgs/OdometryArray.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <carplanner_msgs/mocha_conversions.hpp>
#include <carplanner_msgs/io_conversion_tools.hpp>

#include <dynamic_reconfigure/server.h>
#include <carplanner_msgs/MochaPlannerConfig.h>

// #define X_WEIGHT 3.0
// #define Y_WEIGHT 3.0
// #define Z_WEIGHT 1.5
// #define THETA_WEIGHT 0.2
// #define VEL_WEIGHT_TRAJ 0.5
// #define VEL_WEIGHT_POINT 0.75
// #define TIME_WEIGHT 0.05
// #define CURV_WEIGHT 0.001
// #define TILT_WEIGHT 1.5 // dRoll
// #define CONTACT_WEIGHT 0.1 
// #define BADNESS_WEIGHT 5e-8;
// #define DAMPING_STEPS 7
// #define DAMPING_DIVISOR 1.3

// #define POINT_COST_ERROR_TERMS 7
// #define TRAJ_EXTRA_ERROR_TERMS 2
// #define TRAJ_UNIT_ERROR_TERMS 5

// #define OPT_ACCEL_DIM 3
// #define OPT_AGGR_DIM 4
// #define OPT_DIM 4

// assert(DAMPING_STEPS <= OPT_DIM*2+1);

// other structs
struct Waypoint
{
    VehicleState state;
    bool isDirty;

    Waypoint() { 
        isDirty = true;
    }

    Waypoint(VehicleState& state_) { 
        state = state_; 
        isDirty = true;
    }

    inline static Waypoint& OdomMsg2Waypoint(const nav_msgs::Odometry& odom_msg, double steer=0, double curv=0)
    {
        Waypoint* wp = new Waypoint();
        wp->state = VehicleState::OdomMsg2VehicleState(odom_msg, steer, curv);
        // wp.state.fromOdomMsg(odom_msg, steer, curv);
        wp->isDirty = true;
        return (*wp);
    }

    inline static Waypoint tf2Waypoint(tf::StampedTransform tf, double steer=0, double curv=0)
    {
        Waypoint wp;
        wp.state = VehicleState::tf2VehicleState(tf, steer, curv);
        wp.isDirty = true;
        return wp;
    }
};

// struct PlannerSettings
// {
//     ProblemSettings problem_settings;
//     double eps; // perturbation to params during Gauss Newton
//     double dt; // time interval of simulation
//     double success_norm; // norm under which a plan is successful and planning ends, INF=disable
//     double improvement_norm; // delta norm under which a plan is not improving and is thus aborted, INF=disable
//     int iteration_limit; // number of Gauss Newton iterations after which a plan fails and is aborted, INF=disable
// };  

class MochaPlanner
{
public:
    struct Config{
        std::string params_file=ros::package::getPath("mochapc") + "/learning_params.csv", 
            terrain_mesh_file=ros::package::getPath("mochapc") + "/labLoop.ply",
            map_frame="map",
            base_link_frame="base_link";
        enum Mode{ Simulation=0, Experiment=1 } mode=Mode::Simulation;
    } m_config;

    MochaPlanner(ros::NodeHandle&,ros::NodeHandle&);
    ~MochaPlanner(); 
  

private:
    ros::NodeHandle m_private_nh, m_nh;
    ros::Subscriber m_subVehicleWp;
    ros::Subscriber m_subGoalWp;
    ros::Subscriber m_subWaypoints;
    ros::Publisher m_pubPlan;
    ros::Publisher m_pubWaypoints;
    ros::Publisher m_pubSimPath;
    ros::Publisher m_pubActualTraj;
    ros::Publisher m_pubControlTraj;
    bool m_bSubToVehicleWp;

    void Init();
    void Initialize();
    void InitializeParameters();
    void InitializeExternals();

    bool m_bServersInitialized = false;

    // bool ApplyVelocities(const VehicleState& startState,
    //                                                   MotionSample& sample,
    //                                                   int nWorldId=0,
    //                                                   bool noCompensation=false);
    // actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_client;
    // void ApplyVelocitiesDoneCb(const actionlib::SimpleClientGoalState&, const carplanner_msgs::ApplyVelocitiesResultConstPtr&);

    // double GetControlDelay(int worldId);
    // actionlib::SimpleActionClient<carplanner_msgs::GetControlDelayAction>* m_actionGetControlDelay_client;

    // const Eigen::Vector3d GetInertiaTensor(int worldId);
    // actionlib::SimpleActionClient<carplanner_msgs::GetInertiaTensorAction> m_actionGetInertiaTensor_client;

    // void SetNoDelay(bool);    
    // actionlib::SimpleActionClient<carplanner_msgs::SetNoDelayAction> m_actionSetNoDelay_client;

    inline void EnableTerrainPlanning(bool do_plan) { m_bIterateGN = do_plan; }
    bool EnableTerrainPlanningSvcCb(carplanner_msgs::EnableTerrainPlanning::Request &req, carplanner_msgs::EnableTerrainPlanning::Response &res);
    ros::ServiceServer m_srvEnableTerrainPlanning_server;

    inline void EnableContinuousPlanning(bool do_plan_continuously) { m_bPlanContinuously = do_plan_continuously; }
    bool EnableContinuousPlanningSvcCb(carplanner_msgs::EnableContinuousPlanning::Request &req, carplanner_msgs::EnableContinuousPlanning::Response &res);
    ros::ServiceServer m_srvEnableContinuousPlanning_server;

    bool Raycast(const Eigen::Vector3d& dSource, const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool& biDirectional, int index=0);
    actionlib::SimpleActionClient<carplanner_msgs::RaycastAction> m_actionRaycast_client;

    bool ReplaySvcCb(carplanner_msgs::Replay::Request &req, carplanner_msgs::Replay::Response &res);
    ros::ServiceServer m_srvReplay_server;
    void replay(float rate=1.f);

    bool SetVehicleState(VehicleState stateIn, int worldId=0);
    actionlib::SimpleActionClient<carplanner_msgs::SetStateAction> m_actionSetState_client;

    bool GetVehicleState(VehicleState&, int worldId=0);
    actionlib::SimpleActionClient<carplanner_msgs::GetStateAction> m_actionGetState_client;

    // virtual void onInit();

    void vehicleWpCb(const nav_msgs::Odometry&);
    void goalWpCb(const nav_msgs::Odometry&);
    // void wpPathCb(const nav_msgs::OdometryConstPtr);
    // void publishPath();

    void WaitForPlannerVehicles();

    void waypointsCb(const carplanner_msgs::OdometryArray&);

    dynamic_reconfigure::Server<carplanner_msgs::MochaPlannerConfig> m_dynReconfig_server;
    void dynReconfigCb(carplanner_msgs::MochaPlannerConfig &config, uint32_t level);

    ros::Timer m_timerPubLoop;
    void PubLoopFunc(const ros::TimerEvent& event);

    void _pubPlan(std::vector<MotionSample>& );

    void _pubWaypoints(std::vector<Waypoint*>& );
    void _pubSimPath(std::vector<MotionSample>& );
    void _pubActualTraj(Eigen::Vector3dAlignedVec& );
    void _pubControlTraj(Eigen::Vector3dAlignedVec& );

    ros::Timer m_timerWpLookup;
    float m_dWpLookupDuration;
    void vehicleWpLookupFunc(const ros::TimerEvent& event);

    // boost::thread* m_pPlannerThread;
    ros::Timer m_timerPlanningLoop;
    void PlanningLoopFunc(const ros::TimerEvent& event);
    bool replan();

    double m_dTimeInterval;
    bool m_bIterateGN;
    bool m_bPlanContinuously;
    bool m_bSimulate3dPath;
    // MochaVehicle m_PlanCarModel;
    // bool& m_bControl3dPath;
    bool m_bPlanning;
    boost::mutex m_mutexPlanning;
    boost::mutex m_mutexWaypoints;
    std::vector<Waypoint*> m_vWaypoints;
    std::vector<MotionSample> m_vSegmentSamples;
    Eigen::Vector3dAlignedVec m_vActualTrajectory;
    Eigen::Vector3dAlignedVec m_vControlTrajectory;

    std::vector<MotionSample> m_vPotentialPaths;
    ros::Publisher m_pubPotentialPaths;
    std::vector<double> m_vPotentialNorms;
    ros::Publisher m_pubPotentialNorms;
    void pubPotentialPathsViz();
    void clearPotentialPathsViz();

    // boost::mutex m_mutexApplyVelocitiesInfo;
    // std::vector<actionlib::GoalID> m_vApplyVelocitiesGoalIds;
    // std::vector<MotionSample*> m_vApplyVelocitiesMotionSamples;
    
    bool g_bUseCentralDifferences = true;
    double g_dTimeTarget = 0.0;
    //bool& g_bUseGoalPoseStepping = CVarUtils::CreateGetCVar("debug.UseGoalPoseStepping",false);
    bool g_bDisableDamping = false;
    bool g_bMonotonicCost = true;
    bool g_bVerbose = false;
    bool g_bTrajectoryCost = true;
    int g_nTrajectoryCostSegments = 10;
    int g_nIterationLimit = 10;
    double g_dSuccessNorm = 50;
    double g_dImprovementNorm = 0.1;

public:
    /// Initializes the Problem structu which is passed in using the given parameters
    // bool InitializeProblem(Problem& problem,  //< The local problem struct which will be fixed
    //                             const double dStartTime, //< Starting time of the problem, this is used to parametrized the generated command laws
    //                             const VelocityProfile* pVelProfile  = NULL,
    //                             CostMode eCostMode = eCostPoint);
    /// Given a Problem struct which contains a velocity profile, this function obtains the corresponding
    /// acceleration profile based
    // void _GetAccelerationProfile(Problem& problem) const;
    /// Iterate the given problem using gauss-newton
    // bool Iterate(Problem &problem);
    /// Samples the 2D control law that is generated by solving the boundary value problem
    // void SamplePath(const Problem &problem, Eigen::Vector3dAlignedVec &vSamples, bool bBestSolution = false);
    /// Given the local problem struct, will simulate the vehicle physics and produce a motion sample
    // Eigen::Vector6d SimulateTrajectory(MotionSample& sample,     //< The motion sample which will be filled by the function
    //                                   Problem& problem,    //< The Local Problem structure which will define the trajectory
    //                                   const int iWorld = 0,      //< The index of the world to run the simulation in (this is to do with the thread)
    //                                   const bool &bBestSolution = false);
    /// Samples the acceleration and curvature of the current control law
    // void SampleAcceleration(std::vector<ControlCommand>& vCommands, Problem &problem) const;
    // void StressTest(Problem &problem);
    // void CalculateTorqueCoefficients(Problem &problem, MotionSample *pSample);
    /// Calculates the error for the current trajectory. The error is parametrized as [x,y,t,v]
    // Eigen::VectorXd _CalculateSampleError(Problem& problem, double& dMinTrajTime) const { return _CalculateSampleError(problem.m_CurrentSolution.m_Sample,problem,dMinTrajTime); }
    // Eigen::VectorXd _CalculateSampleError(const MotionSample &sample, Problem &problem, double &dMinTrajTime) const;
    // Eigen::VectorXd _GetWeightVector(const Problem& problem);
    // double _CalculateErrorNorm(const Problem &problem, const Eigen::VectorXd& dError);
    // static int GetNumWorldsRequired(const int nOptParams) { return nOptParams*2+1;}
    
    bool _IteratePlanner(
        MochaProblem& problem,
        //const GLWayPoint* pStart,
        MotionSample& sample, //< Output:
        Eigen::Vector3dAlignedVec& vActualTrajectory,
        Eigen::Vector3dAlignedVec& vControlTrajectory,
        bool only2d=false);

    void SetWaypoint(int idx, Waypoint& wp);
    void AddWaypoint(Waypoint& wp);

private:
    /// Calculates the jacobian of the trajectory at the current point in the trajectory
    // bool _CalculateJacobian(Problem &problem,          //< The problem struct defining the current trajectory and goals
    //                           Eigen::VectorXd& dCurrentErrorVec,          //< This is the current error vector
    //                           Solution& coordinateDescent,
    //                           Eigen::MatrixXd &J                    //< Output: The jacobian matrix
    //                           );
    /// Internal function that iterates the gauss-newton optimization step
    // bool _IterateGaussNewton( Problem& problem );

    /// Calculates the distance travelled, given the time passed
    // double _DistanceTraveled( const double& t,const AccelerationProfile& profile ) const;

    /// Transforms the goal pose so that it is on the 2D manifold specified by the problem struct
    // Eigen::Vector6d _TransformGoalPose(const Eigen::Vector6d &dGoalPose, const Problem& problem) const;
    /// Returns the trajectory error given the trajectory and a transformed trajectory and end pose
    // Eigen::VectorXd _GetTrajectoryError(const MotionSample& sample, const Eigen::Vector6dAlignedVec& vTransformedPoses, const Eigen::Vector6d& endPose, double &dMinTime) const;
    /// Transforms a vehicle state so that it is on the 2D manifold specified by the problem struct
    // Eigen::Vector6d _Transform3dGoalPose(const VehicleState& state, const Problem &problem) const;

    // boost::threadpool::pool m_ThreadPool;                       //< Threadpool for multitasking in jacobians and damping calculation
    double m_dEps;                                              //< The epsilon used in the calculation of the finite difference jacobian


    // BezierBoundarySolver m_BoundarySolver;                      //< The boundary value problem solver

    Eigen::MatrixXd m_dPointWeight;                                       //< The matrix which holds the weighted Gauss-Newton weights
    Eigen::MatrixXd m_dTrajWeight;
    int m_nPlanCounter;
};

#endif