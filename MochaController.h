#ifndef MOCHACONTROLLER_H
#define MOCHACONTROLLER_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "CarPlannerCommon.h"
// #include "CVarHelpers.h"
#include "MochaProblem.h"
// #include "MochaPlanner.h"
// #include "MochaVehicle.h"
#include <carplanner_msgs/MotionPlan.h>

// #include <dynamic_reconfigure/server.h>
// #include <carplanner_msgs/MochaConfig.h>

#define X_WEIGHT 3.0
#define Y_WEIGHT 3.0
#define Z_WEIGHT 1.5
#define THETA_WEIGHT 0.2
#define VEL_WEIGHT_TRAJ 0.5
#define VEL_WEIGHT_POINT 0.75
#define TIME_WEIGHT 0.05
#define CURV_WEIGHT 0.001
#define TILT_WEIGHT 1.5 // dRoll
#define CONTACT_WEIGHT 0.1 
// #define BADNESS_WEIGHT 5e-8;
// #define DAMPING_STEPS 7
// #define DAMPING_DIVISOR 1.3

// #define POINT_COST_ERROR_TERMS 7
// #define TRAJ_EXTRA_ERROR_TERMS 2
// #define TRAJ_UNIT_ERROR_TERMS 5

// #define OPT_ACCEL_DIM 3
// #define OPT_AGGR_DIM 4
// #define OPT_DIM 4

class ControlPlan
{
public:

    double m_dStartTime;
    double m_dEndTime;
    double m_dNorm;
    MotionSample m_Sample;

    Sophus::SE3d m_dStartPose;
    Sophus::SE3d m_dEndPose;

    int m_nStartSegmentIndex;   //the segment at which this plan starts
    int m_nStartSampleIndex; //the sample in the segment at which this control plan starts

    int m_nEndSegmentIndex;   //the segment at which this plan ends
    int m_nEndSampleIndex; //the sample in the segment at which this control plan ends
    int m_nPlanId;


    VehicleState m_StartState;
    Eigen::Vector3d m_dStartTorques;
    VehicleState m_GoalState;

    void Clear() {
        m_Sample.Clear();
    }

    ~ControlPlan(){
        //DLOG(INFO) << "Deleting control plan.";
    }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::list<ControlPlan*> PlanPtrList;

enum ControllerState{ IDLE, PLANNING };

class MochaController
{
public:
    MochaController(ros::NodeHandle&,ros::NodeHandle&);
    ~MochaController()
    { 
        m_bStillRun = false;
    }

    void InitController() ;
    void Reset();

    void GetCurrentCommands(const double time, ControlCommand &command);
    VehicleState GetCurrentPose();
    // void SetCurrentPoseFromCarModel(int nWorldId);
    void SetCurrentPose(VehicleState pose, CommandList* pCommandList = NULL);
    void GetCurrentCommands(const double time,
                            ControlCommand& command,
                            Eigen::Vector3d& targetVel,
                            Sophus::SE3d &dT_target);
    float* GetMaxControlPlanTimePtr(){ return &m_dMaxControlPlanTime; }
    float* GetLookaheadTimePtr(){ return &m_dLookaheadTime; }
    double GetLastPlanStartTime();
    bool PlanControl(double dPlanStartTime, ControlPlan*& pPlanOut);
    static double AdjustStartingSample(const std::vector<MotionSample>& segmentSamples,
                                       VehicleState& state,
                                       int& segmentIndex,
                                       int& sampleIndex,
                                       int lowerLimit = 100,
                                       int upperLimit = 100);
    void PrepareLookaheadTrajectory(const std::vector<MotionSample>& vSegmentSamples,
                                           ControlPlan *pPlan, VelocityProfile &trajectoryProfile, MotionSample &trajectorySample, const double dLookaheadTime);
    static bool ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int nWorldId=0,
                                                      bool noCompensation=false,
                                                      bool noDelay=false);

    bool g_bShow2DResult = false;
    bool g_bOptimize2DOnly = false;
    bool g_bForceZeroStartingCurvature = false;
    double g_dMinLookaheadTime = 0.05;
    double g_dMaxLookaheadTime = 2.0;
    double g_dInitialLookaheadTime = 1.0;
    double g_dMaxPlanTimeLimit = 1.0;
    double g_dLookaheadEmaWeight = 1.0;
    // static bool& g_bFreezeControl FreezeControl",false,""));
    bool g_bPointCost = true;
    bool g_bInertialControl = false;
    bool g_bInfiniteTime = false;
    bool g_bFrontFlip = false;
    double g_dMaxPlanNorm = 50;

private:
    ros::NodeHandle m_private_nh;
    ros::NodeHandle m_nh;

    ros::Subscriber m_subVehicleState;
    void vehicleStateCb(const carplanner_msgs::VehicleState::ConstPtr&);
    // VehicleState m_CurrentVehicleState;

    ros::Subscriber m_subPlan;
    void planCb(const carplanner_msgs::MotionPlan::ConstPtr&);

    ros::Publisher m_pubCommands;
    void pubCurrentCommand();
    void pubCommand(ControlCommand& );

    ros::Timer m_timerControlLoop;
    void ControlLoopFunc(const ros::TimerEvent& event);
    double m_dControlRate;

    ros::Publisher m_pubLookaheadTraj;
    void PublishLookaheadTrajectory(MotionSample&);


    // dynamic_reconfigure::Server<carplanner_msgs::MochaControllerConfig> m_dynReconfig_server;
    // void dynReconfigCb(carplanner_msgs::MochaControllerConfig &config, uint32_t level);

    std::atomic<double> m_dPlanTime;

    ControllerState m_ControllerState;

    bool _SampleControlPlan(ControlPlan* pPlan, MochaProblem& problem);
    bool _SolveControlPlan(const ControlPlan* pPlan, MochaProblem& problem, const MotionSample &trajectory);

    void _GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sampleIndex, double& interpolationAmount);

    int m_nLastCurrentPlanIndex;
    VehicleState m_CurrentState;
    // BulletCarModel* m_pModel;
    // LocalPlanner* m_pPlanner;
    ControlCommand m_LastCommand;

    double m_dStartTime;
    double m_dt;
    bool m_bStarted;
    bool m_bStopping;
    bool m_bPoseUpdated;
    bool m_bFirstPose;
    bool m_bStillRun;

    double m_dMaxControlPlanTime;
    double m_dLookaheadTime; 

    boost::thread* m_pControlPlannerThread;

    PlanPtrList m_lControlPlans;
    std::vector<MotionSample> m_vSegmentSamples;
    CommandList m_lCurrentCommands;

    MotionSample m_MotionSample2dOnly;

    boost::mutex m_PlanMutex;
    boost::mutex m_PoseMutex;
    boost::mutex m_MotionPlanMutex;

    Eigen::Vector5d m_dLastDelta;

    Eigen::MatrixXd m_dTrajWeight;

    std::string m_map_frame;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};


#endif

