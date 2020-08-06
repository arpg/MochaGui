#ifndef MOCHACONTROLLER_H
#define MOCHACONTROLLER_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "CarPlannerCommon.h"
#include "CVarHelpers.h"
#include "MochaPlanner.h"
#include "MochaVehicle.h"


typedef std::list<ControlPlan*> PlanPtrList;

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

class MochaController
{
public:
    MochaController();
    ~MochaController()
    { 
        m_bStillRun = false;
    }

    void Init(std::vector<MotionSample> &segmentSamples, LocalPlanner *pPlanner, BulletCarModel *pModel, double dt) ;
    void Reset();

    void GetCurrentCommands(const double time, ControlCommand &command);
    VehicleState GetCurrentPose();
    void SetCurrentPoseFromCarModel(BulletCarModel* pModel, int nWorldId);
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
    static void PrepareLookaheadTrajectory(const std::vector<MotionSample>& vSegmentSamples,
                                           ControlPlan *pPlan, VelocityProfile &trajectoryProfile, MotionSample &trajectorySample, const double dLookaheadTime);

private:
    ros::NodeHandle private_nh, nh;

    ros::Subscriber m_subVehicleState;
    void vehicleStateCb(const carplanner_msgs::VehicleState::ConstPtr&);
    // VehicleState m_CurrentVehicleState;

    ros::Subscriber m_subPlan;
    void planCb(const carplanner_msgs::Plan::ConstPtr&);
    ControlPlan* m_pCurrentControlPlan;

    ros::Timer m_timerControllingLoop;
    void ControlLoopFunc();
    double m_dControlRate;

    bool _SampleControlPlan(ControlPlan* pPlan,LocalProblem& problem);
    bool _SolveControlPlan(const ControlPlan* pPlan, LocalProblem& problem, const MotionSample &trajectory);

    void _GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sampleIndex, double& interpolationAmount);



    int m_nLastCurrentPlanIndex;s
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

    float& m_dMaxControlPlanTime;
    float& m_dLookaheadTime; 

    boost::thread* m_pControlPlannerThread;

    PlanPtrList m_lControlPlans;
    std::vector<MotionSample> m_vSegmentSamples;
    CommandList m_lCurrentCommands;

    MotionSample m_MotionSample2dOnly;

    boost::mutex m_PlanMutex;
    boost::mutex m_PoseMutex;

    Eigen::Vector5d m_dLastDelta;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};


#endif

