#ifndef MOCHAPROBLEM_H
#define MOCHAPROBLEM_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "CarPlannerCommon.h"
// #include <CarPlanner/BulletCarModel.h>
#include "BoundarySolver.h"
#include "MochaVehicle.h"
// #include "MochaPlanner.h"
// #include "ApplyVelocitiesFunctor.h"
#include "BezierBoundarySolver.h"
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

// #include <actionlib/client/simple_action_client.h>
// #include <carplanner_msgs/GetControlDelayAction.h>
// #include <carplanner_msgs/GetInertiaTensorAction.h>
// #include <carplanner_msgs/SetNoDelayAction.h>
#include <carplanner_msgs/GetStateAction.h>
#include <carplanner_msgs/SetStateAction.h>

#include <carplanner_msgs/EnableTerrainPlanning.h>
#include <carplanner_msgs/EnableContinuousPlanning.h>
#include <carplanner_msgs/Replay.h>

#include <carplanner_msgs/OdometryArray.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <carplanner_msgs/mocha_conversions.hpp>
#include <carplanner_msgs/io_conversion_tools.hpp>


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

#define DAMPING_STEPS 7
#define DAMPING_DIVISOR 1.3

#define POINT_COST_ERROR_TERMS 8
#define TRAJ_EXTRA_ERROR_TERMS 2
#define TRAJ_UNIT_ERROR_TERMS 5

#define OPT_ACCEL_DIM 3
#define OPT_AGGR_DIM 4
#define OPT_DIM 4

// assert(DAMPING_STEPS <= OPT_DIM*2+1);

enum Error
{
    eSuccess,
    eJacobianColumnNan,
    eJacobianColumnZero,
    eDeltaNan,
};

struct VelocityProfileNode
{
    VelocityProfileNode(const double& dDistanceRatio, const double& dVel) : m_dDistanceRatio(dDistanceRatio),m_dVel(dVel){}
    double m_dDistanceRatio;
    double m_dVel;
};

struct AccelerationProfileNode
{
    AccelerationProfileNode(const double& dEndTime, const double& dAccel, const double& dEndDist,
                            const double& dVStart, const double& dVEnd) :
        m_dEndTime(dEndTime),
        m_dAccel(dAccel),
        m_dEndDist(dEndDist),
        m_dVStart(dVStart),
        m_dVEnd(dVEnd){}
    double m_dEndTime;
    double m_dAccel;
    double m_dEndDist;
    double m_dVStart;
    double m_dVEnd;
};

typedef std::vector<VelocityProfileNode > VelocityProfile;
typedef std::vector<AccelerationProfileNode > AccelerationProfile;

struct Solution
{
    Solution() {}
    Solution(const MotionSample& sample,
            const Eigen::Vector5d& dOptParams,
            const double dMinTrajectoryTime,
            const double dNorm) :
        m_dOptParams(dOptParams),
        m_Sample(sample),
        m_dMinTrajectoryTime(dMinTrajectoryTime),
        m_dNorm(dNorm)
    {

    }
    //BezierBoundaryProblem m_Solution;
    Eigen::Vector5d m_dOptParams; // x, y, theta, acceleration, aggressiveness (OPT_DIM is used to selectively enable aggressiveness, currently set to 4)
    MotionSample m_Sample;
    double m_dMinTrajectoryTime;
    double m_dNorm;
};

enum CostMode
{
    eCostPoint, // error terms: x y z yaw pitch (sometimes curvature, aggressiveness, or 'badness') between goal and pose
    eCostTrajectory
};

inline Eigen::VectorXd GetPointLineError(const Eigen::Vector6d& line1, const Eigen::Vector6d& line2, const Eigen::Vector6d& point, double &dInterpolationFactor);

// struct ProblemSettings
// {
//     CostMode cost_mode;
//     double x_pt_weight, 
//            y_pt_weight, 
//            z_pt_weight, 
//            theta_pt_weight, 
//            speed_pt_weight, 
//            roll_vel_traj_weight, 
//            pitch_vel_traj_weight, 
//            yaw_vel_traj_weight, 
//            contact_traj_weight, 
//            collision_traj_weight, 
//            time_pt_weight;
    
//     bool enable_damping;
//     int num_damping_steps;
//     double damping_divisor;
// };  

class MochaProblem
{

    void Reset();

public:
    MochaProblem(std::string name);
    MochaProblem(std::string name, const VehicleState& startState, const VehicleState& goalState, const double& dt);
    ~MochaProblem();

    MochaProblem(const MochaProblem& problem);

    int m_nPlanId;

    VehicleState m_StartState;
    VehicleState m_GoalState;

    Eigen::Vector6d m_dStartPose;   //< Starting 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature,v]'
    Eigen::Vector6d m_dGoalPose;    //< Goal 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature,v]'
    Sophus::SO3d m_dTinv;
    Sophus::SE3d m_dT3dInv;
    Sophus::SE3d m_dT3d;

    //Eigen::VectorCubic2D m_dCubic;
    double m_dSegmentTime;
    double m_dMaxSegmentTime;                   //< This is to stop runaway simulation in case the gauss newton delta destroys the solution

    VelocityProfile m_vVelProfile;              //< Velocity profile for this trajectory
    AccelerationProfile m_vAccelProfile;        //< Acceleration profile for this trajectory

    double m_dStartTime;

    Eigen::Vector5d m_dOptParams;               //< Optimization parameters, which are parametrized as [x,y,theta,accel,aggr]
    Eigen::Vector5d m_dInitOptParams;
    Eigen::Vector6d m_dTransformedGoal;
    double m_dT;                                //< The dt used in the functor to push the simulation forward

    // ApplyVelocitiesFunctor5d* m_pFunctor;        //< The functor which is responsible for simulating the car dynamics

    //optimization related properties
    BezierBoundaryProblem m_BoundaryProblem;           //< The boundary problem structure, describing the 2D boundary problem
    BoundarySolver* m_pBoundarySovler;          //< Pointer to the boundary value solver which will be used to solve the 2D problem
    //double m_dCurrentNorm;                      //< The current norm of the optimization problem
    //MotionSample* m_pCurrentMotionSample;       //< Pointer to the current motion sample (which represents the current 3D trajectory)
    bool m_bInLocalMinimum;                     //< Boolean which indicates if we're in a local minimum, which would indicate that the optimization is finished
    Error m_eError;

    CostMode m_eCostMode;
    MotionSample m_Trajectory;
    // Eigen::Vector6dAlignedVec m_vTransformedTrajectory;
    //double m_dMinTrajectoryTime;

    //double m_dDistanceDelta;
    Eigen::Vector4d m_dCoefs;
    bool m_bInertialControlActive;
    double m_dTorqueStartTime;
    double m_dTorqueDuration;
    Eigen::Vector3d m_dStartTorques;

    void UpdateOptParams(const Eigen::VectorXd& dOptParams)
    {
        ROS_DEBUG("Updating opt params.");
        m_CurrentSolution.m_dOptParams.head(OPT_DIM) = dOptParams; // x, y, theta, acceleration, aggressiveness
        m_BoundaryProblem.m_dGoalPose.head(3) = dOptParams.head(3);
        if(OPT_DIM > OPT_AGGR_DIM){
            //DLOG(INFO) << "Setting opt params to " << dOptParams.transpose();
            m_BoundaryProblem.m_dAggressiveness = m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM];
        }
    }

    std::list<Solution> m_lSolutions;
    Solution* m_pBestSolution;
    Eigen::VectorXd m_vBestSolutionErrors;
    Solution m_CurrentSolution;  

    std::string m_problemName;

    bool ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int nWorldId=0,
                                                      bool noCompensation=false,
                                                      bool noDelay=false);

    
    bool g_bUseCentralDifferences = true;
    double g_dTimeTarget = 0.00;
    // // bool g_bUseGoalPoseStepping = bug.UseGoalPoseStepping",false);
    bool g_bDisableDamping = false;
    bool g_bMonotonicCost = true;
    bool g_bVerbose = false;
    // bool g_bTrajectoryCost = true;
    int g_nTrajectoryCostSegments = 10;
    // int g_nIterationLimit = 10;
    // // double g_dSuccessNorm = 2.0;
    bool bFlatten2Dcurves = false;

private:

    bool m_bServersInitialized = false;

    // void ApplyVelocitiesDoneCb(const actionlib::SimpleClientGoalState&, const carplanner_msgs::ApplyVelocitiesResultConstPtr&);
    ApplyVelocitiesClients m_clientsApplyVelocities;

    // double GetControlDelay(int worldId);
    // actionlib::SimpleActionClient<carplanner_msgs::GetControlDelayAction>* m_actionGetControlDelay_client;

    // const Eigen::Vector3d GetInertiaTensor(int worldId);
    // actionlib::SimpleActionClient<carplanner_msgs::GetInertiaTensorAction> m_actionGetInertiaTensor_client;

    // void SetNoDelay(bool);    
    // actionlib::SimpleActionClient<carplanner_msgs::SetNoDelayAction> m_actionSetNoDelay_client;

    // VehicleState SetVehicleState(VehicleState stateIn, int worldId=0);
    // actionlib::SimpleActionClient<carplanner_msgs::SetStateAction> m_actionSetState_client;

    // VehicleState GetVehicleState(int worldId=0);
    // actionlib::SimpleActionClient<carplanner_msgs::GetStateAction> m_actionGetState_client;

    // virtual void onInit();

    bool m_bIterateGN;
    bool m_bPlanContinuously;
    bool m_bSimulate3dPath;
    // MochaVehicle m_PlanCarModel;
    // bool& m_bControl3dPath;
    bool m_bPlanning;
    boost::mutex m_mutexPlanning;
    boost::mutex m_mutexWaypoints;
    std::vector<MotionSample> m_vSegmentSamples;
    Eigen::Vector3dAlignedVec m_vActualTrajectory;
    Eigen::Vector3dAlignedVec m_vControlTrajectory;

    // boost::mutex m_mutexApplyVelocitiesInfo;
    // std::vector<actionlib::GoalID> m_vApplyVelocitiesGoalIds;
    // std::vector<MotionSample*> m_vApplyVelocitiesMotionSamples;
    
    // ros::Publisher m_pubPotentialPathsViz;
    std::vector<MotionSample> m_vPotentialPaths;
    // void pubPotentialPathsViz();

public:

    void GetPotentialPaths(std::vector<MotionSample>&);
    
    /// Initializes the MochaProblem structu which is passed in using the given parameters
    bool Initialize(const double dStartTime, //< Starting time of the problem, this is used to parametrized the generated command laws
                                const VelocityProfile* pVelProfile  = NULL,
                                CostMode eCostMode = eCostPoint);
    /// Given a MochaProblem struct which contains a velocity profile, this function obtains the corresponding
    /// acceleration profile based
    void _GetAccelerationProfile() ;
    /// Iterate the given problem using gauss-newton
    bool Iterate();
    /// Samples the 2D control law that is generated by solving the boundary value problem
    void SamplePath(Eigen::Vector3dAlignedVec &vSamples, bool bBestSolution=true);
    /// Given the local problem struct, will simulate the vehicle physics and produce a motion sample
    Eigen::Vector6d SimulateTrajectory(MotionSample& sample,     //< The motion sample which will be filled by the function
                                      const int iWorld = 0,      //< The index of the world to run the simulation in (this is to do with the thread)
                                       const bool& bBestSolution=false);
    /// Samples the acceleration and curvature of the current control law
    void SampleAcceleration(std::vector<ControlCommand>& vCommands) ;
    /// Calculates the error for the current trajectory. The error is parametrized as [x,y,t,v]
    Eigen::VectorXd _CalculateSampleError()  { return _CalculateSampleError(m_CurrentSolution.m_Sample, m_CurrentSolution.m_dMinTrajectoryTime); }
    Eigen::VectorXd _CalculateSampleError(const MotionSample &sample)  { return _CalculateSampleError(sample, m_CurrentSolution.m_dMinTrajectoryTime); }
    Eigen::VectorXd _CalculateSampleError(double &dMinTrajTime) const { return _CalculateSampleError(m_CurrentSolution.m_Sample, dMinTrajTime); }
    Eigen::VectorXd _CalculateSampleError(const MotionSample &sample, double &dMinTrajTime) const;
    Eigen::VectorXd CalculatePointError(const MotionSample &sample) const;
    Eigen::VectorXd CalculateTrajectoryError(const MotionSample &sample, double &dMinTrajTime) const;
    Eigen::VectorXd _GetWeightVector();
    double _CalculateErrorNorm(const Eigen::VectorXd& dError);
    static int GetNumWorldsRequired(const int nOptParams=OPT_DIM) { return nOptParams*2+2;}
    void CalculateTorqueCoefficients(MotionSample& pSample);
    
    ros::NodeHandle* m_nh;

    void SetStartState(VehicleState);
    void SetGoalState(VehicleState);
    void SetTimeInterval(double);
    void SetParameterEpsilon(double);
    void SetPointWeights(Eigen::VectorXd weights);
    void SetTrajWeights(Eigen::VectorXd weights);

private:
    /// Calculates the jacobian of the trajectory at the current point in the trajectory
    bool _CalculateJacobian(Eigen::VectorXd& dCurrentErrorVec,          //< This is the current error vector
                              Solution& coordinateDescent,
                              Eigen::MatrixXd &J                    //< Output: The jacobian matrix
                              );
    /// Internal function that iterates the gauss-newton optimization step
    bool _IterateGaussNewton( );

    /// Calculates the distance travelled, given the time passed
    double _DistanceTraveled( const double& t, const AccelerationProfile& profile ) const;

    /// Transforms the goal pose so that it is on the 2D manifold specified by the problem struct
    Eigen::Vector6d _TransformGoalPose(const Eigen::Vector6d &dGoalPose) const;
    /// Returns the trajectory error given the trajectory and a transformed trajectory and end pose
    Eigen::VectorXd _GetTrajectoryError(const MotionSample& sample, const Eigen::Vector6dAlignedVec& vTransformedPoses, const Eigen::Vector6d& endPose, double &dMinTime) const;
    /// Transforms a vehicle state so that it is on the 2D manifold specified by the problem struct
    Eigen::Vector6d _Transform3dGoalPose(const VehicleState& state) const;

    boost::threadpool::pool m_ThreadPool;                       //< Threadpool for multitasking in jacobians and damping calculation
    double m_dEps;                                              //< The epsilon used in the calculation of the finite difference jacobian


    BezierBoundarySolver m_BoundarySolver;                      //< The boundary value problem solver

    Eigen::MatrixXd m_dPointWeight;                                       //< The matrix which holds the weighted Gauss-Newton weights
    Eigen::MatrixXd m_dTrajWeight;
    int m_nPlanCounter;

    bool m_bNoDelay;
};

#endif