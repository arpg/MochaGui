#ifndef MOCHAPLANNER_H
#define MOCHAPLANNER_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>
#include <CarPlanner/BoundarySolver.h>
#include <CarPlanner/ApplyVelocitiesFunctor.h>
#include <CarPlanner/BezierBoundarySolver.h>
#include <sophus/se3.hpp>
#include <sophus/se2.hpp>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
// #include <tf/TransformListener.h>

// #include "tf_conversion_tools.hpp"

#define XYZ_WEIGHT 2
#define THETA_WEIGHT 0.5
#define VEL_WEIGHT_TRAJ 0.5
#define VEL_WEIGHT_POINT 1.0
#define TIME_WEIGHT 0.05
#define CURV_WEIGHT 0.001
#define TILT_WEIGHT 0.5 // dRoll*dPitch
#define BADNESS_WEIGHT 5e-8;
#define DAMPING_STEPS 8
#define DAMPING_DIVISOR 1.3

#define POINT_COST_ERROR_TERMS 6
#define TRAJ_EXTRA_ERROR_TERMS 2
#define TRAJ_UNIT_ERROR_TERMS 5

#define OPT_ACCEL_DIM 3
#define OPT_AGGR_DIM 4
#define OPT_DIM 4

// other structs
struct Waypoint
{
    VehicleState state;
    bool isDirty;

    static Waypoint& OdomMsg2Waypoint(nav_msgs::OdometryConstPtr odom_msg, double steer=0, double curv=0)
    {
        Waypoint wp;
        wp.state = VehicleState::OdomMsg2VehicleState(odom_msg, steer, curv);
        wp.isDirty = true;
        return wp;
    }

    static Waypoint& tf2Waypoint(tf::StampedTransform tf, double steer=0, double curv=0)
    {
        Waypoint wp;
        wp.state = VehicleState::tf2VehicleState(tf, steer, curv);
        wp.isDirty = true;
        return wp;
    }
};

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

struct Problem
{
    void Reset()
    {
        m_dTorqueStartTime = -1;
        m_dCoefs = Eigen::Vector4d::Zero();
        m_dStartTorques = Eigen::Vector3d::Zero();
        m_bInertialControlActive = false;
        m_pBestSolution = nullptr;
        m_lSolutions.clear();;
    }

    Problem()
    {
        Reset();
    }

    Problem(ApplyVelocitesFunctor5d* m_pf, const VehicleState& startState, const VehicleState& goalState, const double& dt) :
        m_dSegmentTime(-1), 
        m_dStartTime(-1.0),
        m_dT(dt),
        m_pFunctor(m_pf)
    {
        Reset();
        m_StartState = startState;
        m_GoalState = goalState;
    }

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

    //Eigen::Vector5d m_dOptParams;               //< Optimization parameters, which are parametrized as [x,y,t,a]
    Eigen::Vector5d m_dInitOptParams;
    Eigen::Vector6d m_dTransformedGoal;
    double m_dT;                                //< The dt used in the functor to push the simulation forward

    ApplyVelocitesFunctor5d* m_pFunctor;        //< The functor which is responsible for simulating the car dynamics

    //optimization related properties
    BezierBoundaryProblem m_BoundaryProblem;           //< The boundary problem structure, describing the 2D boundary problem
    BoundarySolver* m_pBoundarySovler;          //< Pointer to the boundary value solver which will be used to solve the 2D problem
    //double m_dCurrentNorm;                      //< The current norm of the optimization problem
    //MotionSample* m_pCurrentMotionSample;       //< Pointer to the current motion sample (which represents the current 3D trajectory)
    bool m_bInLocalMinimum;                     //< Boolean which indicates if we're in a local minimum, which would indicate that the optimization is finished
    Error m_eError;

    CostMode m_eCostMode;
    MotionSample m_Trajectory;
    Eigen::Vector6dAlignedVec m_vTransformedTrajectory;
    //double m_dMinTrajectoryTime;

    //double m_dDistanceDelta;
    Eigen::Vector4d m_dCoefs;
    bool m_bInertialControlActive;
    double m_dTorqueStartTime;
    double m_dTorqueDuration;
    Eigen::Vector3d m_dStartTorques;

    void UpdateOptParams(const Eigen::VectorXd& dOptParams)
    {
        m_CurrentSolution.m_dOptParams.head(OPT_DIM) = dOptParams;
        m_BoundaryProblem.m_dGoalPose.head(3) = dOptParams.head(3);
        if(OPT_DIM > OPT_AGGR_DIM){
            //DLOG(INFO) << "Setting opt params to " << dOptParams.transpose();
            m_BoundaryProblem.m_dAggressiveness = m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM];
        }
    }

    std::list<Solution> m_lSolutions;
    Solution* m_pBestSolution;
    Solution m_CurrentSolution;
};

inline Eigen::VectorXd GetPointLineError(const Eigen::Vector6d& line1, const Eigen::Vector6d& line2, const Eigen::Vector6d& point, double &dInterpolationFactor);

class MochaPlanner
{
public:
    struct Config{
        std::string params_file, 
            terrain_mesh_file, 
            mode;
    } m_config;

    MochaPlanner(ros::NodeHandle&,ros::NodeHandle&);
    ~MochaPlanner();      

private:
    ros::NodeHandle *m_private_nh, *m_nh;
    ros::Subscriber m_subVehicleWp;
    ros::Subscriber m_subGoalWp;
    ros::Publisher m_pubPlan;
    bool m_bSubToVehicleWp;

    // virtual void onInit();

    void vehicleWpCb(const nav_msgs::OdometryConstPtr);
    void goalWpCb(const nav_msgs::OdometryConstPtr);
    // void wpPathCb(const nav_msgs::OdometryConstPtr);
    // void publishPath();

    ros::Timer m_timerWpLookup;
    float m_dWpLookupDuration;
    void vehicleWpLookupFunc(const ros::TimerEvent& event);

    // boost::thread* m_pPlannerThread;
    ros::Timer m_timerPlanningLoop;
    void PlanningLoopFunc(const ros::TimerEvent& event);
    bool replan();

    double& m_dTimeInterval;
    bool m_bPlannerOn;
    bool m_bSimulate3dPath;
    BulletCarModel m_PlanCarModel;
    // bool& m_bControl3dPath;
    bool m_bPlanning;
    std::vector<Waypoint*> m_vWaypoints;
    std::vector<MotionSample> m_vSegmentSamples;
    Eigen::Vector3dAlignedVec m_vActualTrajectory;
    Eigen::Vector3dAlignedVec m_vControlTrajectory;

public:
    /// Initializes the Problem structu which is passed in using the given parameters
    bool InitializeProblem(Problem& problem,  //< The local problem struct which will be fixed
                                const double dStartTime, //< Starting time of the problem, this is used to parametrized the generated command laws
                                const VelocityProfile* pVelProfile  = NULL,
                                CostMode eCostMode = eCostPoint);
    /// Given a Problem struct which contains a velocity profile, this function obtains the corresponding
    /// acceleration profile based
    void _GetAccelerationProfile(Problem& problem) const;
    /// Iterate the given problem using gauss-newton
    bool Iterate(Problem &problem);
    /// Samples the 2D control law that is generated by solving the boundary value problem
    void SamplePath(const Problem &problem, Eigen::Vector3dAlignedVec &vSamples, bool bBestSolution = false);
    /// Given the local problem struct, will simulate the vehicle physics and produce a motion sample
    Eigen::Vector6d SimulateTrajectory(MotionSample& sample,     //< The motion sample which will be filled by the function
                                      Problem& problem,    //< The Local Problem structure which will define the trajectory
                                      const int iWorld = 0,      //< The index of the world to run the simulation in (this is to do with the thread)
                                      const bool &bBestSolution = false);
    /// Samples the acceleration and curvature of the current control law
    void SampleAcceleration(std::vector<ControlCommand>& vCommands, Problem &problem) const;
    void StressTest(Problem &problem);
    void CalculateTorqueCoefficients(Problem &problem, MotionSample *pSample);
    /// Calculates the error for the current trajectory. The error is parametrized as [x,y,t,v]
    Eigen::VectorXd _CalculateSampleError(Problem& problem, double& dMinTrajTime) const { return _CalculateSampleError(problem.m_CurrentSolution.m_Sample,problem,dMinTrajTime); }
    Eigen::VectorXd _CalculateSampleError(const MotionSample &sample, Problem &problem, double &dMinTrajTime) const;
    Eigen::VectorXd _GetWeightVector(const Problem& problem);
    double _CalculateErrorNorm(const Problem &problem, const Eigen::VectorXd& dError);
    static int GetNumWorldsRequired(const int nOptParams) { return nOptParams*2+2;}
    
    bool _IteratePlanner(
        Problem& problem,
        //const GLWayPoint* pStart,
        MotionSample& sample, //< Output:
        Eigen::Vector3dAlignedVec& vActualTrajectory,
        Eigen::Vector3dAlignedVec& vControlTrajectory,
        bool only2d=false);

private:
    /// Calculates the jacobian of the trajectory at the current point in the trajectory
    bool _CalculateJacobian(Problem &problem,          //< The problem struct defining the current trajectory and goals
                              Eigen::VectorXd& dCurrentErrorVec,          //< This is the current error vector
                              Solution& coordinateDescent,
                              Eigen::MatrixXd &J                    //< Output: The jacobian matrix
                              );
    /// Internal function that iterates the gauss-newton optimization step
    bool _IterateGaussNewton( Problem& problem );

    /// Calculates the distance travelled, given the time passed
    double _DistanceTraveled( const double& t,const AccelerationProfile& profile ) const;

    /// Transforms the goal pose so that it is on the 2D manifold specified by the problem struct
    Eigen::Vector6d _TransformGoalPose(const Eigen::Vector6d &dGoalPose, const Problem& problem) const;
    /// Returns the trajectory error given the trajectory and a transformed trajectory and end pose
    Eigen::VectorXd _GetTrajectoryError(const MotionSample& sample, const Eigen::Vector6dAlignedVec& vTransformedPoses, const Eigen::Vector6d& endPose, double &dMinTime) const;
    /// Transforms a vehicle state so that it is on the 2D manifold specified by the problem struct
    Eigen::Vector6d _Transform3dGoalPose(const VehicleState& state, const Problem &problem) const;

    boost::threadpool::pool m_ThreadPool;                       //< Threadpool for multitasking in jacobians and damping calculation
    double& m_dEps;                                              //< The epsilon used in the calculation of the finite difference jacobian


    BezierBoundarySolver m_BoundarySolver;                      //< The boundary value problem solver

    Eigen::MatrixXd& m_dPointWeight;                                       //< The matrix which holds the weighted Gauss-Newton weights
    Eigen::MatrixXd& m_dTrajWeight;
    int m_nPlanCounter;
};

#endif