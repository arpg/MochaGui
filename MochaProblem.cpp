#include "MochaProblem.h"

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

struct ApplyCommandsThreadFunctor {
    ApplyCommandsThreadFunctor(std::shared_ptr<MochaProblem> problem,const int index , Eigen::Vector6d& poseOut,Eigen::VectorXd& errorOut,
                               MotionSample& sample, const bool bSolveBoundary = false) :
        m_Problem(problem),
        m_index(index),
        m_poseOut(poseOut),
        m_dErrorOut(errorOut),
        m_Sample(sample),
        m_bSolveBoundary(bSolveBoundary)
    {
        // ROS_INFO("Constructing ApplyCommandsFunctor for %s idx %d", m_Problem.m_problemName.c_str(), m_index);
    }

    void operator()()
    {
        // ROS_INFO("Running ApplyCommandsFunctor for %s idx %d", m_Problem->m_problemName.c_str(), m_index);
        SetThreadName((boost::format("Bullet Simulation Thread #%d") % m_index).str().c_str());

        double t0 = Tic();
        if(m_bSolveBoundary){
            // ROS_INFO("Solving BVP for %s", m_Problem->m_problemName.c_str());
            ROS_DBG("Solving boundary problem (%d).", m_index);
            m_Problem->m_pBoundarySovler->Solve(&(m_Problem->m_BoundaryProblem));
        }
        double t1 = Tic();
        m_poseOut = m_Problem->SimulateTrajectory(m_Sample, m_index);
        double t2 = Tic();
        m_dErrorOut = m_Problem->_CalculateSampleError(m_Sample);
        double norm = m_Problem->_CalculateErrorNorm(m_dErrorOut);

        // DLOG(INFO) << "Ran ACF." 
        //     << " Index " << m_index << "."
        // //     // << "|  SolveBoundary " << (m_bSolveBoundary ? "true" : "false") << ".\n"
        // //     // << "|  Problem OptParams " << m_Problem.m_CurrentSolution.m_dOptParams.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << ".\n"
        //     << "\n PoseOut (of " << std::to_string(m_Sample.m_vStates.size()) << ") " << m_poseOut.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << "."
        //     << "\n ErrorsOut " << m_dErrorOut.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << "."
        //     ;

        // ROS_INFO("Ran ACF, idx %d, final pose [%s], error [%s], norm %f, simtraj took %.2fs",
        //     m_index,
        //     convertEigenMatrix2String(m_poseOut.transpose()).c_str(), 
        //     convertEigenMatrix2String(m_dErrorOut.transpose()).c_str(),
        //     norm,
        //     t2-t1);

        return;
    }

    // MochaPlanner *m_pPlanner;
    //const double m_startingCurvature;
    //const double m_dt;
    std::shared_ptr<MochaProblem> m_Problem;
    const int m_index;
    Eigen::Vector6d& m_poseOut;
    Eigen::VectorXd& m_dErrorOut;
    MotionSample& m_Sample;
    bool m_bSolveBoundary;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// struct ApplyCommandsThreadPreFunctor {
//     ApplyCommandsThreadPreFunctor(std::shared_ptr<MochaProblem> problem,const int index , Eigen::Vector6d& poseOut,Eigen::VectorXd& errorOut,
//                                MotionSample& sample, const bool bSolveBoundary = false) :
//         m_Problem(problem),
//         m_index(index),
//         m_poseOut(poseOut),
//         m_dErrorOut(errorOut),
//         m_Sample(sample),
//         m_bSolveBoundary(bSolveBoundary)
//     {
//         // ROS_INFO("Constructing ApplyCommandsFunctor for %s idx %d", m_Problem.m_problemName.c_str(), m_index);
//     }

//     void operator()()
//     {
//         SetThreadName((boost::format("Bullet AC Thread #%d") % m_index).str().c_str());
//         if(m_bSolveBoundary){
//             // ROS_INFO("Solving BVP for %s", m_Problem->m_problemName.c_str());
//             ROS_DBG("Solving boundary problem.");
//             m_Problem->m_pBoundarySovler->Solve(&(m_Problem->m_BoundaryProblem));
//         }
//         // m_poseOut = m_Problem->SimulateTrajectory(m_Sample, m_index);
//         m_Sample.Clear();
//         bool bUsingBestSolution = false;
//         if(bBestSolution && m_pBestSolution != NULL && m_pBestSolution->m_Sample.m_vCommands.size() != 0){
//             ROS_DBG(" with best solution.");
//             bUsingBestSolution = true;
//             m_Sample.m_vCommands = m_pBestSolution->m_Sample.m_vCommands;
//         }else{
//             ROS_DBG(" with new solution.");
//             // calc sample.commands from problem velocity profile
//             SampleAcceleration(m_Sample.m_vCommands);
//         }
//         return;
//     }

//     std::shared_ptr<MochaProblem> m_Problem;
//     const int m_index;
//     Eigen::Vector6d& m_poseOut;
//     Eigen::VectorXd& m_dErrorOut;
//     MotionSample& m_Sample;
//     bool m_bSolveBoundary;

// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

// struct ApplyVelocitiesThreadFunctor {
//     ApplyVelocitiesThreadFunctor(std::shared_ptr<MochaProblem> problem,const int index , Eigen::Vector6d& poseOut,Eigen::VectorXd& errorOut,
//                                MotionSample& sample, const bool bSolveBoundary = false) :
//         m_Problem(problem),
//         m_index(index),
//         m_poseOut(poseOut),
//         m_dErrorOut(errorOut),
//         m_Sample(sample),
//         m_bSolveBoundary(bSolveBoundary)
//     {
//         // ROS_INFO("Constructing ApplyCommandsFunctor for %s idx %d", m_Problem.m_problemName.c_str(), m_index);
//     }

//     void operator()()
//     {
//         SetThreadName((boost::format("Bullet AV Thread #%d") % m_index).str().c_str());
//         if(m_bSolveBoundary){
//             // ROS_INFO("Solving BVP for %s", m_Problem->m_problemName.c_str());
//             ROS_DBG("Solving boundary problem.");
//             m_Problem->m_pBoundarySovler->Solve(&(m_Problem->m_BoundaryProblem));
//         }
//         m_poseOut = m_Problem->SimulateTrajectory(m_Sample, m_index);
//         return;
//     }

//     std::shared_ptr<MochaProblem> m_Problem;
//     const int m_index;
//     Eigen::Vector6d& m_poseOut;
//     Eigen::VectorXd& m_dErrorOut;
//     MotionSample& m_Sample;
//     bool m_bSolveBoundary;

// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

MochaProblem::MochaProblem(std::string name) :
    m_dSegmentTime(-1), 
    m_dStartTime(-1.0),
    m_dT(0.01),
    // m_pFunctor(m_pf)
    // m_nh(nh),
    m_problemName(name),
    m_dEps(0.1),
    m_dPointWeight(Eigen::MatrixXd(1,1)),
    m_dTrajWeight(Eigen::MatrixXd(1,1)),
    m_nPlanCounter(0)
    // m_actionGetInertiaTensor_client("vehicle/manager/get_inertia_tensor",true),
    // m_actionSetNoDelay_client("vehicle/manager/set_no_delay",true)//,
    // m_actionGetControlDelay_client("vehicle/manager/get_control_delay",true)
{
    Reset();

    // m_clientsApplyVelocities.clear();
    // for (uint i=0; i<GetNumWorldsRequired(); i++)
    // {
    //     ApplyVelocitiesClient* client = new ApplyVelocitiesClient("vehicle/"+std::to_string(i)+"/apply_velocities",true);
    //     m_clientsApplyVelocities.push_back(client);
    //     client->waitForServer();
    // }
}

MochaProblem::MochaProblem(std::string name, const VehicleState& startState, const VehicleState& goalState, const double& dt) :
    m_dSegmentTime(-1), 
    m_dStartTime(-1.0),
    m_dT(dt),
    // m_pFunctor(m_pf)
    // m_nh(nh),
    m_problemName(name),
    m_dEps(0.1),
    m_dPointWeight(Eigen::MatrixXd(1,1)),
    m_dTrajWeight(Eigen::MatrixXd(1,1)),
    m_nPlanCounter(0)
    // m_actionGetInertiaTensor_client("vehicle/manager/get_inertia_tensor",true),
    // m_actionSetNoDelay_client("vehicle/manager/set_no_delay",true)//,
    // m_actionGetControlDelay_client("vehicle/manager/get_control_delay",true)
{
    Reset();
    m_StartState = startState;
    m_GoalState = goalState;

    // m_clientsApplyVelocities.clear();
    // for (uint i=0; i<GetNumWorldsRequired(); i++)
    // {
    //     ApplyVelocitiesClient* client = new ApplyVelocitiesClient("vehicle/"+std::to_string(i)+"/apply_velocities",true);
    //     m_clientsApplyVelocities.push_back(client);
    //     client->waitForServer();
    // }
}

MochaProblem::~MochaProblem()
{
}

MochaProblem::MochaProblem(const MochaProblem& problem)
{
    m_dTorqueStartTime = problem.m_dTorqueStartTime;
    m_dCoefs = problem.m_dCoefs;
    m_dStartTorques = problem.m_dStartTorques;
    m_bInertialControlActive = problem.m_bInertialControlActive;
    m_pBestSolution = problem.m_pBestSolution;
    m_lSolutions = problem.m_lSolutions;
    m_CurrentSolution = problem.m_CurrentSolution;

    m_StartState = problem.m_StartState;
    m_GoalState = problem.m_GoalState;

    m_dStartPose = problem.m_dStartPose;   //< Starting 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature,v]'
    m_dGoalPose = problem.m_dGoalPose;    //< Goal 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature,v]'
    m_dTinv = problem.m_dTinv;
    m_dT3dInv = problem.m_dT3dInv;
    m_dT3d = problem.m_dT3d;

    //Eigen::VectorCubic2D m_dCubic;
    m_dSegmentTime = problem.m_dSegmentTime;
    m_dMaxSegmentTime = problem.m_dMaxSegmentTime;                   //< This is to stop runaway simulation in case the gauss newton delta destroys the solution

    m_vVelProfile = problem.m_vVelProfile;              //< Velocity profile for this trajectory
    m_vAccelProfile = problem.m_vAccelProfile;        //< Acceleration profile for this trajectory

    m_dStartTime = problem.m_dStartTime;

    m_dOptParams = problem.m_dOptParams;               //< Optimization parameters, which are parametrized as [x,y,theta,accel,aggr]
    m_dInitOptParams = problem.m_dInitOptParams;
    m_dTransformedGoal = problem.m_dTransformedGoal;
    m_dT = problem.m_dT;             
    
    m_BoundaryProblem = problem.m_BoundaryProblem;           //< The boundary problem structure, describing the 2D boundary problem
     
    m_eError = problem.m_eError;

    m_eCostMode = problem.m_eCostMode;
    m_Trajectory = problem.m_Trajectory;
    
    m_dCoefs = problem.m_dCoefs;
    m_bInertialControlActive = problem.m_bInertialControlActive;
    m_dTorqueStartTime = problem.m_dTorqueStartTime;
    m_dTorqueDuration = problem.m_dTorqueDuration;
    m_dStartTorques = problem.m_dStartTorques;

    m_lSolutions = problem.m_lSolutions;
    m_pBestSolution = new Solution(*problem.m_pBestSolution);
    m_vBestSolutionErrors = problem.m_vBestSolutionErrors;
    m_CurrentSolution = problem.m_CurrentSolution;  

    m_ThreadPool = problem.m_ThreadPool;                       //< Threadpool for multitasking in jacobians and damping calculation
    m_dEps = problem.m_dEps;                                              //< The epsilon used in the calculation of the finite difference jacobian

    m_BoundarySolver = problem.m_BoundarySolver;                      //< The boundary value problem solver
    m_pBoundarySovler = &m_BoundarySolver;          //< Pointer to the boundary value solver which will be used to solve the 2D problem

    m_dPointWeight = problem.m_dPointWeight;                                       //< The matrix which holds the weighted Gauss-Newton weights
    m_dTrajWeight = problem.m_dTrajWeight;
    m_nPlanCounter = problem.m_nPlanCounter;

    m_nh = problem.m_nh;

    // m_clientsApplyVelocities.clear();
    // for (uint i=0; i<GetNumWorldsRequired(); i++)
    // {
    //     ApplyVelocitiesClient* client = new ApplyVelocitiesClient("vehicle/"+std::to_string(i)+"/apply_velocities",true);
    //     m_clientsApplyVelocities.push_back(client);
    //     client->waitForServer();
    // }
}

void MochaProblem::Reset()
{
    m_dTorqueStartTime = -1;
    m_dCoefs = Eigen::Vector4d::Zero();
    m_dStartTorques = Eigen::Vector3d::Zero();
    m_bInertialControlActive = false;
    m_pBestSolution = nullptr;
    m_lSolutions.clear();
    m_CurrentSolution = Solution();
    m_CurrentSolution.m_dNorm = DBL_MAX;

    m_ThreadPool.size_controller().resize(8);

    m_dPointWeight = Eigen::MatrixXd(POINT_COST_ERROR_TERMS,1);
    m_dTrajWeight = Eigen::MatrixXd(TRAJ_EXTRA_ERROR_TERMS+TRAJ_UNIT_ERROR_TERMS,1);
    m_dPointWeight.setIdentity();
    m_dTrajWeight.setIdentity();
}

void MochaProblem::SetTimeInterval(double dt)
{   
    if (dt<=0)
    {
        ROS_WARN("Invalid time interval. Skipping.");
        return;
    }
    m_dT = dt;
}

void MochaProblem::SetParameterEpsilon(double eps)
{   
    if (eps<=0)
    {
        ROS_WARN("Invalid epsilon. Skipping.");
        return;
    }
    m_dEps = eps;
}

void MochaProblem::SetPointWeights(Eigen::VectorXd weights)
{   
    if (weights.size() != m_dPointWeight.size())
    {
        ROS_WARN("Invalid vector size in SetPointWeights. Skipping.");
        return;
    }
    m_dPointWeight = weights;
    // m_dPointWeight(0) = X_WEIGHT;
    // m_dPointWeight(1) = Y_WEIGHT;
    // m_dPointWeight(2) = Z_WEIGHT;
    // m_dPointWeight(3) = THETA_WEIGHT;
    // m_dPointWeight(4) = VEL_WEIGHT_POINT;
    // // m_dPointWeight(5) = CURV_WEIGHT;
    // m_dPointWeight(5) = TILT_WEIGHT;
    // m_dPointWeight(6) = CONTACT_WEIGHT;
}

void MochaProblem::SetTrajWeights(Eigen::VectorXd weights)
{
    if (weights.size() != m_dTrajWeight.size())
    {
        ROS_WARN("Invalid vector size in SetTrajWeights. Skipping.");
        return;
    }
    m_dTrajWeight = weights;
    // m_dTrajWeight(0) = X_WEIGHT;
    // m_dTrajWeight(1) = Y_WEIGHT;
    // m_dTrajWeight(2) = Z_WEIGHT;
    // m_dTrajWeight(3) = THETA_WEIGHT;
    // m_dTrajWeight(4) = VEL_WEIGHT_TRAJ;
    // m_dTrajWeight(5) = TIME_WEIGHT;
    // m_dTrajWeight(6) = CURV_WEIGHT;
    // // m_dTrajWeight(7) = BADNESS_WEIGHT;
}

// void MochaProblem::ApplyCommands(std::vector<Eigen::VectorXd> delta)
// {
//     uint numThreads = 2*epsVec.size();

//     Eigen::Vector6d pPoses[numThreads], dCurrentPose;
//     Eigen::VectorXd errors[numThreads], dCurrentError;

//     std::vector<std::shared_ptr<MochaProblem>> vCubicProblems;
//     vCubicProblems.resize(numThreads);

//     std::vector<std::shared_ptr<ApplyCommandsThreadFunctor > > vFunctors;
//     vFunctors.resize(numThreads);
    
//     for( int ii = 0; ii < numThreads; ii++ )
//     {
//         int plusIdx = ii*2, minusIdx = ii*2+1;
//         vCubicProblems[plusIdx] = std::make_shared<MochaProblem>(*this);
//         Eigen::VectorXd delta(OPT_DIM);
//         delta.setZero();
//         delta(ii) += dEps;
//         vCubicProblems[plusIdx]->UpdateOptParams(vCubicProblems[plusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
//         vFunctors[plusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(vCubicProblems[plusIdx],
//                                                                           plusIdx,
//                                                                           pPoses[plusIdx],
//                                                                           errors[plusIdx],
//                                                                           (vCubicProblems[plusIdx]->m_CurrentSolution.m_Sample),
//                                                                           true);
//         m_ThreadPool.schedule(*vFunctors[plusIdx]);
//     }
// }

void MochaProblem::SamplePath(Eigen::Vector3dAlignedVec& vSamples, bool bBestSolution /* = true */ )
{
    vSamples.clear();
    BezierBoundaryProblem boundaryProblem = m_BoundaryProblem;
    Sophus::SE2d T_start(Sophus::SO2(m_dStartPose[3]), m_dStartPose.head(2));
    vSamples.reserve(boundaryProblem.m_vPts.size());
    for(const Eigen::Vector2d& pt : boundaryProblem.m_vPts) {
        Eigen::Vector3d dPos(pt[0],pt[1],0);
        dPos.head(2) = T_start*dPos.head(2);

        //now transform this into the proper 3d pose
        dPos = m_dT3dInv * dPos;
        vSamples.push_back(dPos);
    }
}

Eigen::Vector6d MochaProblem::_Transform3dGoalPose(const VehicleState& state) const
{
    //also transfer this pose into the 3d projected space we are working on
    const Sophus::SE3d dPose = m_dT3d * state.m_dTwv;
    Eigen::Vector6d untransformedPose;
    untransformedPose << dPose.translation()[0],
                         dPose.translation()[1],
                         dPose.translation()[2],
                         atan2( dPose.matrix()(1,0), dPose.matrix()(0,0)),
                         0,
                         state.m_dV.norm();
    return _TransformGoalPose(untransformedPose);
}

Eigen::Vector6d  MochaProblem::_TransformGoalPose(const Eigen::Vector6d& dGoalPose) const
{
    Eigen::Vector6d result;
    Eigen::Vector3d pt;
    pt << dGoalPose[0], dGoalPose[1], 1;  // x y 1
    pt = m_dTinv * pt;
    result << pt[0], pt[1], dGoalPose[2], rpg::AngleWrap( dGoalPose[3] - m_dStartPose[3] ), dGoalPose[4], dGoalPose[5];  // x_transformed y_transformed z t_clamped c v
    return result;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd MochaProblem::_GetTrajectoryError(const MotionSample& sample,
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
Eigen::VectorXd MochaProblem::_GetWeightVector()
{
    int errorVecSize = m_eCostMode == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd dW;
    Eigen::VectorXd trajW(errorVecSize);
    if(m_eCostMode == eCostTrajectory){
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
double MochaProblem::_CalculateErrorNorm(const Eigen::VectorXd& dError)
{
    ROS_DBG("Calculating weighted error norm.");
    Eigen::VectorXd dW = _GetWeightVector();
    Eigen::VectorXd error = dError;
    // ROS_INFO("error %f %f %f %f %f %f weights %f %f %f %f %f %f", error[0], error[1], error[2], error[3], error[4], error[5], dW[0], dW[1], dW[2], dW[3], dW[4], dW[5]);
    //DLOG(INFO) << "error vector is " << error.transpose();
    error.array() *= dW.array();
    return error.norm();
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd MochaProblem::_CalculateSampleError(const MotionSample& sample, double& dMinTrajTime) const
{
    ROS_DBG("Calculating sample error.");
    if(sample.m_vStates.size() == 0 ){
        // DLOG(ERROR) << m_nPlanId << ":Sample with size 0 detected. Aborting.";
        ROS_WARN("%d:Sample with size 0 detected. Aborting.", m_nPlanId);
        int errorVecSize = m_eCostMode == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS;
        Eigen::VectorXd error;
        error = Eigen::VectorXd(errorVecSize);
        error.setOnes();
        error *= DBL_MAX;
        return error;
    }
    
    switch(m_eCostMode)
    {
        case eCostPoint:
            return CalculatePointError(sample);
        case eCostTrajectory:
            return CalculateTrajectoryError(sample, dMinTrajTime);
    }
}

Eigen::VectorXd MochaProblem::CalculatePointError(const MotionSample& sample) const
{

    //get the normalized velocity
    VehicleState state = sample.m_vStates.back();
    Eigen::Vector3d dW_goal = m_GoalState.m_dTwv.so3().inverse()* state.m_dW;
    //double dPrevAngle = state.GetTheta();
    
    if(state.IsAirborne()){
        state.AlignWithVelocityVector();
        //DLOG(INFO) << "End state transformed from " << dPrevAngle << " to " << state.GetTheta();
    }
    Eigen::Vector6d endPose = _Transform3dGoalPose(state);

    Eigen::VectorXd error(POINT_COST_ERROR_TERMS);
    //Eigen::Vector3d cartError = T2Cart_2D(Cart2T_2D(m_dTransformedGoal.head(3))*Tinv_2D(Cart2T_2D(endPose.head(3))));
    //error.head(3) = cartError;
    //transform the angular velocity into the plane of goal pose
    
    error.head(3) = m_dTransformedGoal.head(3) - endPose.head(3);
    error[3] = rpg::AngleWrap(m_dTransformedGoal[3] - endPose[3]);
    error[4] = m_dTransformedGoal[5] - endPose[5];
    //error[5] = state.m_dV.norm()*dW_goal[2] - m_GoalState.m_dCurvature;

    error[5] = sample.GetTiltCost();
    error[6] = sample.GetContactCost();
    error[7] = sample.GetCollisionCost();
    //error[5] = -std::log(m_BoundaryProblem->m_dAggressiveness);
    //error.array() *= m_dPointWeight.block<POINT_COST_ERROR_TERMS,1>(0,0).array();
    //DLOG(INFO) << "Error vector is " << error.transpose() << " weights are " << m_dPointWeight.transpose();

    return error;
}

Eigen::VectorXd MochaProblem::CalculateTrajectoryError(const MotionSample& sample, double& dMinTrajTime) const
{
    //get the normalized velocity
    VehicleState state = sample.m_vStates.back();
    Eigen::Vector3d dW_goal = m_GoalState.m_dTwv.so3().inverse()* state.m_dW;
    //double dPrevAngle = state.GetTheta();
    if(state.IsAirborne()){
        state.AlignWithVelocityVector();
        //DLOG(INFO) << "End state transformed from " << dPrevAngle << " to " << state.GetTheta();
    }
    Eigen::Vector6d endPose = _Transform3dGoalPose(state);

    Eigen::VectorXd error(TRAJ_UNIT_ERROR_TERMS*g_nTrajectoryCostSegments+TRAJ_EXTRA_ERROR_TERMS);
    Eigen::Vector6dAlignedVec m_vTransformedTrajectory;
    if(m_vTransformedTrajectory.empty()){
        int nTrajSize = m_Trajectory.m_vStates.size();
        m_vTransformedTrajectory.reserve(nTrajSize);
        //push back the transformed poses
        for(int ii = 0; ii < nTrajSize ; ii++){
            VehicleState tempState = m_Trajectory.m_vStates[ii];
            if(tempState.IsAirborne()){
                tempState.AlignWithVelocityVector();
            }
            m_vTransformedTrajectory.push_back(_Transform3dGoalPose(tempState));
        }
    }

    error.setZero();
    error.segment(TRAJ_UNIT_ERROR_TERMS*(g_nTrajectoryCostSegments-1),TRAJ_UNIT_ERROR_TERMS)  = _GetTrajectoryError(m_Trajectory,
                                                                                                                        m_vTransformedTrajectory,
                                                                                                                        endPose,
                                                                                                                        dMinTrajTime);

    //now that we have the minimum point, calculate trajectory error points along the way
    int counter = 0;
    int nStartIndex = 0;

    for(double ii = dMinTrajTime/(double)g_nTrajectoryCostSegments ;
                ii < dMinTrajTime && counter < ((double)g_nTrajectoryCostSegments-1) ;
                ii += dMinTrajTime/(double)g_nTrajectoryCostSegments ){
        //get the poses at this point in time
        //Eigen::Vector6d poseTraj = _Transform3dGoalPose(VehicleState::GetInterpolatedState(m_Trajectory.m_vStates,nStartIndex,ii,nStartIndex));
        Eigen::Vector6d poseSample = _Transform3dGoalPose(VehicleState::GetInterpolatedState(sample.m_vStates,nStartIndex,ii,nStartIndex));

        double dMinTime;
        error.segment(TRAJ_UNIT_ERROR_TERMS*counter,TRAJ_UNIT_ERROR_TERMS) = _GetTrajectoryError(m_Trajectory,
                                                                                                    m_vTransformedTrajectory,
                                                                                                    poseSample,
                                                                                                dMinTime);
        counter++;
    }
    error /= (double)g_nTrajectoryCostSegments;


    //segment cost

    error[error.rows()-2] = g_dTimeTarget-dMinTrajTime;
    error[error.rows()-1] = state.m_dV.norm()*dW_goal[2] - m_GoalState.m_dCurvature; // sample.GetBadnessCost();

    //error[7] = sample.GetBadnessCost();
    //error[6] = -std::log(m_BoundaryProblem->m_dAggressiveness);
    //error.array() *= m_dTrajWeight.block<TRAJ_COST_ERROR_TERMS,1>(0,0).array();
    return error;
}

///////////////////////////////////////////////////////////////////////
bool MochaProblem::_CalculateJacobian(Eigen::VectorXd& dCurrentErrorVec,
                                      Solution& coordinateDescent,
                                      Eigen::MatrixXd& J)
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");

    Eigen::Vector6d pPoses[OPT_DIM*2],dCurrentPose;
    Eigen::VectorXd errors[OPT_DIM*2],dCurrentError;

    std::vector<std::shared_ptr<MochaProblem>> vCubicProblems;
    vCubicProblems.resize(OPT_DIM*2);

    std::vector<std::shared_ptr<ApplyCommandsThreadFunctor > > vCommandsFunctors;
    vCommandsFunctors.resize(OPT_DIM*2);

    const double dEps = m_dEps;// * m_CurrentSolution.m_dNorm;
    // DLOG(INFO) << "Calcing Jacobian";
    // ROS_INFO("Calculating Jacobian.");
    ROS_INFO("Calcing Jacobian on %d threads", OPT_DIM*2+1);
    double t0 = Tic();
    //g_bUseCentralDifferences = false;
    for( int ii = 0; ii < OPT_DIM; ii++ )
    {
        int plusIdx = ii*2, minusIdx = ii*2+1;
        vCubicProblems[plusIdx] = std::make_shared<MochaProblem>(*this);
        Eigen::VectorXd delta(OPT_DIM);
        delta.setZero();
        delta(ii) += dEps;
        vCubicProblems[plusIdx]->UpdateOptParams(vCubicProblems[plusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
        vCommandsFunctors[plusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(vCubicProblems[plusIdx],
                                                                          plusIdx,
                                                                          pPoses[plusIdx],
                                                                          errors[plusIdx],
                                                                          (vCubicProblems[plusIdx]->m_CurrentSolution.m_Sample),
                                                                          true);
        m_ThreadPool.schedule(*vCommandsFunctors[plusIdx]);

        if(g_bUseCentralDifferences == true){
            vCubicProblems[minusIdx] = std::make_shared<MochaProblem>(*this);
            Eigen::VectorXd delta(OPT_DIM);
            delta.setZero();
            delta(ii) -= dEps;
            vCubicProblems[minusIdx]->UpdateOptParams(vCubicProblems[minusIdx]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vCommandsFunctors[minusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(vCubicProblems[minusIdx],
                                                                               minusIdx,
                                                                               pPoses[minusIdx],
                                                                               errors[minusIdx],
                                                                               (vCubicProblems[minusIdx]->m_CurrentSolution.m_Sample),
                                                                               true);
            m_ThreadPool.schedule(*vCommandsFunctors[minusIdx]);
        }
    }
    // MochaVehicle::ApplyAllVelocitiesFromClient(ApplyAllVelocitiesClient* client,
    //                                             const std::vector<VehicleState>& startStates,
    //                                             std::vector<MotionSample>& samples,
    //                                             bool noCompensation=false,
    //                                             bool noDelay=false);
    // for (int ii=0; i<samples.size(); i++)
    // {
    //     VehicleState vState;
    //     if(samples[ii].m_vCommands.size() == 0){
    //         vState = m_StartState;
    //     }else {
    //         if (samples[ii].m_vStates.size()>0)
    //             vState = samples[ii].m_vStates.back();
    //         else
    //             vState = VehicleState();
    //     }
    //     pPoses[ii] = _Transform3dGoalPose(vState);
    //     errors[ii] = _CalculateSampleError(samples[ii]);
    // }

    std::shared_ptr<MochaProblem > currentProblem = std::make_shared<MochaProblem>(*this);
    std::shared_ptr<ApplyCommandsThreadFunctor > currentFunctor =
        std::make_shared<ApplyCommandsThreadFunctor>(currentProblem,
                                                     OPT_DIM*2,
                                                     dCurrentPose,
                                                     dCurrentError,
                                                     (currentProblem->m_CurrentSolution.m_Sample),
                                                     true);
    m_ThreadPool.schedule(*currentFunctor);
    // MochaVehicle::ApplyVelocitiesFromClient(ApplyVelocitiesClient* client,
    //                                             const std::vector<VehicleState>& startStates,
    //                                             std::vector<MotionSample>& samples,
    //                                             OPT_DIM*2,
    //                                             bool noCompensation=false,
    //                                             bool noDelay=false);

    //wait for all simulations to finish
    // DLOG(INFO) << "Waiting for threads to finish...";
    // ROS_INFO("Waiting for ACF threads to finish...");
    // boost::xtime t = boost::xtime::system_time();
    // // t.system_time();
    // t.sec += 1;
    m_ThreadPool.wait();
    // DLOG(INFO) << "Done waiting.";
    // ROS_INFO("Done waiting.");
    ROS_DBG("Jacobian calulation done, took %fs.", Toc(t0));

    // ROS_INFO("Pubbing potential paths viz from Coordinate Descent.");
    for( int ii = 0; ii < OPT_DIM ; ii++ )
    {
        int plusIdx = ii*2, minusIdx = ii*2+1;
        m_vPotentialPaths.push_back(vCubicProblems[plusIdx]->m_CurrentSolution.m_Sample);
        if(g_bUseCentralDifferences == true)
        {
            m_vPotentialPaths.push_back(vCubicProblems[plusIdx]->m_CurrentSolution.m_Sample);
        }
    }
    // pubPotentialPathsViz();

    ROS_DBG("Calculating coordinate descent solution.");

    dCurrentErrorVec = dCurrentError;

    std::shared_ptr<MochaProblem> pCoordinateDescent;
    double dBestNorm = DBL_MAX;

    for( int ii = 0; ii < OPT_DIM ; ii++ ){
        int plusIdx = ii*2, minusIdx = ii*2+1;
        double norm = vCubicProblems[plusIdx]->_CalculateErrorNorm(errors[plusIdx]);
        if(std::isfinite(norm)){
            if( norm < dBestNorm ) {
                dBestNorm = norm;
                pCoordinateDescent = (vCubicProblems[plusIdx]);
            }
        }

        if(g_bUseCentralDifferences == true){
            norm = vCubicProblems[minusIdx]->_CalculateErrorNorm(errors[minusIdx]);
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
        // ROS_DBG(str);

        J.col(ii) = -col;
        //if this term is NAN, sound the alarm
        if(std::isfinite(col[0]) == false || std::isfinite(col[1]) == false ||
           std::isfinite(col[2]) == false || std::isfinite(col[3]) == false){
            m_eError = eJacobianColumnNan;
            return false;
        }

        //if this column is zero
//        if( col.norm() == 0){
//            m_eError = eJacobianColumnZero;
//            return false;
//        }
    }

    coordinateDescent = pCoordinateDescent->m_CurrentSolution;
    coordinateDescent.m_dNorm = dBestNorm;
    
    // ROS_INFO("Done calculating Jacobian\n%s\nwith curr norm %f, CD norm %f\nerrors [%s]", convertEigenMatrix2String(J,2,", ","\n","\t").c_str(), m_CurrentSolution.m_dNorm, dBestNorm, convertEigenMatrix2String(dCurrentErrorVec.transpose()).c_str());

    // if(g_bVerbose){
    //     DLOG(INFO) << "Jacobian:" << J.format(CleanFmt) << std::endl;
    // }
    return true;
}


///////////////////////////////////////////////////////////////////////
double MochaProblem::_DistanceTraveled( const double& t,const AccelerationProfile& profile ) const
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
void MochaProblem::SampleAcceleration(std::vector<ControlCommand>& vCommands) 
{
    vCommands.clear();

    //double st;// = SegmentTime(p);
    //AccelerationProfile accelProfile;
    _GetAccelerationProfile();
    //m_dSegmentTime += m_dSegmentTimeDelta;
    if(std::isfinite(m_dSegmentTime) == false ){
        // DLOG(ERROR) << m_nPlanId << ":Segment time of " << m_dSegmentTime << " was not finite.";
        ROS_WARN("%d:Segment time of %d was not finite.", m_nPlanId, m_dSegmentTime);
        return;
    }
    double t;
    m_dSegmentTime = std::min(m_dSegmentTime,m_dMaxSegmentTime);
    int numSamples = (int)(m_dSegmentTime/m_dT + 1.0);
    vCommands.reserve(numSamples);

    size_t accelIndex = 0;
    for( t = 0; t < (m_dSegmentTime) ; t+= m_dT  ){
        double curvature = m_pBoundarySovler->GetCurvature(&m_BoundaryProblem,
                                                                   _DistanceTraveled(t,m_vAccelProfile));

        double step = m_dSegmentTime - t;
        double actualDt = std::min(m_dT,step);

        if(m_vAccelProfile[accelIndex].m_dEndTime < t){
            accelIndex++;
        }

        if(accelIndex >= m_vAccelProfile.size()){
            DLOG(ERROR) << m_nPlanId << ":Exceeded bounds of acceleration profile.";
            return;
        }

        //if needed, add torques
        double endTime = m_dTorqueStartTime + m_dTorqueDuration;
        Eigen::Vector3d dTorques = Eigen::Vector3d::Zero();
        if(t >= m_dTorqueStartTime && m_dTorqueStartTime != -1 && t <= (endTime)){
            dTorques(1) = m_dCoefs(0) + m_dCoefs(1)*(t-m_dTorqueStartTime) +
                          m_dCoefs(2)*powi((t-m_dTorqueStartTime),2) +
                          m_dCoefs(3)*powi((t-m_dTorqueStartTime),3);
        }
        double force = m_vAccelProfile[accelIndex].m_dAccel+(m_CurrentSolution.m_dOptParams[OPT_ACCEL_DIM]/m_dSegmentTime);
        vCommands.push_back(ControlCommand(force,curvature,dTorques,actualDt,0));
        // ROS_INFO("adding accel %f at %f", -force, t);
    }
}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d MochaProblem::SimulateTrajectory(MotionSample& sample,
                                                 const int iWorld /*= 0*/,
                                                 const bool& bBestSolution /* = false */)
{
    ROS_DBG("Simulating trajectory (%d)", iWorld);
    double t0 = Tic();

    sample.Clear();
    bool bUsingBestSolution = false;
    if(bBestSolution && m_pBestSolution != NULL && m_pBestSolution->m_Sample.m_vCommands.size() != 0){
        ROS_DBG(" with best solution.");
        bUsingBestSolution = true;
        sample.m_vCommands = m_pBestSolution->m_Sample.m_vCommands;
    }else{
        ROS_DBG(" with new solution.");
        // calc sample.commands from problem velocity profile
        SampleAcceleration(sample.m_vCommands);
    }

    VehicleState vState;
    if(sample.m_vCommands.size() == 0){
        vState = m_StartState;
    }else {
        // augment sample.commands with compensation (gravity, steering, friction) for RaycastVehicle 
        // and use with BulletCarModel::UpdateState to populate sample.states

        // DLOG(INFO) << "Commands(" << sample.m_vCommands.size() << "):";
        // for(uint i=0; i<sample.m_vCommands.size(); i++) {
        //     DLOG(INFO) << "  " << sample.m_vCommands[i].m_dForce << " " << sample.m_vCommands[i].m_dCurvature << " " << sample.m_vCommands[i].m_dT << " " << sample.m_vCommands[i].m_dPhi << " " << sample.m_vCommands[i].m_dTorque.format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]")) << " " << sample.m_vCommands[i].m_dTime;
        // }

        ApplyVelocities( m_StartState, sample, iWorld, bUsingBestSolution, m_bNoDelay);


        // DLOG(INFO) << "States(" << sample.m_vStates.size() << "):";
        // for(uint i=0; i<sample.m_vStates.size(); i++) {
        //     DLOG(INFO) << "  " << sample.m_vStates[i].ToXYZTCV().format(Eigen::IOFormat(8, 0, ", ", "; ", "[", "]"));
        // }

        if (sample.m_vStates.size()>0)
            vState = sample.m_vStates.back();
        else
            vState = VehicleState();
    }

    double t1 = Tic();
    // Eigen::VectorXd finalState = sample.m_vStates.back().ToXYZTCV();
    // Eigen::VectorXd finalParams = m_CurrentSolution.m_dOptParams;
    double minTrajTime;
    Eigen::VectorXd finalErrors = _CalculateSampleError(sample, minTrajTime);
    double finalNorm = _CalculateErrorNorm(finalErrors);
    ROS_INFO("Simulated trajectory w %s soln for world %d w norm %f took %fs",(bUsingBestSolution ? "best" : "new"),iWorld,finalNorm,t1-t0);
    // ROS_INFO("Simulated trajectory for world %d\n final pose [%s],\n opt params [%s],\n error [%s],\n norm %f, took %fs for %d states (%fs/state)",
    //     iWorld,
    //     convertEigenMatrix2String(finalState.transpose()).c_str(), 
    //     convertEigenMatrix2String(finalParams.transpose()).c_str(),
    //     convertEigenMatrix2String(finalErrors.transpose()).c_str(),
    //     finalNorm,
    //     t1-t0, 
    //     sample.m_vStates.size(),
    //     (t1-t0)/sample.m_vStates.size());
    
    // for (uint i=0; i<sample.m_vStates.size(); i+=5)
    // {
    //     ROS_INFO("  %d %f %f %f %f %f", i,
    //         sample.m_vStates[i].ToXYZQuat()[0],
    //         sample.m_vStates[i].ToXYZQuat()[1],
    //         sample.m_vCommands[i].m_dForce,
    //         sample.m_vCommands[i].m_dPhi,
    //         sample.m_vCommands[i].m_dCurvature); 
    // }

    //transform the result back
    Eigen::Vector6d dRes = _Transform3dGoalPose(vState);
    return dRes;
}

// Eigen::Vector6d MochaProblem::SimulateTrajectory(MotionSample& sample,
//                                                  const int iWorld /*= 0*/,
//                                                  const bool& bBestSolution /* = false */)
// {
//     ROS_DBG("Simulating trajectory (%d)", iWorld);
//     double t0 = Tic();

//     sample.Clear();
//     bool bUsingBestSolution = false;
//     if(bBestSolution && m_pBestSolution != NULL && m_pBestSolution->m_Sample.m_vCommands.size() != 0){
//         ROS_DBG(" with best solution.");
//         bUsingBestSolution = true;
//         sample.m_vCommands = m_pBestSolution->m_Sample.m_vCommands;
//     }else{
//         ROS_DBG(" with new solution.");
//         // calc sample.commands from problem velocity profile
//         SampleAcceleration(sample.m_vCommands);
//     }
// }

///////////////////////////////////////////////////////////////////////
void MochaProblem::_GetAccelerationProfile() 
{
    double totalDist = m_BoundaryProblem.m_dDistance;
    double currentDist = 0;
    double totalTime = 0;

    //must have at least two nodes
    assert(m_vVelProfile.size()>1);

    //prepare the accel profile
    m_vAccelProfile.clear();
    m_vAccelProfile.reserve(m_vVelProfile.size());

    //first calcualte the segment time
    for(size_t ii = 1; ii < m_vVelProfile.size(); ii++){
        //calculate the distance in this segment
        double segDist = (m_vVelProfile[ii].m_dDistanceRatio - m_vVelProfile[ii-1].m_dDistanceRatio)*totalDist;
        double segTime = segDist / (m_vVelProfile[ii-1].m_dVel + 0.5 * (m_vVelProfile[ii].m_dVel - m_vVelProfile[ii-1].m_dVel));
        totalTime += segTime;
        currentDist += segDist;

        //push back the accel profile
        double accel = (m_vVelProfile[ii].m_dVel - m_vVelProfile[ii-1].m_dVel)/segTime;
        m_vAccelProfile.push_back(AccelerationProfileNode(totalTime,accel,currentDist,m_vVelProfile[ii-1].m_dVel,m_vVelProfile[ii].m_dVel));
    }
    m_dSegmentTime = totalTime;
    //m_dMaxSegmentTime = DBL_MAX;
}

// void MochaProblem::ApplyVelocitiesDoneCb(const actionlib::SimpleClientGoalState& state,
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

bool MochaProblem::ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int nWorldId /*= 0*/,
                                                      bool noCompensation /*= false*/,
                                                      bool noDelay /*=false*/) {
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

    // ROS_INFO("ApplyVelocities for world %d started", nWorldId);

    // ROS_INFO("Applying velocities (%d)", nWorldId);

    // double t0 = Tic();

    // if (nWorldId >= m_clientsApplyVelocities.size())
    // {
    //     ROS_ERROR("ApplyVelocities client %d doens't exist.",nWorldId);
    //     return false;
    // }
    // ApplyVelocitiesClient* client = m_clientsApplyVelocities[nWorldId];
    ApplyVelocitiesClient client("vehicle/"+std::to_string(nWorldId)+"/apply_velocities",true);
    client.waitForServer();

    carplanner_msgs::ApplyVelocitiesGoal goal;
    carplanner_msgs::ApplyVelocitiesResultConstPtr result;
    
    goal.initial_state = startState.toROS();
    goal.initial_motion_sample = sample.toROS();
    goal.world_id = nWorldId;
    goal.no_compensation = noCompensation;
    goal.no_delay = noDelay;

    double t1 = Tic();
    ROS_DBG("Problem sending AV (%d) goal with %d commands", nWorldId, goal.initial_motion_sample.commands.size());
    // while(!client.isServerConnected())
    // {
    //     ROS_WARN_THROTTLE(0.1, "AV (%d) disconnected.", nWorldId);
    // }
    client.sendGoal(goal
        // , boost::bind(&MochaProblem::ApplyVelocitiesDoneCb, this, _1, _2)
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
        );

    double t2 = Tic();

    // {
    // boost::mutex::scoped_lock lock(m_mutexApplyVelocitiesInfo);
    // m_vApplyVelocitiesGoalIds.push_back(goal.goal_id);
    // m_vApplyVelocitiesMotionSamples.push_back(&sample);
    // }

    // DLOG(INFO) << "AV called:" 
    //     << " world " << std::to_string(goal.world_id) 
      // "\nstart " << std::to_string(VehicleState::fromROS(goal->initial_state))
      ;

    bool success;

    float timeout(15.0);
    timeout = 0.1 * goal.initial_motion_sample.commands.size();
    bool finished_before_timeout = client.waitForResult(ros::Duration(timeout));

    double t3 = Tic();
    ROS_INFO("Got goal result (%d), took %fs", nWorldId, t3-t1);
    if (finished_before_timeout)
    {
        success = true;
        actionlib::SimpleClientGoalState state = client.getState();
        // DLOG(INFO) << "ApplyVelocities finished: " << state.toString();

        // ROS_INFO("Applying velocities (%d) succeeded, took %.2fs.", nWorldId, t3-t1);

        result = client.getResult();
        sample.fromROS(result->motion_sample);
    }
    else
    {
        success = false;
        ROS_ERROR("ApplyVelocities (%d) did not finish before the %fs timeout.", nWorldId, timeout);
    }

    return success;
}

void MochaProblem::GetPotentialPaths(std::vector<MotionSample>& paths)
{
    for (uint ii=0; ii<m_vPotentialPaths.size(); ii++)
    {
        paths.push_back(m_vPotentialPaths[ii]);
    }
    m_vPotentialPaths.clear();
}

// double MochaProblem::GetControlDelay(int nWorldId)
// {
//     double valOut;

//     actionlib::SimpleActionClient<carplanner_msgs::GetControlDelayAction> actionGetControlDelay_client("vehicle/manager/get_control_delay",true);

//     carplanner_msgs::GetControlDelayGoal goal;
//     carplanner_msgs::GetControlDelayResultConstPtr result;
    
//     goal.worldId = nWorldId;
//     actionGetControlDelay_client.sendGoal(goal);

//     bool finished_before_timeout = actionGetControlDelay_client.waitForResult(ros::Duration(0.5));
//     if (finished_before_timeout)
//     {
//         actionlib::SimpleClientGoalState state = actionGetControlDelay_client.getState();
//         // ROS_INFO("GetControlDelay Action finished: %s", state.toString().c_str());

//         result = actionGetControlDelay_client.getResult();
//         valOut = result->val;
//     }
//     else
//         ROS_WARN("GetControlDelay Action did not finish before the time out.");

//     return valOut;

//     // m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
// }

// const Eigen::Vector3d MochaProblem::GetInertiaTensor(int nWorldId)
// { 
//     actionlib::SimpleActionClient<carplanner_msgs::GetInertiaTensorAction> actionGetInertiaTensor_client("vehicle/manager/get_inertia_tensor",true);
//     carplanner_msgs::GetInertiaTensorGoal goal;
//     carplanner_msgs::GetInertiaTensorResultConstPtr result;

//     actionGetInertiaTensor_client.waitForServer();

//     goal.world_id = nWorldId;
//     actionGetInertiaTensor_client.sendGoal(goal);

//     bool finished_before_timeout = actionGetInertiaTensor_client.waitForResult(ros::Duration(0.5));
//     if (finished_before_timeout)
//     {
//         actionlib::SimpleClientGoalState state = actionGetInertiaTensor_client.getState();
//         // ROS_INFO("GetInertiaTensor Action finished: %s", state.toString().c_str());

//         result = actionGetInertiaTensor_client.getResult();
//     }
//     else
//     {
//         ROS_WARN("GetInertiaTensor Action did not finish before the time out.");
//         return Eigen::Vector3d();
//     }

//     const Eigen::Vector3d valsOut(result->vals[0], result->vals[1], result->vals[2]);
//     return valsOut;
// }

// void MochaProblem::SetNoDelay(bool no_delay)
// {
//     actionlib::SimpleActionClient<carplanner_msgs::SetNoDelayAction> actionSetNoDelay_client("vehicle/manager/set_no_delay",true);
//     carplanner_msgs::SetNoDelayGoal goal;
//     carplanner_msgs::SetNoDelayResultConstPtr result;

//     goal.no_delay = no_delay;
//     actionSetNoDelay_client.sendGoal(goal);

//     bool finished_before_timeout = actionSetNoDelay_client.waitForResult(ros::Duration(0.5));
//     if (finished_before_timeout)
//     {
//         actionlib::SimpleClientGoalState state = actionSetNoDelay_client.getState();
//         // ROS_INFO("SetNoDelay Action finished: %s", state.toString().c_str());

//     }
//     else
//     {
//         ROS_WARN("SetNoDelay Action did not finish before the time out.");
//     }
//     return;
// }

///////////////////////////////////////////////////////////////////////
bool MochaProblem::Initialize(const double dStartTime,
                                          const VelocityProfile* pVelProfile /* = NULL*/,
                                          CostMode eCostMode /*= eCostPoint */)
{
    ROS_DBG("Initializing problem %s starting %f",m_problemName.c_str(), dStartTime);
    boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);

    Reset();
    m_nPlanId = m_nPlanCounter++;

    //if there are previous commands, apply them so we may make a more educated guess
    MotionSample delaySample;
    // double totalDelay = GetControlDelay(0);
    // if(totalDelay > 0 && m_pFunctor->GetPreviousCommand().size() != 0){
    //     CommandList::iterator it  = m_pFunctor->GetPreviousCommand().begin();
    //     while(totalDelay > 0 && it != m_pFunctor->GetPreviousCommand().end()){
    //         delaySample.m_vCommands.insert(delaySample.m_vCommands.begin(),(*it));
    //         //delaySample.m_vCommands.push_back((*it)  );
    //         totalDelay -= (*it).m_dT;
    //         ++it;
    //     }
    //     m_pFunctor->ResetPreviousCommands();
    //     m_pFunctor->SetNoDelay(true);
    //     m_pFunctor->ApplyVelocities(m_StartState,delaySample,0,true);
    //     //and now set the starting state to this new value
    //     m_StartState = delaySample.m_vStates.back();
    // }

    //regardless of the delay, for local planning we always want to proceed with no delay and with no previous commands
    //as the previous section should take care of that
    // m_pFunctor->ResetPreviousCommands();
    // SetNoDelay(true);
    m_bNoDelay = true;

    Sophus::SE3d dTranslation(Sophus::SO3d(),-m_StartState.m_dTwv.translation());

    //first translate to base everything around dStartPose
    Sophus::SE3d dFixedStart = m_StartState.m_dTwv;
    Sophus::SE3d dFixedGoal = m_GoalState.m_dTwv;

    //now rotate everything to a frame inbetween the two
    Eigen::Quaternion<double> dStartQuat = m_StartState.m_dTwv.so3().unit_quaternion();
    Eigen::Quaternion<double> dEndQuat = m_GoalState.m_dTwv.so3().unit_quaternion();
    //find the halfway rotation
    Eigen::Quaternion<double> dMidQuat = dStartQuat.slerp(0.5,dEndQuat);
    //dMidQuat.setIdentity();

    //now rotate both waypoints into this intermediate frame
    Sophus::SE3d dRotation(Sophus::SO3d(dMidQuat.toRotationMatrix().transpose()),Eigen::Vector3d::Zero()); //we want the inverse rotation here
    if(bFlatten2Dcurves){
        dRotation.so3() = Sophus::SO3d();
    }

    // ROS_INFO("sr %f %f %f %f", dStartQuat.w(), dStartQuat.x(), dStartQuat.y(), dStartQuat.z());
    // ROS_INFO("er %f %f %f %f", dEndQuat.w(), dEndQuat.x(), dEndQuat.y(), dEndQuat.z());
    // ROS_INFO("t %f %f %f", dTranslation.translation().x(), dTranslation.translation().y(), dTranslation.translation().z());
    // ROS_INFO("r %f %f %f %f", dRotation.unit_quaternion().w(), dRotation.unit_quaternion().x(), dRotation.unit_quaternion().y(), dRotation.unit_quaternion().z());
    
    m_dT3d = (dRotation*dTranslation);
    m_dT3dInv = m_dT3d.inverse();

    //now rotate both points;
    dFixedStart = m_dT3d * dFixedStart;
    dFixedGoal = m_dT3d * dFixedGoal;

    VehicleState dStartStateFixed = m_StartState;
    VehicleState dGoalStateFixed = m_GoalState;
    dStartStateFixed.m_dTwv = dFixedStart;
    dGoalStateFixed.m_dTwv = dFixedGoal;
    //and now we can project to 2d and get our 2d planner points
    Eigen::Vector6d dStartPose2D = dStartStateFixed.ToXYZTCV(); // x y z t c v
    Eigen::Vector6d dGoalPose2D = dGoalStateFixed.ToXYZTCV();

    //make sure we don't have start and end velocities of zero
    if(dStartPose2D[5] == 0 && dGoalPose2D[5] == 0){
        return false;
    }

    m_dStartTime = dStartTime;

    //setup the boundary problem
    m_dStartPose = dStartPose2D;
    m_dGoalPose = dGoalPose2D;
    m_pBoundarySovler = &m_BoundarySolver;

    m_dTinv = rpg::TInv( rpg::Cart2T( m_dStartPose[0], m_dStartPose[1], m_dStartPose[3] ));  // x y t
    Eigen::Vector6d dTemp = _TransformGoalPose(dStartPose2D); // x y z t c v
    m_BoundaryProblem.m_dStartPose = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]); // x y t c
    dTemp  = _TransformGoalPose(dGoalPose2D);
    m_BoundaryProblem.m_dGoalPose = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    m_BoundaryProblem.m_nDiscretization = 50;

    if(pVelProfile == NULL){
        m_vVelProfile.clear();
        m_vVelProfile.reserve(2);
        m_vVelProfile.push_back(VelocityProfileNode(0, double(m_dStartPose[5])));
        m_vVelProfile.push_back(VelocityProfileNode(1, double(m_dGoalPose[5])));
    }else{
        m_vVelProfile = *pVelProfile;
    }

    //this puts the result in m_dP
    // ROS_INFO("Solving 2D problem.");
    ROS_DBG("Solving boundary problem.");
    m_pBoundarySovler->Solve(&m_BoundaryProblem);
    _GetAccelerationProfile();

    //initialize the max time
    m_dMaxSegmentTime = m_dSegmentTime*5;

    //reset optimization parameters
    m_CurrentSolution.m_dNorm = DBL_MAX;
    m_bInLocalMinimum = false;
    m_CurrentSolution.m_dOptParams.head(3) = m_BoundaryProblem.m_dGoalPose.head(3); //  x y t 
    m_CurrentSolution.m_dOptParams[OPT_ACCEL_DIM] = 0.0; // this is the percentage of the calculated acceleration
    if(OPT_DIM > OPT_AGGR_DIM){
        m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM] = m_BoundaryProblem.m_dAggressiveness;
    }
    m_dInitOptParams = m_CurrentSolution.m_dOptParams;
    m_pBestSolution = &m_CurrentSolution;

    //the transformed final position
    m_dTransformedGoal = _TransformGoalPose(m_dGoalPose);

    //restore the original start state for simulation purposes
    //m_StartState = originalStartState;
    m_eCostMode = eCostMode;

    return true;
}

///////////////////////////////////////////////////////////////////////
bool MochaProblem::Iterate()
{
    ROS_INFO("Iterating problem %s id %d", m_problemName.c_str(), m_nPlanId);
    try
    {
                //get the current state and norm for the first iteration
        if(m_lSolutions.size() == 0 ) {
            //here we have to re-evaluate the segment time of the trajectory

            SimulateTrajectory(m_CurrentSolution.m_Sample,0,false);
            //m_dDistanceDelta = m_pCurrentMotionSample->GetDistance() - m_BoundaryProblem->m_dDistance;
            if(m_bInertialControlActive){
                CalculateTorqueCoefficients(m_CurrentSolution.m_Sample);
                SimulateTrajectory(m_CurrentSolution.m_Sample,0,false);
            }

            // ROS_INFO("iterating world %d norm %f", 0, _CalculateErrorNorm(_CalculateSampleError()));

            //calculate the distance
            double dMinLookahead;
            Eigen::VectorXd error = _CalculateSampleError(dMinLookahead);
            //DLOG(INFO) << m_nPlanId << ":Initial error is " << error.transpose();
            m_CurrentSolution.m_dNorm = _CalculateErrorNorm(error);
            if(std::isfinite(m_CurrentSolution.m_dNorm) == false){
                DLOG(INFO) << m_nPlanId << ":Initial norm is not finite. Exiting optimization";
                return false;
            }
            m_lSolutions.push_back(m_CurrentSolution);
            m_pBestSolution = &m_lSolutions.back();
        }

        if( m_dEps > 5 || m_bInLocalMinimum == true) {
            DLOG(INFO) << "Failed to plan. Norm = " << m_CurrentSolution.m_dNorm;
            return false;
        }

        m_vPotentialPaths.push_back(this->m_CurrentSolution.m_Sample);

        _IterateGaussNewton();

        return false;
    }catch(...){
        return false;
    }
}

void MochaProblem::CalculateTorqueCoefficients(MotionSample& pSample)
{
    ROS_DBG("Calculating torque coefficients.");
    //find the last air section and record its duration
    double dAirTime = 0;
    int nStartIndex = -1;
    double dStartTime = 0;

    double dLongestAirTime = 0, dLongestStartTime = 0;
    int nLongestStartIndex = -1;

    //find the longest airborne segment in the trajectory
    for(int ii = std::min(pSample.m_vStates.size()-1,
                          pSample.m_vCommands.size()-1) ;
            ii>=0 ; --ii){
        if(pSample.m_vStates[ii].IsAirborne()){
            dAirTime += pSample.m_vCommands[ii].m_dT;
            nStartIndex = ii;
            dStartTime = pSample.m_vStates[ii].m_dTime;
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
        ROS_DBG("Got air time! Calculating torques.");
        //find the goal state based on the cost type
        Sophus::SO3d dGoalState;
        if(m_eCostMode == eCostPoint){
            dGoalState = m_GoalState.m_dTwv.so3();
        }else{
            dGoalState = m_GoalState.m_dTwv.so3();

            bool bCurrentAirborne = m_Trajectory.m_vStates[0].IsAirborne();
            for(size_t ii = 1 ; ii < m_Trajectory.m_vStates.size() ; ii++){
                bool bTemp = m_Trajectory.m_vStates[ii].IsAirborne();
                if(bTemp == false && bCurrentAirborne == true){
                    dGoalState = m_Trajectory.m_vStates[ii].m_dTwv.so3();
                    break;
                }
                bCurrentAirborne = bTemp;
            }
        }
        //calculate the change in angles necessary
        const Sophus::SO3d Rw_dest = m_GoalState.m_dTwv.so3();
        const Sophus::SO3d Rw_init = pSample.m_vStates[nStartIndex].m_dTwv.so3();

        //const Sophus::SO3d Rinit_w = Rw_init.inverse();
        //const Sophus::SO3d Rinit_dest = Rinit_w * Rw_dest;
        //angle in body frame
        const Eigen::Vector3d angles(0,
                                     rpg::AngleWrap(rpg::R2Cart(Rw_dest.matrix())[1]-rpg::R2Cart(Rw_init.matrix())[1]),// - (m_eCostMode == eCostPoint ? M_PI*2 : 0),
                                     0);// = Rinit_dest.log();

        // const Eigen::Vector3d dInertia = m_pFunctor->GetCarModel()->GetVehicleInertiaTensor(0);
        const Eigen::Vector3d dInertia = Eigen::Vector3d(0,0,0);//GetInertiaTensor(0);

        //Eigen::Vector3d currentV = pSample.m_vStates[nStartIndex].m_dV;
        const Sophus::SO3d Rwv = pSample.m_vStates[nStartIndex].m_dTwv.so3();
        //angular velocities in body frame
        //create the inertia tensor
//        Eigen::Matrix3d omega_w;
//        omega_w << 0, -pSample.m_vStates[nStartIndex].m_dW[2], pSample.m_vStates[nStartIndex].m_dW[1],
//                   pSample.m_vStates[nStartIndex].m_dW[2],0,-pSample.m_vStates[nStartIndex].m_dW[0],
//                   -pSample.m_vStates[nStartIndex].m_dW[1],pSample.m_vStates[nStartIndex].m_dW[0],0;
        //DLOG(INFO) << "omega " << omega_w;

        //Sophus::SO3d dRwv = Sophus::SO3d::hat(pSample.m_vStates[nStartIndex].m_dW);
        //const Eigen::Vector3d omega_v =  Sophus::SO3d::vee((Rwv.inverse()*dRwv).matrix());
        const Eigen::Vector3d omega_v = Rwv.inverse() * pSample.m_vStates[nStartIndex].m_dW;

        //calculate the coefficients
        Eigen::Matrix4d A;
        Eigen::Vector4d B;

        A << 1, 0              ,0                     ,0,
                1, dAirTime, powi(dAirTime,2), powi(dAirTime,3),
                dAirTime, powi(dAirTime,2)/2,powi(dAirTime,3)/3,powi(dAirTime,4)/4,
                powi(dAirTime,2)/2,  powi(dAirTime,3)/6,powi(dAirTime,4)/12,powi(dAirTime,5)/20;

            Eigen::Matrix4d Ainertia = A / dInertia(1);
            B << m_dStartTorques(1),0, -omega_v(1), angles(1) - omega_v(1)*dAirTime;
            m_dCoefs = Ainertia.inverse() * B;
            m_dTorqueStartTime = dStartTime;
            m_dTorqueDuration = dAirTime;
    }else{
        m_dTorqueStartTime = -1;
    }
}

// void MochaProblem::StressTest(MochaProblem &problem)
// {
//    std::vector<std::shared_ptr<boost::thread> > threads;
//    for(int jj = 0; jj < 1000 ; jj++){
//        Eigen::Vector6d pHypeStates[DAMPING_STEPS];
//        Eigen::VectorXd pHypeErrors[DAMPING_STEPS];
//        std::vector<std::shared_ptr<MochaProblem> > vCubicProblems;
//        std::vector<std::shared_ptr<ApplyCommandsThreadFunctor> >vFunctors;
//        vCubicProblems.resize(DAMPING_STEPS);
//        vFunctors.resize(DAMPING_STEPS);

////        vCubicProblems[0] = std::make_shared<MochaProblem>(problem);
////        vFunctors[0] = std::make_shared<ApplyCommandsThreadFunctor>(this,*vCubicProblems[0],0,pHypeStates[0],m_DampingMotionSamples[0],true);
////        vFunctors[0]->operator()();

//        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
//            vCubicProblems[ii] = std::make_shared<MochaProblem>(problem);
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
// }


///////////////////////////////////////////////////////////////////////
bool MochaProblem::_IterateGaussNewton()
{
    ROS_DBG("Iterating Gauss Newton.");

    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd dDeltaP;
    //        DLOG(INFO) << "Entered gauss-newton search with e = " << m_dCurrentEps;
    double bestDamping;
    unsigned int nBestDampingIdx = 0;

    Eigen::VectorXd dWvec = _GetWeightVector();

    Eigen::MatrixXd J = Eigen::MatrixXd(dWvec.rows(),OPT_DIM);
    //create an appropriate weighting matrix
    Eigen::MatrixXd dW = dWvec.asDiagonal();

    //double dMinLookahead;
    Eigen::VectorXd error;// = _CalculateSampleError(problem,dMinLookahead);

    //DLOG(INFO) << m_nPlanId << ":Iteration error is" << error.transpose();
    Solution coordinateDescent;
    if(_CalculateJacobian(error,coordinateDescent,J) == false){
        return false;
    }

    // ROS_INFO("Errors:");
    // for (uint i=0; i<error.size(); i++)
    // {
    //     ROS_INFO("%f",error[i]);
    // }

    // m_CurrentSolution.m_dNorm = _CalculateErrorNorm(problem,error);
    // if(g_bVerbose){
    //     DLOG(INFO) << "Calculated jacobian with base norm: " << m_CurrentSolution.m_dNorm;
    // }
    // ROS_INFO("Calculated jacobian with base norm %f", m_CurrentSolution.m_dNorm);

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
    ROS_DBG("Solving for delta P.");
    (JtJ).llt().solveInPlace(dDeltaP);

    //this is a hack for now, but it should take care of large deltas
    if(dDeltaP.norm() > 100 ){
        ROS_WARN("Large delta P detected. Hacking...");
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
        dDeltaP = J.transpose()*dW*error;
        (JtJ).llt().solveInPlace(dDeltaP);
    }

    // if(g_bVerbose){
    //     DLOG(INFO) << "Gauss newton delta: [" << dDeltaP.transpose().format(CleanFmt) << "]";
    // }
    // ROS_INFO("Gauss newton delta [%s]", convertEigenMatrix2String(dDeltaP.transpose()).c_str());


    if(std::isfinite(dDeltaP.norm()) == false){
        ROS_ERROR("%f:Deltas are NAN. Dump => \nJ: %s\nb: %s",m_nPlanId, J.format(CleanFmt), error.transpose().format(CleanFmt));
        m_eError = eDeltaNan;
        return false;
    }

    ROS_INFO("Finished CD with norm %f, opts [%s]", coordinateDescent.m_dNorm, convertEigenMatrix2String(coordinateDescent.m_dOptParams.transpose()).c_str());

    //if no damping is required, just pick the result of the gauss newton
    if(g_bDisableDamping == true){
//        m_CurrentSolution.m_dOptParams.head(OPT_DIM) += dDeltaP.head(OPT_DIM);
//        m_BoundaryProblem->m_dGoalPose.head(3) = m_CurrentSolution.m_dOptParams.head(3);
//        m_BoundaryProblem->m_dAggressiveness = m_CurrentSolution.m_dOptParams[OPT_AGGR_DIM];
        UpdateOptParams(m_CurrentSolution.m_dOptParams.head(OPT_DIM) + dDeltaP);
        ROS_DBG("Solving boundary problem.");
        m_pBoundarySovler->Solve(&m_BoundaryProblem);
        SimulateTrajectory(m_CurrentSolution.m_Sample);
        m_CurrentSolution.m_dNorm = _CalculateErrorNorm(_CalculateSampleError());
        m_lSolutions.push_back(m_CurrentSolution);
        m_pBestSolution = &m_lSolutions.back();
        // if (m_lSolutions.back().m_dNorm < m_pBestSolution->m_dNorm)
        // {
        //     m_pBestSolution = &m_lSolutions.back();
        // }
        // DLOG(INFO) << "Iteration with no damping finished with norm " << m_CurrentSolution.m_dNorm << " and opts "<< m_CurrentSolution.m_dOptParams.transpose().format(CleanFmt);
        ROS_DBG("Finished Gauss Newton iteration without dampening.");
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
    std::vector<std::shared_ptr<MochaProblem> > vCubicProblems;
    std::vector<std::shared_ptr<ApplyCommandsThreadFunctor> >vFunctors;
    vCubicProblems.resize(DAMPING_STEPS);
    vFunctors.resize(DAMPING_STEPS);
    double dampings[DAMPING_STEPS];
    damping = 1.0;

    ROS_INFO("Calculating damped soln on %d threads", DAMPING_STEPS);

    Solution dampedSolution;
    dampedSolution.m_dNorm = DBL_MAX;
    if(std::isfinite(dDeltaP[0]) ){
        //create the damped problems and run the thread queue to sample them
        double t0 = Tic();
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            dampings[ii] = damping;
            Eigen::VectorXd delta = dDeltaP *damping;
            vCubicProblems[ii] = std::make_shared<MochaProblem>(*this);
            vCubicProblems[ii]->UpdateOptParams(vCubicProblems[ii]->m_CurrentSolution.m_dOptParams.head(OPT_DIM)+delta);
            vFunctors[ii] = std::make_shared<ApplyCommandsThreadFunctor>(vCubicProblems[ii],
                                                                         ii,
                                                                         pDampingStates[ii],
                                                                         pDampingErrors[ii],
                                                                         vCubicProblems[ii]->m_CurrentSolution.m_Sample,
                                                                         true);
            m_ThreadPool.schedule(*vFunctors[ii]);
            damping/= DAMPING_DIVISOR;
        }
        m_ThreadPool.wait();
        ROS_DBG("Applied commands, took %fs.", Toc(t0));

        // ROS_INFO("Pubbing potential paths viz from Damped Solution.");
        m_vPotentialPaths.push_back(this->m_CurrentSolution.m_Sample);
        for( int ii = 0; ii < DAMPING_STEPS ; ii++ )
        {
            m_vPotentialPaths.push_back(vCubicProblems[ii]->m_CurrentSolution.m_Sample);
        }
        // pubPotentialPathsViz();

        // if(g_bVerbose){
        //     DLOG(INFO) << "Damping norms are: [";
        // }
        //pick out the best damping by comparing the norms
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            double norm = 0;
            // dout_cond("Final state is " << pDampingStates[ii].transpose().format(CleanFmt),g_bVerbose);
            norm = vCubicProblems[ii]->_CalculateErrorNorm(pDampingErrors[ii]);
            // if(g_bVerbose){
            //     DLOG(INFO) << " " << norm;
            // }
            // ROS_INFO("Dampened solution step %d, norm %f, dampening %f, final state [%s], errors [%s]", ii, norm, dampings[ii], convertEigenMatrix2String(pDampingStates[ii].transpose()).c_str(), convertEigenMatrix2String(pDampingErrors[ii].transpose()).c_str());
            if(norm < dampedSolution.m_dNorm ) {
                // ROS_INFO("New best damped solution with norm %f.", norm);
                dampedSolution = vCubicProblems[ii]->m_CurrentSolution;
                dampedSolution.m_dNorm = norm;
                bestDamping = dampings[ii];
                nBestDampingIdx = ii;
            }
        }

        // if(g_bVerbose){
        //     DLOG(INFO) << "]" ;
        // }

    }else{
        Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
        ROS_ERROR("%d:Deltas are NAN. Dump => J:%s", m_nPlanId, J.format(CleanFmt));
        m_eError = eDeltaNan;
        return false;
    }

    ROS_INFO("Finished Damping with norm %f, opts [%s]", dampedSolution.m_dNorm, convertEigenMatrix2String(dampedSolution.m_dOptParams.transpose()).c_str());

    Solution newSolution;
    Eigen::VectorXd bestSolutionErrors;
    // if(coordinateDescent.m_dNorm > m_CurrentSolution.m_dNorm && g_bMonotonicCost)
    // {
    //     m_bInLocalMinimum = true;
    //     ROS_WARN("Local minimum detected in coordinate descent solution.");
    // }
    // else if ( dampedSolution.m_dNorm > m_CurrentSolution.m_dNorm && g_bMonotonicCost) {
    //     m_bInLocalMinimum = true;
    //     ROS_WARN("Local minimum detected in damped solution.");
    // }
    // else
    // {
    //     newSolution = dampedSolution;
    //     bestSolutionErrors = pDampingErrors[nBestDampingIdx];
    // }
    if(coordinateDescent.m_dNorm > m_CurrentSolution.m_dNorm && dampedSolution.m_dNorm > m_CurrentSolution.m_dNorm && g_bMonotonicCost)
    {
        m_bInLocalMinimum = true;
        ROS_WARN("Local minimum detected.");
    }
    else
    {
        if (coordinateDescent.m_dNorm < dampedSolution.m_dNorm)
        {
            ROS_INFO("Setting newSoln to CD norm %f", coordinateDescent.m_dNorm);
            newSolution = coordinateDescent;
            bestSolutionErrors = error;
        }
        else
        {
            ROS_INFO("Setting newSoln to Damp norm %f", dampedSolution.m_dNorm);
            newSolution = dampedSolution;
            bestSolutionErrors = pDampingErrors[nBestDampingIdx];
        }
    }

    //update the problem params
    //m_CurrentSolution.m_dOptParams = newSolution.m_dOptParams;
    //update the best solution if necessary
    if(m_bInLocalMinimum == false){
        if(newSolution.m_dNorm < m_CurrentSolution.m_dNorm){
            // ROS_INFO("newSoln w norm %f better than currSoln w norm %f", newSolution.m_dNorm, m_CurrentSolution.m_dNorm);
            // ROS_DBG("Updating best solution for plan %d with norm %f.", m_nPlanId, newSolution.m_dNorm);
            //add this solution to the list
            m_lSolutions.push_back(newSolution);
            if (newSolution.m_dNorm < m_pBestSolution->m_dNorm)
            {
                // ROS_INFO("Updating bestSoln w old norm %f to new w %f", m_pBestSolution->m_dNorm, m_lSolutions.back().m_dNorm);
                m_pBestSolution = &m_lSolutions.back();
            }
            // m_vBestSolutionErrors = bestSolutionErrors;
            // m_vBestSolutionErrors = _CalculateSampleError(m_Sample, m_Problem, m_m_CurrentSolution.m_dMinTrajectoryTime);
        }
        // ROS_INFO("Setting currSoln w old norm %f to newSoln w norm %f", m_CurrentSolution.m_dNorm, newSolution.m_dNorm);
        m_CurrentSolution = newSolution;
    }
    else
    {
        // ROS_WARN("Local minimum detected.");
    }

    // ROS_INFO("bestSoln is w norm %f", m_pBestSolution->m_dNorm);
    // for (uint i=0; i<m_pBestSolution->m_Sample.m_vStates.size(); i+=5)
    // {
    //     ROS_INFO("  %d %f %f %f %f %f", i,
    //         m_pBestSolution->m_Sample.m_vStates[i].ToXYZQuat()[0],
    //         m_pBestSolution->m_Sample.m_vStates[i].ToXYZQuat()[1],
    //         m_pBestSolution->m_Sample.m_vCommands[i].m_dForce,
    //         m_pBestSolution->m_Sample.m_vCommands[i].m_dPhi,
    //         m_pBestSolution->m_Sample.m_vCommands[i].m_dCurvature); 
    // }
    
    //m_CurrentSolution.m_Sample = newSolution.m_Sample;


    // ROS_INFO("Finished GN dampening for plan %d with coordinated descent solution, final state [%s], errors [%s]", m_nPlanId, newSolution.m_dNorm, convertEigenMatrix2String(pDampingStates[ii].transpose()).c_str(), convertEigenMatrix2String(pDampingErrors[ii].transpose()).c_str());

    UpdateOptParams(m_CurrentSolution.m_dOptParams.head(OPT_DIM));
    ROS_DBG("Solving boundary problem.");
    m_pBoundarySovler->Solve(&m_BoundaryProblem);

//    if(m_pCurrentMotionSample == NULL) {
//        assert(false);
//    }
    m_eError = eSuccess;
    return true;
}
