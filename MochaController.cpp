#include "MochaController.h"

// class MochaController;

/////////////////////////////////////////////////////////////////////////////////////////
MochaController::MochaController(ros::NodeHandle& private_nh_, ros::NodeHandle& nh_) :
    m_private_nh(private_nh_),
    m_nh(nh_),
    m_dTrajWeight(Eigen::MatrixXd(1,1)),
    m_dControlRate(10),
    m_dMaxControlPlanTime(0.2),
    m_dLookaheadTime(1.0),
    m_pControlPlannerThread(NULL),
    m_ControllerState(IDLE),
    m_dt(0.01),
    m_map_frame("map")
    // m_bControllerRunning(false)
{
    m_lControlPlans.clear();
    m_dLastDelta.setZero();
    //m_vControlPlans.reserve(10);

    m_dTrajWeight = Eigen::MatrixXd(TRAJ_EXTRA_ERROR_TERMS+TRAJ_UNIT_ERROR_TERMS,1);
    m_dTrajWeight.setIdentity();
    m_dTrajWeight(0) = X_WEIGHT;
    m_dTrajWeight(1) = Y_WEIGHT;
    m_dTrajWeight(2) = Z_WEIGHT;
    m_dTrajWeight(3) = THETA_WEIGHT;
    m_dTrajWeight(4) = VEL_WEIGHT_TRAJ;
    m_dTrajWeight(5) = TIME_WEIGHT;
    m_dTrajWeight(6) = CURV_WEIGHT;

    m_private_nh.param("map_frame", m_map_frame, m_map_frame);

    m_timerControlLoop = m_private_nh.createTimer(ros::Duration(1.0/m_dControlRate), &MochaController::ControlLoopFunc, this);
    // m_timerControlLoop = m_private_nh.createTimer(ros::Duration(1.0/m_dControlRate), boost::bind(&MochaController::ControlLoopFunc, this, _1));

    m_subVehicleState = m_nh.subscribe("controller/state", 1, &MochaController::vehicleStateCb, this);

    m_subPlan = m_nh.subscribe("planner/plan", 1, &MochaController::planCb, this);

    m_pubCommands = m_nh.advertise<carplanner_msgs::Command>("controller/command",1);

    m_pubLookaheadTraj = m_nh.advertise<nav_msgs::Path>("controller/lookahead_traj",1);
    // m_pubLookaheadTraj = m_nh.advertise<carplanner_msgs::MotionSample>("controller/lookahead",1);

    ROS_INFO("Controller constructed.");

    InitController();
}

// void MochaController::dynReconfigCb(carplanner_msgs::MochaConfig &config, uint32_t level)
// {
//     ROS_INFO("Reconfigure requested.");

//     m_dt = config.dt;
// }

/////////////////////////////////////////////////////////////////////////////////////////
void MochaController::InitController( ) {
    // m_vSegmentSamples = segmentSamples;
    // m_pModel = pModel;
    // m_pPlanner = pPlanner;
    // m_bStopping = false;
    // m_bStarted = false;
    m_bFirstPose = true;
    // m_bStillRun = true;
    m_bPoseUpdated = false;


    // m_nh.param("dt",               m_dt  , m_dt  );

    // m_dynReconfig_server.setCallback(boost::bind(&MochaController::dynReconfigCb, this, _1, _2));

    ROS_INFO("Controller initialized.");
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaController::Reset()
{
    {
        boost::mutex::scoped_lock(m_PlanMutex);
        while(m_lControlPlans.begin() != m_lControlPlans.end()) {
            //delete this plan
            delete(m_lControlPlans.front());
            m_lControlPlans.erase(m_lControlPlans.begin());
        }
    }

    m_LastCommand = ControlCommand();
    m_bFirstPose = true;
    // m_bStopping = false;
    // m_bStarted = false;
    ROS_INFO("Controller reset.");
}

/////////////////////////////////////////////////////////////////////////////////////////
bool MochaController::_SampleControlPlan(ControlPlan* pPlan, MochaProblem& problem)
{
    ROS_INFO("Sampling control plan.");
    //get the motion sample for the new control plan
    if(g_bOptimize2DOnly == true){
        pPlan->m_Sample.m_vCommands = m_MotionSample2dOnly.m_vCommands;
    }else{
        pPlan->m_Sample.m_vCommands = problem.m_pBestSolution->m_Sample.m_vCommands;
    }

    if(g_bShow2DResult) {
        //ONLY FOR VISUALIZATION. REMOVE WHEN NO LONGER NEEDED
       Eigen::Vector3dAlignedVec samples;

        problem.SamplePath(samples,true);
        pPlan->m_Sample.m_vStates.reserve(samples.size());
        for(const Eigen::Vector3d& pos : samples){
            Sophus::SE3d Twv(Sophus::SO3d(),pos);
            pPlan->m_Sample.m_vStates.push_back(VehicleState(Twv,0));
        }
    }else{
        ApplyVelocities(pPlan->m_StartState,
                                            pPlan->m_Sample,
                                            0,
                                            true);
        //if we are in the air, make sure no force is applied and the wheels are straight
        for(size_t ii = 0 ; ii < pPlan->m_Sample.m_vStates.size() ; ii++){
            if(pPlan->m_Sample.m_vStates[ii].IsAirborne()){
                if(g_bFrontFlip){
                    pPlan->m_Sample.m_vCommands[ii].m_dForce = 0;
                }else{
                    pPlan->m_Sample.m_vCommands[ii].m_dForce = 0;// + problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::AccelOffset]*SERVO_RANGE;
                }
                pPlan->m_Sample.m_vCommands[ii].m_dPhi = 0;// + problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::SteeringOffset]*SERVO_RANGE;
            }
        }
    }


    //get the plan times in order by offsetting them by the start time
    for(VehicleState& state: pPlan->m_Sample.m_vStates) {
        state.m_dTime += pPlan->m_dStartTime;
    }

    for(ControlCommand& command: pPlan->m_Sample.m_vCommands) {
        command.m_dTime += pPlan->m_dStartTime;
    }

    if(pPlan->m_Sample.m_vCommands.empty()) {
        ROS_ERROR("Empty control plan discovered. Skipping.");
        return false;
    }

    pPlan->m_dEndTime = pPlan->m_Sample.m_vStates.back().m_dTime;
    //set the norm on the plan
    pPlan->m_dNorm = problem.m_CurrentSolution.m_dNorm;

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool MochaController::_SolveControlPlan(const ControlPlan* pPlan,MochaProblem& problem,const MotionSample& trajectory)
{
    ROS_INFO("Solving control plan.");
    bool res = problem.Initialize(pPlan->m_dStartTime,&problem.m_vVelProfile,g_bPointCost ? eCostPoint : eCostTrajectory);
    //double dLastDeltaNorm = m_dLastDelta.norm();
//    if(std::isfinite(dLastDeltaNorm) && dLastDeltaNorm < 1.0 ){
//        problem.m_CurrentSolution.m_dOptParams += m_dLastDelta;
//        problem.m_BoundaryProblem.m_dGoalPose.head(3) = problem.m_CurrentSolution.m_dOptParams.head(3);
//        problem.m_pBoundarySovler->Solve(&problem.m_BoundaryProblem);
//    }else{
//        m_dLastDelta.setZero();
//    }

    problem.m_bInertialControlActive = g_bInertialControl;
    problem.m_Trajectory = trajectory;

    if( res == false ){
        ROS_ERROR("2d planner failed to converge. Skipping");
        return false;
    }

    res = true;

    boost::timer::cpu_timer timer;
    while(1)
    {
        //make sure the plan is not fully airborne
        //bool isAirborne = (pPlan->m_StartState.IsAirborne() && pPlan->m_GoalState.IsAirborne());
        if(g_bOptimize2DOnly /*|| isAirborne*/) {
            ROS_INFO("Simulating control trajectory.");
            problem.SimulateTrajectory(m_MotionSample2dOnly,0,true);
            break;
        }else{
            ROS_INFO("Iterating control trajectory.");
            if( (problem.Iterate()) == true ) {
                break;
            }
        }

        if(m_bStopping){
            res = false;
        }
        boost::timer::cpu_times const elapsed_times(timer.elapsed());
        boost::timer::nanosecond_type const elapsed(elapsed_times.system+ elapsed_times.user);
        //time elapsed is in nanoseconds, hence the 1e9
        if(g_bInfiniteTime){
            if(elapsed > 1e9*g_dMaxPlanTimeLimit){
                break;
            }
        }else{
            if(elapsed > 1e9*(m_dMaxControlPlanTime*m_dLookaheadTime)){
                break;
            }
        }
    }

    //and now obtain the new delta
    m_dLastDelta = problem.m_CurrentSolution.m_dOptParams - problem.m_dInitOptParams;

    if(problem.m_CurrentSolution.m_dNorm > g_dMaxPlanNorm){
        // DLOG(ERROR) << "Planned control plan with norm too high -> " << problem.m_CurrentSolution.m_dNorm;
        ROS_WARN_THROTTLE(.1,"Planned control plan with norm too high -> %d", problem.m_CurrentSolution.m_dNorm);
        res = false;
    }

    return res;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool MochaController::PlanControl(double dPlanStartTime, ControlPlan*& pPlanOut) 
{
    ROS_INFO("Planning control at %fs", dPlanStartTime);
    try
    {
        pPlanOut = NULL;
        int nCurrentSampleIndex;
        double interpolationAmount;
        double planStartCurvature;
        Eigen::Vector3d planStartTorques = Eigen::Vector3d::Zero();
        PlanPtrList::iterator nCurrentPlanIndex;
        ControlPlan* pPlan = NULL;

        //reset the starting position
        int oldPlanStartSegment = 0;
        int oldPlanStartSample = 0;
        int sampleCount = 0;

        //only continue planning if the pose has been updated since the last plan
        {
            boost::mutex::scoped_lock lock(m_PoseMutex);
            if(m_bPoseUpdated == false) {
                //DLOG(ERROR) << "Pose not updated, exiting control.";
                ROS_WARN("Pose not updated. Skipping.");
                return false;
            }
            /*else{
                m_bPoseUpdated = false;
            }*/
        }

        pPlan = new ControlPlan();

        {
            boost::mutex::scoped_lock lock(m_PlanMutex);

            //first find out where we are on the current plan
            _GetCurrentPlanIndex(dPlanStartTime,nCurrentPlanIndex,nCurrentSampleIndex,interpolationAmount);
            // ROS_INFO("Controller got current plan index at %f plan idx %d, sample idx %d interp amt %f", dPlanStartTime, distance(m_lControlPlans.begin(),nCurrentPlanIndex), nCurrentSampleIndex, interpolationAmount);

            if(nCurrentPlanIndex == m_lControlPlans.end() )
            {
                // if we have overshot all plans, clear
                ROS_WARN("Controller overshot plans, clearing control plans.");
                while(m_lControlPlans.begin() != m_lControlPlans.end() )
                {
                    delete(m_lControlPlans.front());
                    m_lControlPlans.erase(m_lControlPlans.begin());
                }
            }
            else
            {
                if(nCurrentPlanIndex != m_lControlPlans.begin() )
                {
                    ROS_INFO("Controller undershot plans, clearing unneccessary control plans.");
                    //remove all plans before the current plan
                    while(m_lControlPlans.begin() != nCurrentPlanIndex)
                    {
                        //delete this plan
                        delete(m_lControlPlans.front());
                        m_lControlPlans.erase(m_lControlPlans.begin());
                    }
                    //the active plan should now be the first plan
                    nCurrentPlanIndex = m_lControlPlans.begin();
                }
            }
        }


        VehicleState currentState;
        {
            boost::mutex::scoped_lock lock(m_PoseMutex);
            currentState = m_CurrentState;
        }

//        ApplyVelocitesFunctor5d compDelayFunctor(m_pModel,planStartTorques, &m_lCurrentCommands);
//        //compDelayFunctor.ResetPreviousCommands();
//        compDelayFunctor.SetNoDelay(false);

//        //push the state forward by the duration of the solver if we have commmands
//        MotionSample compDelaySample;
//        double commandTime = dPlanStartTime;
//        double maxTime =  dPlanStartTime + (m_dMaxControlPlanTime*m_dLookaheadTime);
//        while(commandTime < maxTime){
//            GetCurrentCommands(commandTime,m_LastCommand);
//            m_LastCommand.m_dT = std::min(m_dt,maxTime - commandTime);
//            m_lCurrentCommands.insert(m_lCurrentCommands.begin(),m_LastCommand);
//            compDelaySample.m_vCommands.push_back(m_LastCommand);
//            commandTime += m_dt;
//        }

//        if(compDelaySample.m_vCommands.size() > 0){
//            compDelayFunctor.ApplyVelocities(currentState,compDelaySample,0,true);
//            currentState = compDelaySample.m_vStates.back();
//        }

//        //also push forward the start time of this plan
//        dPlanStartTime += (m_dMaxControlPlanTime*m_dLookaheadTime);

        // ApplyVelocitesFunctor5d delayFunctor(m_pModel,planStartTorques, NULL);
        //push forward the start state if there are commands stacked up
        MotionSample delaySample;
        double totalDelay = 0.1;
        if(totalDelay > 0 && m_lCurrentCommands.size() != 0)
        {
            ROS_INFO("Controller pushing start state forward with %d commands.",m_lCurrentCommands.size());
            for(const ControlCommand& command: m_lCurrentCommands){
                if(totalDelay <= 0){
                    break;
                }
                ControlCommand delayCommand = command;
                delayCommand.m_dT = std::min(totalDelay,command.m_dT);
                delaySample.m_vCommands.insert(delaySample.m_vCommands.begin(),delayCommand);
                totalDelay -= delayCommand.m_dT;
            }
            //delayFunctor.ResetPreviousCommands();
            // delayFunctor.SetNoDelay(true);
            //this applyvelocities call has noCompensation set to true, as the commands
            //are from a previous plan which includes compensation
            ApplyVelocities(currentState,delaySample,0,true,true);
            //and now set the starting state to this new value
            pPlan->m_StartState = delaySample.m_vStates.back();
            m_LastCommand = delaySample.m_vCommands.back();
        }
        else
        {
            ROS_INFO("Controller init'ing undelayed start state.");
            Eigen::Vector3d targetVel;
            Sophus::SE3d targetPos;
            GetCurrentCommands(dPlanStartTime,m_LastCommand,targetVel,targetPos);
            pPlan->m_StartState = currentState;
        }

        planStartTorques = m_LastCommand.m_dTorque;
        planStartCurvature = m_LastCommand.m_dCurvature;

        //DLOG(INFO) << "Plan starting curvature: " << planStartCurvature;

        //double distanceToPath = 0;
        {
            boost::mutex::scoped_lock motionplanlock(m_MotionPlanMutex);
            //if we do not have a plan, create new one from our
            //current position
            if(m_lControlPlans.empty()){
                ROS_INFO("Controller init'ing new control plan.");
                //get the starting curvature of our current plan

                //set the start time as now
                pPlan->m_dStartTime = dPlanStartTime;
                pPlan->m_nStartSegmentIndex = oldPlanStartSegment;
                pPlan->m_nStartSampleIndex = oldPlanStartSample;

                //start by finding the closest segment to our current location
                if(m_bFirstPose)
                {
                    ROS_INFO("Controller init'ing first starting sample.");
                    //if this is the first pose, search everywhere for the car
                    oldPlanStartSegment = 0;
                    oldPlanStartSample = 0;
                    sampleCount = 0;
                    for(size_t jj = 0 ; jj < m_vSegmentSamples.size() ; jj++) {
                        sampleCount += m_vSegmentSamples[jj].m_vCommands.size();
                    }
                    m_bFirstPose = false;
                    AdjustStartingSample(m_vSegmentSamples,pPlan->m_StartState,pPlan->m_nStartSegmentIndex,pPlan->m_nStartSampleIndex,0,sampleCount);
                }else{
                    AdjustStartingSample(m_vSegmentSamples,pPlan->m_StartState,pPlan->m_nStartSegmentIndex,pPlan->m_nStartSampleIndex);
                }

            }else {
                if(nCurrentSampleIndex == -1) {
                    //if we have overshot the current plan, function must be called again to create a new plan
                    ROS_ERROR("Overshot plan.");
                    return false;
                }else {
                    ROS_INFO("Controller init'ing existing plan.");
                    //get the curvature at the end of the projection to have a smooth transition in steering
                    pPlan->m_dStartTime = dPlanStartTime;

                    pPlan->m_nStartSegmentIndex = (*nCurrentPlanIndex)->m_nStartSegmentIndex;
                    //push forward the index by the precalculated amount
                    pPlan->m_nStartSampleIndex = (*nCurrentPlanIndex)->m_nStartSampleIndex;// + nCurrentSampleIndex;
                    MotionSample::FixSampleIndexOverflow(m_vSegmentSamples, pPlan->m_nStartSegmentIndex, pPlan->m_nStartSampleIndex);

                    AdjustStartingSample(m_vSegmentSamples,pPlan->m_StartState, pPlan->m_nStartSegmentIndex, pPlan->m_nStartSampleIndex);
                }
            }
        }

        if(g_bForceZeroStartingCurvature == true){
            planStartCurvature = 0;
        }
        pPlan->m_StartState.m_dCurvature = planStartCurvature;

        MotionSample trajectorySample;
        VelocityProfile profile;
        //prepare the trajectory ahead
        MochaController::PrepareLookaheadTrajectory(m_vSegmentSamples, pPlan, profile, trajectorySample, g_dInitialLookaheadTime);
        // std::string str = "Lookahead traj:";
        // for(uint i=0; i<trajectorySample.m_vStates.size(); i++)
        // {
        //     str += "\n" + convertEigenMatrix2String(trajectorySample.m_vStates[i].ToXYZTCV().transpose());
        // }
        // ROS_INFO("%s",str.c_str());
        ROS_INFO("Pubbing Lookahead, frame %s", m_map_frame.c_str());
        PublishLookaheadTrajectory(trajectorySample);

        // SetNoDelay(true);
        MochaProblem problem("ControlProblem",pPlan->m_StartState,pPlan->m_GoalState,m_dt);
        problem.SetTrajWeights(m_dTrajWeight);
        problem.m_dStartTorques = planStartTorques;
        problem.m_CurrentSolution.m_dMinTrajectoryTime = g_dInitialLookaheadTime;
        problem.m_vVelProfile = profile;

        //solve the control plan
        if( _SolveControlPlan(pPlan,problem,trajectorySample) == false ) {
            //do not use the plan
            // DLOG(ERROR) << "Could not solve plan.";
            ROS_ERROR("Could not solve plan.");
            return false;
        }

        //only need to sample the planner if the plan is not airborne
        if( _SampleControlPlan(pPlan,problem) == false ) {
            ROS_ERROR("Failed to sample plan.");
            return false;
        }

        //DLOG(INFO) << "Plan start heading is " << pPlan->m_StartState.GetTheta() << " and goal heading is " << pPlan->m_GoalState.GetTheta() <<
        //     " and traj end heading is " << pPlan->m_Sample.m_vStates.back().GetTheta();

        //double controlDelay = problem.m_pFunctor->GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
        double newLookahead = std::max(std::min(problem.m_pBestSolution->m_dMinTrajectoryTime, g_dMaxLookaheadTime),g_dMinLookaheadTime);
        m_dLookaheadTime = g_dLookaheadEmaWeight*newLookahead + (1-g_dLookaheadEmaWeight)*m_dLookaheadTime;

        //DLOG(INFO) << "Planned control with norm " << problem.m_dCurrentNorm << " and starting curvature " << pPlan->m_StartState.m_dCurvature;
        pPlan->m_dStartPose = pPlan->m_StartState.m_dTwv;
        pPlan->m_dEndPose = pPlan->m_GoalState.m_dTwv;

        {
            boost::mutex::scoped_lock planlock(m_PlanMutex);
            pPlan->m_nPlanId = rand() % 10000;
            //DLOG(INFO) << "Created control plan id:" << pPlan->m_nPlanId << " with starting torques: " << planStartTorques.transpose() << "with norm " << m_pPlanner->GetCurrentNorm();
            ROS_INFO("Controller created new control plan id %d norm %f", pPlan->m_nPlanId, pPlan->m_dNorm);
            m_lControlPlans.push_back(pPlan);
        }

        //update the old plan segment and samples
        oldPlanStartSegment = pPlan->m_nStartSegmentIndex;
        oldPlanStartSample = pPlan->m_nStartSampleIndex;

        //make sure the pointer returned back is valid
        pPlanOut = pPlan;

        //do this so we create a new plan in the next iteration
        pPlan = NULL;

    }catch(...)
    {
        DLOG(ERROR) << "Exception caught while planning.";
        return false;
    }

    ROS_INFO("Planned control.");
    return true;
}

VehicleState MochaController::GetCurrentPose() {
    VehicleState poseOut;
    boost::mutex::scoped_lock lock(m_PoseMutex);
    poseOut = m_CurrentState;
    return poseOut;
}

void MochaController::PublishLookaheadTrajectory(MotionSample& sample)
{
    if (sample.m_vStates.size()==0) { return; }
    
    // carplanner_msgs::MotionSample sample_msg = sample.toROS();
    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(sample, &path_msg, m_map_frame);

    m_pubLookaheadTraj.publish(path_msg);
    ros::spinOnce();
}

/////////////////////////////////////////////////////////////////////////////////////////
// void MochaController::SetCurrentPoseFromCarModel(BulletCarModel* pModel, int nWorldId) {
//     boost::mutex::scoped_lock lock(m_PoseMutex);
//     //Sophus::SE3d oldTwv = m_CurrentState.m_dTwv;
//     pModel->GetVehicleState(0,m_CurrentState);
//     //remove the car offset from the car state
//     //m_CurrentState.m_dTwv.block<3,1>(0,3) += m_CurrentState.m_dTwv.block<3,1>(0,2)*CAR_HEIGHT_OFFSET;
//     GetCommandHistory(0,m_lCurrentCommands);
//     // m_bPoseUpdated = g_bFreezeControl ? false : true;
//     // m_bPoseUpdated = true;
// }

/////////////////////////////////////////////////////////////////////////////////////////
void MochaController::SetCurrentPose(VehicleState pose, CommandList* pCommandList /*= NULL*/) {
    boost::mutex::scoped_lock lock(m_PoseMutex);

    if( std::isfinite(pose.m_dV[0]) == false ){
        assert(false);
    }

    m_CurrentState = pose;

    if(pCommandList != NULL) {
        m_lCurrentCommands = *pCommandList;
    }

    m_bPoseUpdated = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
double MochaController::GetLastPlanStartTime()
{
    boost::mutex::scoped_lock lock(m_PlanMutex);
    if(m_lControlPlans.empty() == false){
        return m_lControlPlans.back()->m_dStartTime;
    }else{
        return -1;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaController::GetCurrentCommands(const double time,
                                       ControlCommand& command)
{
    Eigen::Vector3d targetVel;
    Sophus::SE3d dT_target;
    GetCurrentCommands(time,command,targetVel,dT_target);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaController::GetCurrentCommands(const double time,
                                       ControlCommand& command,
                                       Eigen::Vector3d& targetVel,
                                       Sophus::SE3d& dT_target)
{
    boost::mutex::scoped_lock planlock(m_PlanMutex);
    boost::mutex::scoped_lock motionplanlock(m_MotionPlanMutex);

    int nCurrentSampleIndex;
    PlanPtrList::iterator nCurrentPlanIndex;
    double interpolationAmount;

    _GetCurrentPlanIndex(time,nCurrentPlanIndex,nCurrentSampleIndex,interpolationAmount);

    if( nCurrentSampleIndex == -1 || nCurrentPlanIndex == m_lControlPlans.end() ) {

        //DLOG(INFO) << "GetCurrentCommands returning last commands a:" << m_dLastAccel << " c:" << m_dLastTurnRate << " t:" << m_dLastTorques.transpose();
        command.m_dForce = 0;
        command.m_dPhi = 0;
        command.m_dTorque = Eigen::Vector3d::Zero();//m_dLastTorques;

        //DLOG(INFO) << "Torque output of: [ " << torques.transpose() << "] from previous plan";
    }else {
        command.m_dForce = (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dForce + interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dForce;
        command.m_dPhi =   (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dPhi + interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dPhi;
        command.m_dCurvature = (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dCurvature + interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dCurvature;
        command.m_dTorque = (1-interpolationAmount) * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex].m_dTorque + interpolationAmount * (*nCurrentPlanIndex)->m_Sample.m_vCommands[nCurrentSampleIndex+1].m_dTorque;

        //DLOG(INFO) << "v: " << m_vSegmentSamples[(*nCurrentPlanIndex)->m_nStartSegmentIndex].m_vStates[(*nCurrentPlanIndex)->m_nStartSampleIndex].m_dV.transpose();
        //calculate target values

        int currentSegIndex, currentSampleIndex;
        currentSegIndex = (*nCurrentPlanIndex)->m_nStartSegmentIndex;
        currentSampleIndex = (*nCurrentPlanIndex)->m_nStartSampleIndex + nCurrentSampleIndex;
        MotionSample::FixSampleIndexOverflow(m_vSegmentSamples,currentSegIndex,currentSampleIndex);
        dT_target =  m_vSegmentSamples[currentSegIndex].m_vStates[currentSampleIndex].m_dTwv;
        targetVel = m_vSegmentSamples[currentSegIndex].m_vStates[currentSampleIndex].m_dV;

        //DLOG(INFO) << "GetCurrentCommands planid:" << (*nCurrentPlanIndex)->m_nPlanId << " sample index:" << nCurrentSampleIndex << " returning interpolation with i:" << interpolationAmount << " a:" << accel << " c:" << curvature << " t:" << torques.transpose();

        m_LastCommand.m_dForce = command.m_dForce;
        m_LastCommand.m_dCurvature = command.m_dCurvature;
        m_LastCommand.m_dPhi = command.m_dPhi;
        m_LastCommand.m_dTorque = command.m_dTorque;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaController::_GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sampleIndex, double& interpolationAmount) 
{
    interpolationAmount = 0;
    sampleIndex = -1;
    planIndex = m_lControlPlans.end();
    bool bPlanValid = false;

    if(m_lControlPlans.empty() == false) {
        //for(int ii = 0; ii < m_vControlPlans.size() ; ii++) {
        for(PlanPtrList::iterator it = m_lControlPlans.begin() ; it != m_lControlPlans.end() ; it++) {
            sampleIndex = 0;

            //only if the current time is within the bounds of the plan, will we go and search
            //for the exact command
            if(currentTime >= (*it)->m_Sample.m_vCommands.front().m_dTime &&
               currentTime <= (*it)->m_Sample.m_vCommands.back().m_dTime &&
               (*it)->m_Sample.m_vCommands.size() > 1)
            {
                ROS_INFO("Controller current time of %fs is between %fs and %fs", currentTime, (*it)->m_Sample.m_vCommands.front().m_dTime, (*it)->m_Sample.m_vCommands.back().m_dTime);
                planIndex = it;
                for(size_t jj = 1; jj < (*it)->m_Sample.m_vCommands.size() ; jj++){
                    if(((*it)->m_Sample.m_vCommands[jj].m_dTime) >= currentTime){
                        bPlanValid = true; //the plan has not yet finished
                        if(sampleIndex != -1){
                            double prevTime = (*it)->m_Sample.m_vCommands[sampleIndex].m_dTime;
                            double nextTime = (*it)->m_Sample.m_vCommands[jj].m_dTime;
                            interpolationAmount = (currentTime - prevTime) /(nextTime-prevTime);
                        }
                        break;
                    }
                    sampleIndex = jj;
                }
            }
            else
            {
                ROS_WARN("Controller failed to get plan index at %fs. Only have %fs - %fs.", currentTime, (*it)->m_Sample.m_vCommands.front().m_dTime, (*it)->m_Sample.m_vCommands.back().m_dTime);
            }
            
        }
    }
    else
    {
        ROS_WARN("No control plans.");
    }
    

    if( bPlanValid == false ) {
        planIndex = m_lControlPlans.end();
        sampleIndex = -1;
    }

    if( sampleIndex != m_nLastCurrentPlanIndex) {
        m_nLastCurrentPlanIndex = sampleIndex;
    }

}




////////////////////////////////////////////////////////////////
double MochaController::AdjustStartingSample(const std::vector<MotionSample>& segmentSamples,
                                           VehicleState& state,
                                           int& segmentIndex,
                                           int& sampleIndex,
                                           int lowerLimit /*= 100*/,
                                           int upperLimit /*= 100*/)
{
    //move within a certain neighbourhood of the samples and see if you can find a minimum distance to the trajectory
    int currentSegmentIndex = segmentIndex;
    int currentSampleIndex = sampleIndex;
    double minDistance = DBL_MAX;
    const VehicleState& currentState = segmentSamples[currentSegmentIndex].m_vStates[currentSampleIndex];
    Eigen::Vector3d distVect = currentState.m_dTwv.translation() - state.m_dTwv.translation();

    int minSegmentIndex = segmentIndex;
    int minSampleIndex = sampleIndex;

    //offset by a certain amount before
    currentSampleIndex -= lowerLimit; //0.5 seconds
    MotionSample::FixSampleIndexOverflow(segmentSamples,currentSegmentIndex,currentSampleIndex);

    for(int ii = 0; ii < upperLimit + lowerLimit ; ii++){ //-0.5s -> +0.5s
        //fix any over/underflow
        MotionSample::FixSampleIndexOverflow(segmentSamples,currentSegmentIndex,currentSampleIndex);

        //see if this distance is less than the prevous
        const VehicleState& currentState2 = segmentSamples[currentSegmentIndex].m_vStates[currentSampleIndex];

        distVect = currentState2.m_dTwv.translation() - state.m_dTwv.translation();
        double sn = distVect.squaredNorm();
        if( sn <= minDistance ) {
            minDistance = sn;
            minSegmentIndex = currentSegmentIndex;
            minSampleIndex = currentSampleIndex;
        }

        //increment the current sample
        currentSampleIndex++;
    }
    sampleIndex = minSampleIndex;
    segmentIndex = minSegmentIndex;

    //return the minimum distance
    return minDistance;
}

////////////////////////////////////////////////////////////////
void MochaController::PrepareLookaheadTrajectory(const std::vector<MotionSample> &vSegmentSamples,
                                               ControlPlan *pPlan,
                                               VelocityProfile& trajectoryProfile,
                                               MotionSample& trajectorySample,
                                               const double dLookaheadTime)
{
    ROS_INFO("Controller preparing lookahead trajectory.");
    double dLookahead = dLookaheadTime;
    //create a motion sample from this plan
    trajectoryProfile.push_back(VelocityProfileNode(0,pPlan->m_StartState.m_dV.norm()));

    int seg = pPlan->m_nStartSegmentIndex;
    int spl = pPlan->m_nStartSampleIndex;
    //reserve some states to improve efficiency
    trajectorySample.m_vStates.reserve(vSegmentSamples[seg].m_vStates.size());
    double trajTime = 0;
    while(trajTime <= dLookahead){
        trajectorySample.m_vStates.push_back(vSegmentSamples[seg].m_vStates[spl]);
        //set the correct time on the trajectory and increment
        trajectorySample.m_vStates.back().m_dTime = trajTime;
        trajTime += vSegmentSamples[seg].m_vCommands[spl].m_dT;
        spl++;
        if(MotionSample::FixSampleIndexOverflow(vSegmentSamples,seg,spl) && trajTime >= g_dMinLookaheadTime){
            trajectoryProfile.push_back(VelocityProfileNode(trajectorySample.GetDistance(),trajectorySample.m_vStates.back().m_dV.norm()));
        }

//        if(trajectorySample.m_vStates.back().IsAirborne()){
//            dLookahead = std::max(dLookahead,trajTime);
//        }
    }

    const double totalDist = trajectorySample.GetDistance();
    trajectoryProfile.push_back(VelocityProfileNode(totalDist,trajectorySample.m_vStates.back().m_dV.norm()));
    for(VelocityProfileNode& node : trajectoryProfile){
        node.m_dDistanceRatio /= totalDist;
    }

    const int nTotalTrajSamples = trajectorySample.m_vStates.size();
    //now add some more states to the end of the sample (for trajectory tracking)
    for(int jj = 0 ; jj < nTotalTrajSamples ; jj++){
        trajectorySample.m_vStates.push_back(vSegmentSamples[seg].m_vStates[spl]);
        //set the correct time on the trajectory and increment
        trajectorySample.m_vStates.back().m_dTime = trajTime;
        trajTime += vSegmentSamples[seg].m_vCommands[spl].m_dT;
        spl++;
        MotionSample::FixSampleIndexOverflow(vSegmentSamples,seg,spl);
    }



    pPlan->m_nEndSegmentIndex = pPlan->m_nStartSegmentIndex;
    pPlan->m_nEndSampleIndex = pPlan->m_nStartSampleIndex+nTotalTrajSamples;
    MotionSample::FixSampleIndexOverflow(vSegmentSamples,pPlan->m_nEndSegmentIndex,pPlan->m_nEndSampleIndex);
    pPlan->m_GoalState = vSegmentSamples[pPlan->m_nEndSegmentIndex].m_vStates[pPlan->m_nEndSampleIndex];
    //pPlan->m_GoalState.m_dV = pPlan->m_StartState.m_dV;

    //search for an airborne transition and adjust weights accordingly
//            int searchSeg = pPlan->m_nStartSegmentIndex;
//            int searchSpl = pPlan->m_nStartSampleIndex+lookaheadSamples;
//            MotionSample::FixSampleIndexOverflow(segmentSamples,searchSeg,searchSpl);
//            double totalTransitionSearchTime = 0.3;
//            double transitionTime = -1;
//            if(segmentSamples[searchSeg].m_vStates[searchSpl].IsAirborne() == false){
//                while(totalTransitionSearchTime > 0){
//                    searchSpl++;
//                    MotionSample::FixSampleIndexOverflow(segmentSamples,searchSeg,searchSpl);
//                    totalTransitionSearchTime -= segmentSamples[searchSeg].m_vCommands[searchSpl].m_dT;
//                    if(segmentSamples[searchSeg].m_vStates[searchSpl].IsAirborne() && transitionTime == -1){
//                        transitionTime = totalTransitionSearchTime;
//                        break;
//                    }
//                }
//            }

//            //if there is a transition from ground to airborne in this sample, then we must modify the weights
//            if(transitionTime == -1){
//                m_pPlanner->m_dTrajWeight(3) = THETA_MULTIPLIER;
//            }else{
//                interpolationAmount = transitionTime/totalTransitionSearchTime;
//                m_pPlanner->m_dTrajWeight(3) = THETA_MULTIPLIER*4*(interpolationAmount) + THETA_MULTIPLIER*(1-interpolationAmount);
//                DLOG(INFO) << "Transition in " << transitionTime << "s Setting theta weight to " << m_pPlanner->m_dTrajWeight(3);
//            }



    //pPlan->m_StartState.m_dCurvature = planStartCurvature;

    if(pPlan->m_GoalState.IsAirborne()){
        pPlan->m_GoalState.AlignWithVelocityVector();
        pPlan->m_GoalState.m_dCurvature = 0;
    }else{
        pPlan->m_GoalState.m_dCurvature = vSegmentSamples[pPlan->m_nEndSegmentIndex].m_vCommands[pPlan->m_nEndSampleIndex].m_dCurvature;
    }

    if(pPlan->m_StartState.m_dV.norm() >= 0.5){
        pPlan->m_StartState.AlignWithVelocityVector();
    }

}

/////////////////////////////////////////////////////////////////////////////////////////

void MochaController::ControlLoopFunc(const ros::TimerEvent& event)
{
    // int nNumControlPlans = 0;
    // m_dControlPlansPerS = 0;
    // double dLastTime = Tic();

    if (m_vSegmentSamples.size()==0) return;

    // m_bControllerRunning = false;
    m_ControllerState = IDLE;

    //reset the control commands
    //m_ControlCommand = ControlCommand();
    // m_bControllerRunning = true;


    //now run the controller to create a plan
    ControlPlan* pPlan;
    if(g_bInfiniteTime == false){
        m_dPlanTime = Tic();
    }
    PlanControl(m_dPlanTime, pPlan);

    // ROS_INFO("New plan: id %d start %f %f %f %f end %f %f %f %f norm %f sample len %d", 
    //     pPlan->m_nPlanId, 
    //     pPlan->m_dStartTime, pPlan->m_dStartPose.translation().x(), pPlan->m_dStartPose.translation().y(), pPlan->m_dStartPose.translation().z(),
    //     pPlan->m_dEndTime, pPlan->m_dEndPose.translation().x(), pPlan->m_dEndPose.translation().y(), pPlan->m_dEndPose.translation().z(),
    //     pPlan->m_dNorm,
    //     pPlan->m_Sample.m_vStates.size()
    //     );

    pubCurrentCommand();

//                //calculate error metrics
//                m_dTargetVel = dTargetVel.norm();
    // m_dVelError = m_vTargetVel.norm() - currentState.m_dV.norm();
//                m_Log.Log(*m_Controller.GetLookaheadTimePtr(),m_ControlCommand.m_dForce/500.0,pPlan != NULL ? pPlan->m_dNorm : -1,
//                          currentState.m_dV.norm(),currentState.m_dV[2],m_vTargetVel.norm());
        // m_Log.Log(m_ControlCommand.m_dPhi/500.0);

//                m_dPoseError = (dTargetPose*currentState.m_dTwv.inverse()).log().norm();

    // if(pPlan != NULL) 
    // {
    //     pubPlan();
    //     //update control plan statistics if there is a new control plan available
    //     nNumControlPlans++;
    //     if(Toc(dLastTime) > 0.5){
    //         m_dControlPlansPerS = (double)nNumControlPlans / (Toc(dLastTime));
    //         nNumControlPlans = 0;
    //         dLastTime =Tic();
    //     }
    // }

    // m_bControllerRunning = false;
    m_ControllerState = IDLE;
    return;
}

void MochaController::pubCurrentCommand()
{
    ControlCommand command;
    double now = Tic();
    GetCurrentCommands(now, command);
    // command.m_dT = m_dt;
    pubCommand(command);
    ROS_INFO("Published command at %f: force %f curv %f torque %f dt %f phi %f", now, command.m_dForce, command.m_dCurvature, command.m_dTorque.norm(), command.m_dT, command.m_dPhi);
}

void MochaController::pubCommand(ControlCommand& cmd)
{
    carplanner_msgs::Command cmd_msg;
    cmd_msg.force       = cmd.m_dForce;
    cmd_msg.curvature   = cmd.m_dCurvature;
    cmd_msg.dt          = cmd.m_dTime;
    cmd_msg.dphi        = cmd.m_dPhi;
    for(unsigned int i=0; i<3; i++)
    {
        cmd_msg.torques[i] = cmd.m_dTorque[i];
    }

    m_pubCommands.publish(cmd_msg);
    ros::spinOnce();
}

void MochaController::vehicleStateCb(const carplanner_msgs::VehicleState::ConstPtr& state_msg)
{
    // ROS_INFO("Controller got new state.");

    VehicleState state;
    state.fromROS(*state_msg);
    SetCurrentPose(state);

    // boost::mutex::scoped_lock(m_PoseMutex);
    // m_CurrentState.fromROS(*state_msg);
    // // GetCommandHistory(0, m_lCurrentCommands);
    // m_bPoseUpdated = true;
    return;
}

void MochaController::planCb(const carplanner_msgs::MotionPlan::ConstPtr& plan_msg)
{
    ROS_INFO("Controller got new plan.");
    boost::mutex::scoped_lock lock(m_MotionPlanMutex);
    m_vSegmentSamples.clear();
    for (uint i=0; i<plan_msg->samples.size(); i++)
    {
        MotionSample sample;
        sample.fromROS(plan_msg->samples[i]);
        m_vSegmentSamples.push_back(sample);
    }

    Reset();
    InitController();

    return;
}

bool MochaController::ApplyVelocities(const VehicleState& startState,
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

    double t0 = Tic();

    actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction> actionApplyVelocities_client("vehicle/"+std::to_string(nWorldId)+"/apply_velocities",true);
    actionApplyVelocities_client.waitForServer();


    carplanner_msgs::ApplyVelocitiesGoal goal;
    carplanner_msgs::ApplyVelocitiesResultConstPtr result;
    
    goal.initial_state = startState.toROS();
    goal.initial_motion_sample = sample.toROS();
    goal.world_id = nWorldId;
    goal.no_compensation = noCompensation;
    goal.no_delay = noDelay;

    double t1 = Tic();
    ROS_DBG("Sending goal %d with %d states at %fs", nWorldId, goal.initial_motion_sample.states.size(), t1);
    actionApplyVelocities_client.sendGoal(goal
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
    //   ;

    bool success;

    float timeout(15.0);
    timeout = 0.1 * goal.initial_motion_sample.states.size();
    bool finished_before_timeout = actionApplyVelocities_client.waitForResult(ros::Duration(timeout));
    double t3 = Tic();
    ROS_DBG("Got goal result %d at %fs, took %fs", nWorldId, t3, t3-t1);
    if (finished_before_timeout)
    {
        success = true;
        actionlib::SimpleClientGoalState state = actionApplyVelocities_client.getState();
        // DLOG(INFO) << "ApplyVelocities finished: " << state.toString();

        // ROS_INFO("Applying velocities (%d) succeeded, took %.2fs.", nWorldId, t3-t1);

        result = actionApplyVelocities_client.getResult();
        sample.fromROS(result->motion_sample);
    }
    else
    {
        success = false;
        ROS_ERROR("ApplyVelocities (%d) did not finish before the %fs timeout.", nWorldId, timeout);
    }

    double t4 = Tic();
    // ROS_INFO("Result %d received at %.2fs", nWorldId, ros::Time::now().toSec());

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

    // DLOG(INFO) << "AV done:" 
    //     << " world " << std::to_string(goal.world_id) 
    //     << " " << actionApplyVelocities_client.getState().toString()
    //     // << " poseOut " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x) 
    //     //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x)  
    //     //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.y)
    //     //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.z) 
    //     ;

    // ROS_INFO("ApplyVelocities for world %d %s, server wait %.2fs, goal prep %.2fs, server call %.2fs, total %.2fs", nWorldId, (success ? "succeeded" : "failed"), t1-t0, t2-t1, t3-t2, t3-t0);

    return success;
}
