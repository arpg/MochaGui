#include <memory>
#include <iomanip>
#include "MochaGui/EventLogger.h"

/////////////////////////////////////////////////////////////////////////////////////////
EventLogger::EventLogger()
{
}


/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::ReadLogFile(const std::string& fileName)
{
    if(m_ReadFile.is_open()){
        m_ReadFile.close();
    }

    m_ReadFile.open(fileName);
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::OpenLogFile(const std::string& fileName)
{
    CloseLogFile();
    m_File.open(fileName);
}



/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::LogPoseUpdate(const VehicleState& state,const PoseUpdateSource& eSource)
{
    msg_Log logMsg;
    msg_VehicleState& msgState = *logMsg.mutable_vehiclestate();
    WriteVehicleState(state,eSource,msgState);

    WriteMessage(logMsg);
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::LogControlCommand(const ControlCommand& command)
{
    msg_Log logMsg;
    msg_ControlCommand& msg = *logMsg.mutable_controlcommand();
    WriteControlCommand(command,msg);
    WriteMessage(logMsg);
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::ReadCommand(ControlCommand& command, const msg_ControlCommand& msgCommand)
{
    command.m_dT = msgCommand.dt();
    command.m_dForce = msgCommand.force();
    command.m_dPhi = msgCommand.phi();
    command.m_dTime = msgCommand.time();
    command.m_dCurvature = msgCommand.curvature();

    Eigen::MatrixXd tempVec;
    ReadMatrix(tempVec,msgCommand.torque_3d());
    tempVec = command.m_dTorque;
}



/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::WriteControlCommand(const ControlCommand& command, msg_ControlCommand& commandMsg)
{
    commandMsg.set_dt(command.m_dT);
    commandMsg.set_force(command.m_dForce);
    commandMsg.set_phi(command.m_dPhi);
    commandMsg.set_time(command.m_dTime);
    commandMsg.set_curvature(command.m_dCurvature);
    WriteMatrix(command.m_dTorque,*commandMsg.mutable_torque_3d());
}

/////////////////////////////////////////////////////////////////////////////////////////
std::string EventLogger::OpenNewLogFile(const std::string& sLogDir, const std::string& sPrefix /* = "" */ )
{
    CloseLogFile();
    std::string fileDir = sLogDir;
    // create the logging directory if needed
    if(fileDir.empty()) {
      std::time_t now = std::time(nullptr);
      std::tm tm = *std::localtime(&now);
      std::locale loc(std::wcout.getloc());
      std::stringstream wss;
      wss.imbue(loc);
      wss << std::put_time(&tm, "%Y%m%d_%H%M%S");
      fileDir = "logs/" + sPrefix + wss.str() + ".dat";
    }
    OpenLogFile(fileDir);
    return fileDir;
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::CloseLogFile()
{
    if(m_File.is_open()){
        m_File.close();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::ReadControlPlan(ControlPlan &plan, const msg_ControlPlan &msgPlan)
{
    plan.m_dStartTime = msgPlan.starttime();
    plan.m_dEndTime = msgPlan.endtime();
    ReadMotionSample(plan.m_Sample,msgPlan.sample());

    Eigen::MatrixXd vec;
    PoseUpdateSource source;
    ReadVehicleState(plan.m_StartState,source,msgPlan.startstate());
    plan.m_dStartPose = plan.m_StartState.m_dTwv;

    ReadVehicleState(plan.m_GoalState,source,msgPlan.endstate());
    plan.m_dEndPose = plan.m_GoalState.m_dTwv;

    plan.m_nStartSampleIndex = msgPlan.startsampleindex();
    plan.m_nStartSegmentIndex = msgPlan.startsegmentindex();

    plan.m_nEndSampleIndex = msgPlan.endsampleindex();
    plan.m_nEndSegmentIndex = msgPlan.endsegmentindex();
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::LogControlPlan(const ControlPlan& plan)
{
    msg_Log logMsg;
    msg_ControlPlan& planMsg = *logMsg.mutable_controlplan();
    planMsg.set_starttime(plan.m_dStartTime);
    planMsg.set_endtime(plan.m_dEndTime);
    WriteMotionSample(plan.m_Sample,*planMsg.mutable_sample());
    WriteVehicleState(plan.m_StartState,eVicon,*planMsg.mutable_startstate());
    WriteVehicleState(plan.m_GoalState,eVicon,*planMsg.mutable_endstate());
    planMsg.set_startsampleindex(plan.m_nStartSampleIndex);
    planMsg.set_startsegmentindex(plan.m_nStartSegmentIndex);
    planMsg.set_endsampleindex(plan.m_nEndSampleIndex);
    planMsg.set_endsegmentindex(plan.m_nEndSegmentIndex);
    //WriteMatrix(_GetPoseVector(plan.m_dStartPose),*planMsg.mutable_startpose_7d());
    //WriteMatrix(_GetPoseVector(plan.m_dEndPose),*planMsg.mutable_endpose_7d());

    WriteMessage(logMsg);
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::LogImuData(const double dSysTime, const double dDeviceTime, const Eigen::Vector3d &dAccel, const Eigen::Vector3d &dGyro)
{
    msg_Log logMsg;
    msg_ImuLog& imuMsg = *logMsg.mutable_imu();
    imuMsg.set_devicetime(dDeviceTime);
    imuMsg.set_systemtime(dSysTime);
    WriteMatrix(dAccel,*imuMsg.mutable_accel());
    WriteMatrix(dGyro,*imuMsg.mutable_gyro());

    WriteMessage(logMsg);
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::LogLocalizerData(const double dSysTime, const double dDeviceTime, const Sophus::SE3d &dTwb)
{
    msg_Log logMsg;
    msg_ViconLog& viconMsg = *logMsg.mutable_vicon();
    viconMsg.set_devicetime(dDeviceTime);
    viconMsg.set_systemtime(dSysTime);
    WriteMatrix(_GetPoseVector(dTwb),*viconMsg.mutable_pose_7d());

    WriteMessage(logMsg);
}


/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::WriteSegmentSamples(const std::vector<MotionSample> vSegmentSamples)
{
    msg_Log logMsg;
    msg_Segments& segMsg = *logMsg.mutable_segments();
    for(const MotionSample& sample: vSegmentSamples){
        WriteMotionSample(sample,*segMsg.add_segments());
    }

    WriteMessage(logMsg);
}


/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::ReadVehicleState(VehicleState &state, EventLogger::PoseUpdateSource &eSource, const msg_VehicleState &msgState)
{
    state.m_dCurvature = msgState.curvature();
    state.m_dSteering = msgState.steering();
    state.m_dTime = msgState.time();
    eSource = (PoseUpdateSource)msgState.source();

    Eigen::MatrixXd vec;
    ReadMatrix(vec,msgState.pose_7d());
    state.m_dTwv = ReadPoseVector(vec);
    ReadMatrix(vec,msgState.vel_3d());
    state.m_dV = vec;
    ReadMatrix(vec,msgState.w_3d());
    state.m_dW = vec;
}



/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::WriteVehicleState(const VehicleState& state,const PoseUpdateSource& eSource,msg_VehicleState &msgState)
{
    msgState.set_curvature(state.m_dCurvature);
    msgState.set_steering(state.m_dSteering);
    msgState.set_time(state.m_dTime);
    msgState.set_source((int)eSource);

    WriteMatrix(_GetPoseVector(state.m_dTwv),*msgState.mutable_pose_7d());
    WriteMatrix(state.m_dV,*msgState.mutable_vel_3d());
    WriteMatrix(state.m_dW,*msgState.mutable_w_3d());
}

/////////////////////////////////////////////////////////////////////////////////////////
bool EventLogger::ReadMessage(msg_Log &msg)
{
    m_LastLogMessage.Clear();
    int byteSize;
    m_ReadFile >> byteSize;
    if(m_ReadFile.fail()){
        m_ReadFile.close();
        return false;
    }
    char* array = new char[byteSize];
    m_ReadFile.read(array,byteSize);
    if(m_ReadFile.fail()){
        m_ReadFile.close();
        return false;
    }
    msg.ParseFromArray((void*)array,byteSize);
    delete array;
    m_LastLogMessage = msg;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::WriteMessage(msg_Log &msg)
{
    std::unique_lock<std::mutex> lock(m_WriteMutex);
    msg.set_timestamp(CarPlanner::Tic());
    int byteSize = msg.ByteSize();
    m_File << byteSize ; //write the size
    char* array = new char[byteSize];
    msg.SerializeToArray((void*)array,byteSize);
    m_File.write(array,byteSize);
    delete[] array;
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::ReadMotionSample(MotionSample &sample, const msg_MotionSample &msgSample)
{
    sample.m_vStates.resize(msgSample.states_size());
    sample.m_vCommands.resize(msgSample.command_size());

    for(int  ii = 0 ; ii < msgSample.states_size() ; ii++){
        PoseUpdateSource source;
        ReadVehicleState(sample.m_vStates[ii],source,msgSample.states(ii));
    }

    for(int ii = 0 ; ii < msgSample.command_size() ; ii++){
        ReadCommand(sample.m_vCommands[ii],msgSample.command(ii));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::WriteMotionSample(const MotionSample &sample, msg_MotionSample &msgSample)
{
    for(const VehicleState& state: sample.m_vStates){
        msg_VehicleState& msgState = *msgSample.add_states();
        WriteVehicleState(state,eVicon,msgState);
    }

    for(const ControlCommand& command: sample.m_vCommands){
        msg_ControlCommand& msgCommand = *msgSample.add_command();
        WriteControlCommand(command,msgCommand);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::WriteMatrix(const Eigen::MatrixXd &mat, msg_Matrix &msg)
{
    msg.set_cols(mat.cols());
    msg.set_rows(mat.rows());
    for(int ii = 0 ; ii < mat.cols()*mat.rows() ; ii++){
        msg.add_data(mat(ii));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void EventLogger::ReadMatrix(Eigen::MatrixXd &mat, const msg_Matrix &msg)
{
    mat.resize(msg.rows(),msg.cols());
    for(int ii = 0 ; ii < msg.data_size() ; ii++){
        mat(ii) = msg.data(ii);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector7d EventLogger::_GetPoseVector(const Sophus::SE3d &pose)
{
    Eigen::Vector7d vec;
    Eigen::Map<Sophus::SE3d> map(vec.data());
    map = pose;
    return vec;
}

/////////////////////////////////////////////////////////////////////////////////////////
Sophus::SE3d EventLogger::ReadPoseVector(const Eigen::MatrixXd &vec)
{
    const Eigen::Map<const Sophus::SE3d> map(vec.data());
    return map;
}
