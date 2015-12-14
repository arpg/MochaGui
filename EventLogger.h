#ifndef EVENTLOGGER_H
#define EVENTLOGGER_H

#include <node/Node.h>
#include "CarPlanner/CarPlannerCommon.h"
#include "CarPlanner/BulletCarModel.h"
#include "CarPlanner/CarController.h"
#include "Messages.pb.h"


class EventLogger
{   
public:
    enum PoseUpdateSource
    {
        eIMU = 0,
        eLocalizer = 1,
        eSimulation = 2
    };

    EventLogger();
    void ReadLogFile(const std::string& fileName);
    void OpenLogFile(const std::string& fileName);

    bool ReadMessage(msg_Log& msg);
    void ReadMatrix(Eigen::MatrixXd &mat, const msg_Matrix &msg);
    void ReadVehicleState(VehicleState &state, PoseUpdateSource &eSource, const msg_VehicleState &msgState);
    void ReadMotionSample(MotionSample& sample, const msg_MotionSample& msgSample);
    void ReadCommand(ControlCommand& command, const msg_ControlCommand& msgCommand);
    void ReadControlPlan(ControlPlan& plan, const msg_ControlPlan& msgPlan);

    void WriteVehicleState(const VehicleState &state, const PoseUpdateSource &eSource, msg_VehicleState &msgState);
    void WriteMessage(msg_Log& msg);
    void WriteMotionSample(const MotionSample& sample, msg_MotionSample& msgSample);
    void WriteControlCommand(const ControlCommand &command, msg_ControlCommand &commandMsg);
    void WriteSegmentSamples(const std::vector<MotionSample> vSegmentSamples);
    void WriteMatrix(const Eigen::MatrixXd& mat,msg_Matrix& msg);

    void LogPoseUpdate(const VehicleState &state, const PoseUpdateSource &eSource);
    void LogControlCommand(const ControlCommand &command);
    void LogControlPlan(const ControlPlan &plan);
    void LogImuData(const double dSysTime, const double dDeviceTime, const Eigen::Vector3d& dAccel, const Eigen::Vector3d& dGyro);
    void LogLocalizerData(const double dSysTime, const double dDeviceTime, const Sophus::SE3d& dTwb);

    std::string OpenNewLogFile(const std::string &sLogDir, const std::string &sPrefix = "");
    void CloseLogFile();

    bool IsReady(){ return m_bLoggerReady; }
    void SetIsReady(const bool bIsReady){ m_bLoggerReady = bIsReady;}

    msg_Log& GetLastMessage(){return m_LastLogMessage;}
    Sophus::SE3d ReadPoseVector(const Eigen::MatrixXd &vec);
private:
    Eigen::Vector7d _GetPoseVector(const Sophus::SE3d& pose);
    std::ofstream m_File;
    std::ifstream m_ReadFile;
    bool m_bLoggerReady;
    msg_Log m_LastLogMessage;
    boost::mutex m_WriteMutex;
};

#endif // EVENTLOGGER_H
