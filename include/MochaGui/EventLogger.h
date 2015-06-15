#ifndef EVENTLOGGER_H
#define EVENTLOGGER_H

#include "Messages.pb.h"
#include "MochaGui/Matrix.h"
#include "CarPlanner/CarPlannerCommon.h"
#include "CarPlanner/BulletCarModel.h"
#include "CarPlanner/CarController.h"


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

    bool ReadMessage(ninjacar::LogMsg& msg);
    void ReadVehicleState(VehicleState &state, PoseUpdateSource &eSource, const ninjacar::VehicleStateMsg &msgState);
    void ReadMotionSample(MotionSample& sample, const ninjacar::MotionSampleMsg& msgSample);
    void ReadCommand(ControlCommand& command, const ninjacar::ControlCommandMsg& msgCommand);
    void ReadControlPlan(ControlPlan& plan, const ninjacar::ControlPlanMsg& msgPlan);

    void WriteVehicleState(const VehicleState &state, const PoseUpdateSource &eSource, ninjacar::VehicleStateMsg &msgState);
    void WriteMessage(ninjacar::LogMsg& msg);
    void WriteMotionSample(const MotionSample& sample, ninjacar::MotionSampleMsg& msgSample);
    void WriteControlCommand(const ControlCommand &command, ninjacar::ControlCommandMsg &msgControlCommand);
    void WriteSegmentSamples(const std::vector<MotionSample> vSegmentSamples);

    void LogPoseUpdate(const VehicleState &state, const PoseUpdateSource &eSource);
    void LogControlCommand(const ControlCommand &command);
    void LogControlPlan(const ControlPlan &plan);
    void LogImuData(const double dSysTime, const double dDeviceTime, const Eigen::Vector3d& dAccel, const Eigen::Vector3d& dGyro);
    void LogLocalizerData(const double dSysTime, const double dDeviceTime, const Sophus::SE3d& dTwb);

    std::string OpenNewLogFile(const std::string &sLogDir, const std::string &sPrefix = "");
    void CloseLogFile();

    bool IsReady(){ return m_bLoggerReady; }
    void SetIsReady(const bool bIsReady){ m_bLoggerReady = bIsReady;}

    ninjacar::LogMsg& GetLastMessage(){return m_LastLogMessage;}
    Sophus::SE3d ReadPoseVector(const Eigen::MatrixXd &vec);
private:
    Eigen::Vector7d _GetPoseVector(const Sophus::SE3d& pose);
    std::ofstream m_File;
    std::ifstream m_ReadFile;
    bool m_bLoggerReady;
    ninjacar::LogMsg m_LastLogMessage;
    std::mutex m_WriteMutex;
};

#endif // EVENTLOGGER_H
