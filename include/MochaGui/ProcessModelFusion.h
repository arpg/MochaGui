#ifndef PROCESSMODELFUSION_H
#define PROCESSMODELFUSION_H

#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>

#include "SensorFusionCeres.h"

class ProcessModelFusion : public fusion::SensorFusionCeres
{
private:
    BulletCarModel* m_pFusionModel;
    double m_dLastCommandTime;
    bool m_bFusionCarModelInitialized;
    std::mutex m_DriveCarMutex;
    VehicleState m_LastVehicleState;
public:
    ProcessModelFusion(const int& nFilterSize,BulletCarModel* pFusionModel);
    void RegisterInputCommands(ControlCommand command);
    void RegisterGlobalPoseWithProcessModel(const Sophus::SE3d &dTwc, double viconTime, double time, ControlCommand commands);
    void GetVehicleState(VehicleState &state);
};

#endif // PROCESSMODELFUSION_H