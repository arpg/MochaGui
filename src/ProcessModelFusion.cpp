#include "MochaGui/ProcessModelFusion.h"

/////////////////////////////////////////////////////////////////////////////////////////
ProcessModelFusion::ProcessModelFusion(const int &nFilterSize, BulletCarModel *pFusionModel) :
    SensorFusionCeres(nFilterSize),
    m_pFusionModel(pFusionModel),
    m_dLastCommandTime(-1),
    m_bFusionCarModelInitialized(false)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
void ProcessModelFusion::RegisterInputCommands(ControlCommand command)
{
    if(m_bFusionCarModelInitialized == true){
        if(m_dLastCommandTime == -1){
            m_dLastCommandTime = CarPlanner::Tic();
        }else{
            double currentTime = CarPlanner::Tic();
            if((currentTime - m_dLastCommandTime) > 0.01){
                std::lock_guard<std::mutex> lock(m_DriveCarMutex);
                double dt = currentTime - m_dLastCommandTime;
                m_dLastCommandTime = currentTime;
                m_pFusionModel->UpdateState(0, command,dt);
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProcessModelFusion::RegisterGlobalPoseWithProcessModel(const Sophus::SE3d& dTwc,double viconTime,double time,ControlCommand command)
{
    if(m_bFusionCarModelInitialized == true){
        VehicleState currentState;
        {
            std::lock_guard<std::mutex> lock(m_DriveCarMutex);
            double currentTime = CarPlanner::Tic();
            double dt = currentTime - m_dLastCommandTime;
            m_pFusionModel->UpdateState(0,command,dt);
            m_dLastCommandTime = currentTime;
            m_pFusionModel->GetVehicleState(0,currentState);
        }

        //find the delta to the previous state
        fusion::PoseData delta;
        Sophus::SE3d& T_wv2 = currentState.m_dTwv;
        Sophus::SE3d& T_wv1 = m_LastVehicleState.m_dTwv;
        delta.m_dPose = T_wv2.inverse()*T_wv1;
        //dout("Delta pose is " << delta.m_dPose.transpose());
        delta.m_dV = currentState.m_dV-m_LastVehicleState.m_dV;
        delta.m_dW = currentState.m_dW-m_LastVehicleState.m_dW;
        m_LastVehicleState = currentState;
        RegisterGlobalPose(dTwc,delta,viconTime,time,true,true);
    }else{
        RegisterGlobalPose(dTwc,viconTime,time);
    }


    if(IsActive()){

            std::lock_guard<std::mutex> lock(m_DriveCarMutex);
            fusion::PoseParameter currentPose = GetCurrentPose();
            m_LastVehicleState.m_dTwv = currentPose.m_dPose;
            m_LastVehicleState.m_dV = currentPose.m_dV;
            m_LastVehicleState.m_dW = currentPose.m_dW;
            m_pFusionModel->SetState(0,m_LastVehicleState);
            //update the state to raycast wheels in their proper place, but do not push the car forward
            m_pFusionModel->UpdateState(0,command,0.000001);

        if(m_bFusionCarModelInitialized == false){
            m_bFusionCarModelInitialized = true;
            m_dLastCommandTime = CarPlanner::Tic();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void ProcessModelFusion::GetVehicleState(VehicleState &state)
{
    std::lock_guard<std::mutex> lock(m_DriveCarMutex);
    m_pFusionModel->GetVehicleState(0,state);
}


