#pragma once

#include <Eigen/Eigen>
#include <eigen3/Eigen/src/Core/products/GeneralBlockPanelKernel.h>

#include <vector>
#include <condition_variable>
#include "CarPlanner/CarPlannerCommon.h"
#include "Messages.pb.h" // NOT HAL/Messages.pb.h.
#include <Node/Node.h>


//struct MochaEntity
//{
//    std::string m_Name;
//    Eigen::Vector6d m_Pose;
//    Eigen::Vector6d m_DPose;
//};

class Localizer
{
    public:
        Localizer();
        virtual ~Localizer();
        void TrackObject(const std::string& sNodeName , const std::string& sObjectName, bool bRobotFrame = true);
        void TrackObject(const std::string& sNodeName, const std::string& sObjectName , Sophus::SE3d dToffset, bool bRobotFrame = true);
        void Start();
        void Stop();
        Sophus::SE3d GetPose(const std::string& sObjectName , bool blocking = false, double *time = NULL, double *rate = NULL);
        //Eigen::Matrix<double,6,1> GetdPose( const std::string& sObjectName );
        eLocType WhereAmI( Eigen::Vector3d P );
        eLocType WhereAmI( Eigen::Vector6d P );

    private:
        static void _ThreadFunction(Localizer *pVT);

    private:

        struct TrackerObject
        {
            Sophus::SE3d                        m_dSensorPose;
            Sophus::SE3d                        m_dToffset;
            double                              m_dTime;
            //vrpn_Tracker_Remote*              m_pTracker; //crh convert to node call?
            std::shared_ptr<node::node>  				m_pNode;
            Localizer*                          m_pLocalizerObject;
            std::mutex                          m_Mutex;
            std::condition_variable             m_PoseUpdated;
            std::string													m_NodeReceiverName;

            //metrics
            double                                  m_dLastTime;
            int                                     m_nNumPoses;
            double                                  m_dPoseRate;
            bool                                    m_bRobotFrame;
            bool                                    m_bPoseUpdated;
            TrackerObject() : m_dLastTime(-1), m_nNumPoses(-1) , m_bPoseUpdated(false)
            {
            }

            TrackerObject(const Localizer::TrackerObject& obj): m_Mutex(),
                m_PoseUpdated()
            {
                m_dSensorPose = obj.m_dSensorPose;
                m_dTime = obj.m_dTime;
                m_pLocalizerObject = obj.m_pLocalizerObject;

            }
        }; //end struct TrackerObject

        std::map< std::string,  TrackerObject >     m_mObjects;

        bool                                        m_bIsStarted;
        std::thread*                                m_pThread;
};
