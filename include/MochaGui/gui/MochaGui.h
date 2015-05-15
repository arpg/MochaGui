/*
 * File:   MochaGui.h
 * Author: nima
 *
 * Created on April 24, 2012, 10:31 PM
 */

#ifndef _MOCHAGUI_H
#define	_MOCHAGUI_H

#include <sys/types.h>
#include <dirent.h>
#include <atomic>
#include <thread>
#include <assimp/DefaultLogger.hpp>

#include "CarPlanner/CarController.h"
#include "CarPlanner/LocalPlanner.h"
#include "CarPlanner/CarRegressor.h"
#include "CarPlanner/CVarHelpers.h"
#include "SensorFusion/SensorFusionCeres.h"
#include "Node/Node.h"

#include "Messages.pb.h"
#include "MochaGui/GLCar.h"
#include "MochaGui/PlannerGui.h"
#include "MochaGui/GLBulletDebugDrawer.h"
#include "MochaGui/gui/GLGuiPanel.h"
#include "MochaGui/EventLogger.h"
#include "MochaGui/ProcessModelFusion.h"
#include "MochaGui/Vicon.h"

using namespace CVarUtils;
using namespace std;
using namespace Eigen;

extern float g_fTurnrate;
extern float g_fSpeed;



#define DEFAULT_MAP_NAME "room.heightmap"
#define DEFAULT_MAP_IMAGE_NAME "room.image"
#define SAVED_MAP_NAME "room2.heightmap"

#define PARAMS_FILE_NAME "gui_params.csv"


class MochaGui {
public:
    ////////////////////////////////////////////////////////////////

    void Run();
    void Init(string sRefPlane, std::string sMesh, bool bVicon, std::string sMode, string sLogFile);

    static MochaGui *GetInstance();
    ~MochaGui();



protected:
    MochaGui();
    void _FixGround();
    bool _SetWaypointVel(std::vector<std::string> *vArgs);
    bool _Refresh(std::vector<std::string> *vArgs);
    void _RefreshWaypoints();
    bool _CommandFunc(MochaCommands command);
    bool _IteratePlanner(LocalProblem& problem,
                         MotionSample& sample,
                         Eigen::Vector3dAlignedVec& vActualTrajectory,
                         Eigen::Vector3dAlignedVec& vControlTrajectory,
            bool only2d = false);
    void _PhysicsFunc();
    void _ControlCommandFunc();
    void _ControlFunc();
    void _ImuReadFunc();
    void _ViconReadFunc();
    void _PlannerFunc();
    void _LearningFunc(ControlPlan *m_pRegressionPlan);
    void _UpdateLearning(ControlCommand command,
                         VehicleState& state);
    void _UpdateWaypointFiles();
    void _StartThreads();
    void _KillThreads();
    void _KillController();
    void _CreateLogFilesIfNeeded();

    void _PopulateSceneGraph();
    void _UpdateVehicleStateFromFusion(VehicleState& currentState);

    void _UpdateVisuals();
    void _PlaybackLog(double dt);
    double m_dCurrentHue;
    bool _UpdateControlPathVisuals(const ControlPlan* pPlan);

    void _LoadDefaultParams();

    static bool SetWaypointVelHandler(std::vector<std::string> *vArgs) { return GetInstance()->_SetWaypointVel(vArgs); }
    static bool RefreshHandler(std::vector<std::string> *vArgs) { return GetInstance()->_Refresh(vArgs); }
    static bool CommandHandler(MochaCommands command) { return GetInstance()->_CommandFunc(command); }


    //heightmap variable
    //MeshHeightMap m_MeshHeightMap;
    //PeaksHeightMap m_PeaksHeightMap;
    //GLHeightMap m_GLHeightMap;
    //KinectHeightMap m_KHeightMap;
    //HeightMap &m_ActiveHeightMap;

    //car variables
    GLBulletDebugDrawer m_BulletDebugDrawer;
    GLMesh m_TerrainMesh;

    int m_nStateStatusId;
    int m_nLearningStatusId;
    int m_nControlStatusId;
    int m_nGravityStatusId;

    int m_nDriveCarId;

    double m_dControlPlansPerS;

    double m_dDiagTime;

    std::vector<Eigen::MatrixXd*> m_vWayPoints;
    //vector<GLWayPoint*> m_vGLWayPoints;
    const GLWayPoint*m_pCurrentlySolving[2];
    std::vector<GLCachedPrimitives> m_vGLLineSegments;
    std::vector<GLCachedPrimitives> m_vTerrainLineSegments;
    list<GLCachedPrimitives*> m_lPlanLineSegments;
    list<std::vector<VehicleState> *> m_lPlanStates;
    std::shared_ptr<GLCachedPrimitives> m_pControlLine;
    std::vector<MotionSample> m_vSegmentSamples;

    LocalPlanner m_Planner; // Car planner for trajectory plotting
    CarController m_Controller;
    BulletCarModel m_PlanCarModel;
    BulletCarModel m_LearningCarModel;
    BulletCarModel m_DriveCarModel;
    BulletCarModel m_ControlCarModel;

    //CVars
    std::vector<int>& m_Path;
#define WAYPOINT_VEL_INDEX 6
#define WAYPOINT_AIR_INDEX 7


    //flags
    std::atomic<bool> m_bStillRun;
    std::atomic<bool> m_bStillImu;
    std::atomic<bool> m_bStillControl;
    std::atomic<bool> m_bStillVicon;
    bool& m_bPlannerOn;
    bool& m_bShowProjectedPath;
    bool& m_bCompute3dPath;
    bool& m_bSimulate3dPath;
    int& m_eControlTarget;
    bool& m_bControl3dPath;


    bool& m_bFuseImu;
    bool m_bPlanning;
    bool m_bControllerRunning;
    bool m_bLearningRunning;
    unsigned int& m_nPathSegments;
    double& m_dTimeInterval;
    double m_dFps;
    bool m_bPause;
    bool m_bStep;
    bool m_bLastPause;
    bool m_bAllClean;

    Vector3dAlignedVec m_vFixGroundSamples;
    //Eigen::Matrix3d m_RfixGround;

    Vicon m_Vicon;
    std::string m_sCarObjectName;
    //std::vector<MochaEntity> m_vEntities;

    std::thread* m_pPlannerThread;
    std::thread* m_pPhysicsThread;
    std::thread* m_pControlThread;
    std::thread* m_pCommandThread;
    std::thread* m_pLearningThread;
    std::thread* m_pImuThread;
    std::thread* m_pViconThread;

    ControlCommand m_ControlCommand;
    double m_dTargetVel;
    double m_dPoseError;
    double m_dVelError;
    double m_dControlAccel;
    double m_dControlPhi;




    //logging
    EventLogger m_Logger;
    EventLogger m_FusionLogger;
    double m_dStartTime;
    bool& m_bLoggerEnabled;
    std::string& m_sLogFileName;
    bool& m_bFusionLoggerEnabled;
    bool& m_bLoggerPlayback;
    unsigned int m_nNumControlSignals;
    unsigned int m_nNumPoseUpdates;
    std::string m_sPlaybackLogFile;
    double m_dPlaybackTimer;

    node::node m_Node;   //node for capturing IMU data from the car
    ProcessModelFusion m_Fusion;

    std::mutex m_ControlMutex;
    std::mutex m_DrawMutex;

    PlannerGui m_Gui;

    //ui widgets
    GLGuiPanel m_GuiPanel;
    double m_dViconFreq;
    double m_dImuFreq;
    double m_dVel;
    Eigen::Vector3d m_dPos;

    //load/save of files
    bool m_bLoadWaypoints;
    bool m_bSaveWaypoints;
    char m_pSaveFileName[100];
    int m_nSaveFileNameLen;
    int m_nSelectedFileName;
    std::vector<std::string> m_vFileNames;

    std::vector<RegressionParameter> m_vLearningParams;
    std::vector<RegressionParameter> m_vControlParams;
    std::vector<RegressionParameter> m_vPlannerParams;
    std::vector<RegressionParameter> m_vDriveParams;
    CarParameterMap m_mDefaultParameters;

    pangolin::DataLog m_Log;
    pangolin::View* m_pGraphView;
    Eigen::Vector3d m_vTargetVel;
    Sophus::SE3d m_dTargetPose;
    double m_dControlDelay;

    std::atomic<double> m_dPlanTime;

    GLAxis m_DestAxis;
};

#endif	/* MOCHAGUI_H */

