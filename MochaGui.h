/*
 * File:   MochaGui.h
 * Author: nima
 *
 * Created on April 24, 2012, 10:31 PM
 */

#ifndef _MOCHAGUI_H
#define	_MOCHAGUI_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <carplanner_msgs/Command.h>
#include <carplanner_msgs/VehicleState.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "/home/mike/code/MochaGui_ros/conversion_tools.h"

#include <sys/types.h>
#include <dirent.h>
#include <atomic>
#include <thread>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <glog/logging.h>

#include "CarMessages.pb.h"

#include <CarPlanner/CarController.h>
#include <CarPlanner/LocalPlanner.h>
#include <CarPlanner/CarRegressor.h>
#include <CarPlanner/Localizer.h>
#include <CarPlanner/CVarHelpers.h>

#include "GLCar.h"
#include "PlannerGui.h"
#include "GLBulletDebugDrawer.h"

#include "SensorFusionCeres.h"
#include "GLGuiPanel.h"
#include "assimp/DefaultLogger.hpp"
#include "EventLogger.h"
#include "ProcessModelFusion.h"

// #include "WayPoint.h"
// #include "GLWayPoint.h"

using namespace CVarUtils;
using namespace std;
using namespace Eigen;

extern float g_fTurnrate;
extern float g_fSpeed;

class MochaGui {
public:
    ////////////////////////////////////////////////////////////////

    void Run();
    void Init(const std::string& sRefPlane, const std::string& sMesh, bool bLocalizer,
              const std::string sMode, const std::string sLogFile, const std::string& sParamsFile,
              const std::string& sCarMesh, const std::string& sWheelMesh, bool enableROS=false);



    static MochaGui *GetInstance();
    ~MochaGui();



protected:
    MochaGui();
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
    void _LocalizerReadFunc();
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

    // UDP values
//    unsigned m_MochaPort; // UDP port for this gui
//    unsigned m_ComPort; // UDP port for m_bSIL (unused)
//    unsigned m_CarPort; // UDP port of the ninja car
//    struct sockaddr_in mochAddr; // Address of this gui (for sending commands from)
//    struct sockaddr_in comAddr;  // Address for m_bSIL (currently not running 1/16/17)
//    struct sockaddr_in carAddr;  // Address of the car (for sending commands to)
//    socklen_t addrLen = sizeof(mochAddr);
//    int recvLen;
//    int sockFD;
//    unsigned char buf[2048];
//    unsigned int msgSize = 0;

    // aiScene *scene;

    // ROS variable
    bool m_bEnableROS;
    ros::NodeHandle* m_nh;
    tf::TransformBroadcaster m_tfbr;

    void InitROS();

    boost::thread* m_pPublisherThread;
    void _PublisherFunc();

    ros::Publisher m_commandPub;
    void _pubCommand();
    void _pubCommand(ControlCommand& );

    ros::Publisher m_statePub;
    void _pubState();
    void _pubState(VehicleState& );

    ros::Publisher m_meshPub;
    void _pubMesh();
    void _pubMesh(aiMesh* );

    ros::Subscriber m_waypointSub;
    void _waypointCB(const geometry_msgs::PoseStamped::ConstPtr& );

    ros::Subscriber m_meshSub;
    void _meshCB(const mesh_msgs::TriangleMeshStamped::ConstPtr& );

    void _convertMeshMsgToAssimpMesh(mesh_msgs::TriangleMeshStamped*&, aiMesh*& );
    // void _convertMeshMsgToGLMesh(mesh_msgs::TriangleMeshStamped*&, GLMesh*& );
    void _convertAssimpMeshToMeshMsg(aiMesh*&, mesh_msgs::TriangleMeshStamped*& );

    // GUIHelperInterface* m_guiHelper;

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
    vector<GLCachedPrimitives> m_vGLLineSegments;
    vector<GLCachedPrimitives> m_vTerrainLineSegments;
    list<GLCachedPrimitives*> m_lPlanLineSegments;
    list<std::vector<VehicleState> *> m_lPlanStates;
    GLCachedPrimitives* m_pControlLine;
    std::vector<MotionSample> m_vSegmentSamples;

    LocalPlanner m_Planner; // Car planner for trajectory plotting
    CarController m_Controller;
    BulletCarModel m_PlanCarModel;
    BulletCarModel m_LearningCarModel;
    BulletCarModel m_DriveCarModel;
    BulletCarModel m_ControlCarModel;

    //CVars
    vector<int>& m_Path;
#define WAYPOINT_VEL_INDEX 6
#define WAYPOINT_AIR_INDEX 7

    //flags
    std::atomic<bool> m_StillRun;
    std::atomic<bool> m_StillControl;
    bool& m_bPlannerOn;
    bool& m_bShowProjectedPath;
    bool& m_bCompute3dPath;
    bool& m_bSimulate3dPath;
    int& m_eControlTarget;
    bool& m_bControl3dPath;
    bool m_bSIL = false;

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

    Localizer m_Localizer;
    std::string m_sCarObjectName;
    //std::vector<MochaEntity> m_vEntities;

    boost::thread* m_pPlannerThread;
    boost::thread* m_pPhysicsThread;
    boost::thread* m_pControlThread;
    boost::thread* m_pCommandThread;
    boost::thread* m_pLearningThread;
    boost::thread* m_pLocalizerThread;

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

    ProcessModelFusion m_Fusion;

    boost::mutex m_ControlMutex;
    boost::mutex m_DrawMutex;

    PlannerGui m_Gui;

    //ui widgets
    GLGuiPanel m_GuiPanel;
    double m_dLocalizerFreq;
    double m_dVel;
    Eigen::Vector3d m_dPos;

    //load/save of files
    bool m_bLoadWaypoints;
    bool m_bSaveWaypoints;
    char m_pSaveFileName[100];
    int m_nSaveFileNameLen;
    int m_nSelectedFileName;
    std::string m_sParamsFile;
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
