#ifndef LEARNINGGUI_H
#define LEARNINGGUI_H

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

#include <boost/lexical_cast.hpp>
#include <tuple>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/GLMesh.h>
#include <CVars/CVar.h>
#include "CarMessages.pb.h"
#include <CarPlanner/CarPlannerCommon.h>
#include <CarPlanner/BulletCarModel.h>
#include <CarPlanner/Localizer.h>
#include <CarPlanner/CarRegressor.h>
#include <Node/Node.h>
#include <SensorFusion/SensorFusionCeres.h>

#include "JoystickHandler.h"
#include "PlannerGui.h"
#include "GLBulletDebugDrawer.h"
#include "CarMessages.pb.h"
#include "GLLearningPanel.h"
#include "ProcessModelFusion.h"
#include "EventLogger.h"

#define LEARNING_PARAMS_FILE_NAME "/Users/crh/data/params.csv"

using namespace CVarUtils;

enum Mode
{
    Mode_Experiment = 1,
    Mode_Simulation = 2
};

class LearningGui
{
public:
    LearningGui();
    /// Initializes the GUI
    void Init(std::string sRefPlane, std::string sMeshName, bool bLocalizerTransform, Mode eMode);

    /// Runs the GUI
    void Run();

    static LearningGui *GetInstance();
private:
    static bool SaveDataHandler(std::vector<std::string> *vArgs) { return GetInstance()->_SaveData(vArgs); }
    static bool LoadDataHandler(std::vector<std::string> *vArgs) { return GetInstance()->_LoadData(vArgs); }
    bool _SaveData(std::vector<std::string> *vArgs);
    bool _LoadData(std::vector<std::string> *vArgs);

    void _UpdateTrajectories();
    /// Adds a command and vehicle state to the learning queue and starts the learning thread if enough
    /// samples have been collected
    void _UpdateLearning(ControlCommand command, VehicleState &state);

    /// The function which is called by the learning thread
    void _LearningFunc(MotionSample *pRegressionPlan);
    void _LearningCaptureFunc();

    /// Populates the initial scenegraph items
    void _PopulateSceneGraph();

    /// Function that pushes forward the physics based on the input commands
    void _PhysicsFunc();

    void _SetPoseFromFusion();

    void _JoystickReadFunc();
    void _ImuReadFunc();
    void _LocalizerReadFunc();

    void _CommandHandler(const MochaCommands& command);

    /// localizer reader
    Localizer m_Localizer;
    /// node for capturing IMU data from the car
    node::node m_Node;
    /// name of the car used for localizer tracking
    std::string m_sCarObjectName;

    //view related variables
    JoystickHandler m_Joystick;
    BulletCarModel m_FusionCarModel;
    BulletCarModel m_DriveCarModel;
    BulletCarModel m_LearningCarModel;

    SceneGraph::GLMesh m_TerrainMesh;
    GLBulletDebugDrawer m_BulletDebugDrawer;

    PlannerGui m_Gui;
    int m_nDriveCarId;
    int m_nPlayBackCarId;

    double& m_dT;
    double m_dImuRate;
    double m_dLocalizerRate;

    bool& m_bLearn;
    bool m_bLearningRunning;
    MotionSample m_PlaybackSample;
    MotionSample* m_pRegressionSample;
    CarRegressor m_Regressor;
    double& m_dLearnEps;
    int m_nCollectedRegressionSamples;
    int& m_nTotalRegressionSamples;

    boost::thread* m_pPhysicsThread;
    boost::thread* m_pLearningThread;
    boost::thread* m_pLearningCaptureThread;
    boost::thread* m_pImuThread;
    boost::thread* m_pLocalizerThread;
    boost::thread* m_pJoystickThread;
    boost::mutex m_JoystickMutex;
    boost::mutex m_RenderkMutex;

    ControlCommand m_JoystickCommand;

    //ui widgets
    GLLearningPanel m_LearningPanel;

    ProcessModelFusion m_Fusion;
    Mode m_eMode;

    std::vector<RegressionParameter> m_vDriveLearningParams;
    std::vector<RegressionParameter> m_vLearningParams;
    std::list<GLLineStrip> m_lGLLineSegments;
    std::vector<MotionSample> m_vSamples;
    std::vector<int> m_vSampleIndices;
    std::vector<RegressionParameter> m_vTweakParams;

    double m_dLastPoseTime;
    double m_dTotalLearningTime;

    bool m_bPlayback;
    bool m_bStep;
    bool m_bPaused;
    bool m_bMapSteering;
    bool m_bProcessModelEnabled;
    bool m_bRefresh;
    bool m_bRegress;
    //bool m_bProcessModelFusionActive;

    std::vector<std::pair<double,double> > m_vSteeringPairs;
    Eigen::Vector6d m_dLastSteeringPose;
    int m_nSteeringCount;
    double m_dLastCurv;

    EventLogger m_Logger;

    CarParameterMap m_mParameters;
};

#endif // LEARNINGGUI_H
