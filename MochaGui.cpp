#include <stdio.h>
#include "MochaGui.h"

//car control variables
float g_fTurnrate = 0;
float g_fSpeed = 0;
using namespace SceneGraph;
using namespace pangolin;
#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768
#define PANEL_WIDTH 0
#define MAX_ACCEL_PPM 80.0
#define MIN_ACCEL_PPM 20.0

MochaGui *g_pMochaGuiInstance = NULL;
static bool& g_bContinuousPathPlanning = CVarUtils::CreateGetUnsavedCVar("debug.ContinuousPathPlanning",false);
static bool& g_bImuIntegrationOnly = CVarUtils::CreateGetUnsavedCVar("debug.ImuIntegrationOnly",false);
static bool& g_bPlaybackControlPaths = CVarUtils::CreateGetUnsavedCVar("debug.PlaybackControlPaths",false);
static bool& g_bProcessModelActive = CVarUtils::CreateGetUnsavedCVar("debug.ProcessModelActive",false);
static int& g_nStartSegmentIndex = CVarUtils::CreateGetUnsavedCVar("debug.StartSegmentIndex",0);
static bool& g_bFreezeControl(CVarUtils::CreateGetUnsavedCVar("debug.FreezeControl",false,""));
static bool& g_bInertialControl = CVarUtils::CreateGetUnsavedCVar("debug.InertialControl",true);
static bool& g_bInfiniteTime = CVarUtils::CreateGetUnsavedCVar("debug.InfiniteTime",false);
static int& g_nIterationLimit = CVarUtils::CreateGetUnsavedCVar("planner.IterationLimit", 10, "");



////////////////////////////////////////////////////////////////
MochaGui::MochaGui() :
    //m_ActiveHeightMap(m_KHeightMap),
    m_Path(CreateCVar("path", vector<int>(), "Path vector.")),
    m_bPlannerOn(CreateUnsavedCVar("planner.PlannerOn", false, "if true, the car model will only follow the control law from the 2D path.")),
    m_bShowProjectedPath(CreateCVar("planner.ShowProjected2dPath", true, "how the 2D path z-projected onto the surface")),    
    m_bCompute3dPath(CreateCVar("planner.Compute3DPath", true, "Compute path using terrain driven simulation")),
    m_bSimulate3dPath(CreateUnsavedCVar("planner.Simulate3DPath", false, "Drive the car over the path to see the results in real-time")),
    m_eControlTarget(CreateCVar("controller.ControlTarget", (int)eTargetSimulation, "Drive the car over the path to see the results in real-time")),
    m_bControl3dPath(CreateUnsavedCVar("planner.Control3DPath", false, "Drive the car over the path to see the results in real-time")),
    m_bFuseImu(CreateCVar("planner.FuseImu", true)),
    m_nPathSegments(CreateCVar("planner.PathSegments", 20u, "how many segments to draw")),
    m_dTimeInterval(CreateCVar("planner.TimeInteval", 0.01, "")),
    m_pPlannerThread(0),
    m_pPhysicsThread(0),
    m_pControlThread(0),
    m_pCommandThread(0),
    m_pImuThread(0),
    m_pLocalizerThread(0),
    m_StillControl(true),
    m_StillRun(true),
    m_bLoggerEnabled(CreateUnsavedCVar("logger.Enabled", false, "")),

    m_sLogFileName(CreateUnsavedCVar("logger.FileName", std::string(), "/Users/crh/data/mocha.log")),
    m_bFusionLoggerEnabled(CreateUnsavedCVar("logger.FusionEnabled", false, "")),
    m_bLoggerPlayback(CreateUnsavedCVar("logger.Playback", false, "")),
    m_nNumControlSignals(0),
    m_nNumPoseUpdates(0),
    m_dPlaybackTimer(-1),
    m_Fusion(30,&m_DriveCarModel),
    m_bLoadWaypoints(CreateCVar("planner:LoadWaypoints", false, "Load waypoints on start.")),
    m_dPlanTime(Tic()) //the size of the fusion sensor
{
    m_Node.init("MochaGui");
}

////////////////////////////////////////////////////////////////
MochaGui::~MochaGui()
{
    for (list<GLCachedPrimitives*>::iterator iter = m_lPlanLineSegments.begin() ; iter != m_lPlanLineSegments.end() ; iter++) {
        delete *iter;
    }
    _KillThreads();
}

////////////////////////////////////////////////////////////////
MochaGui *MochaGui::GetInstance()
{
    if(g_pMochaGuiInstance == NULL) {
        g_pMochaGuiInstance = new MochaGui();
    }

    return g_pMochaGuiInstance;
}

////////////////////////////////////////////////////////////////
void MochaGui::Run() {
    //create the list of cvars not tsave
    std::vector<std::string> filter = { "not", "debug.MinLookaheadTime", "debug.MaxPlanTimeMultiplier", "debug.FreezeControl", "debug.PointCost",
                                        "debug.Show2DResult","debug.Optimize2DOnly","debug.ForceZeroStartingCurvature","debug.UseCentralDifferences",
                                        "debug.UseGoalPoseStepping","debug.DisableDamping","debug.MonotonicCost","debug.LocalizerDownsampler",
                                        "debug.ImuIntegrationOnly","debug.ShowJacobianPaths","debug.SkidCompensationActive","debug.PlaybackControlPaths",
                                        "debug.MaxLookaheadTime","debug.InertialControl","debug.StartSegmentIndex","planner.PointCostWeights",
                                        "planner.TrajCostWeights", "planner.Epsilon"};
    int frameCount = 0;
    double lastTime = Tic();
    m_dDiagTime = Tic();
    SetThreadName("Main thread");
    _StartThreads();
    double lastFrameTime = Tic();
    while( !pangolin::ShouldQuit() )
    {

      {
        boost::mutex::scoped_lock lock(m_DrawMutex);
        m_Gui.Render();
      }

        usleep(1e6/100.0);

        //calcualte Fps
        frameCount++;
        if(Toc(lastTime) > 0.5){
            m_dFps = (double)frameCount / (Toc(lastTime));
            frameCount = 0;
            lastTime =Tic();
        }




        if(m_bLoggerPlayback){
            _PlaybackLog(Toc(lastFrameTime));
        }else{
            _UpdateVisuals();
        }
        lastFrameTime = Tic();


        //save/load files if necessary
        if(m_bLoadWaypoints == true){
            m_bLoadWaypoints = false;
            CVarUtils::Load(m_vFileNames[m_nSelectedFileName],filter);
            sprintf(m_pSaveFileName,"%s",m_vFileNames[m_nSelectedFileName].c_str());
            _RefreshWaypoints();
            _UpdateWaypointFiles();
        }

        if(m_bSaveWaypoints == true){
            m_bSaveWaypoints = false;
            CVarUtils::Save(m_pSaveFileName,filter);
            _UpdateWaypointFiles();
        }
    }
}

////////////////////////////////////////////////////////////////
void MochaGui::_PlaybackLog(double dt)
{
    dt = 0.02;
    if(m_bPause == true){
        if(m_bStep == false){
            return;
        }else{
            dt = 0.005;
            m_bStep = false;
        }
    }
    if(m_dPlaybackTimer == -1){
        //the playback is just starting, so we must clear everything and set the segment samples
        _CommandFunc(eMochaClear);

        //now open a log file
        m_Logger.ReadLogFile(m_sPlaybackLogFile);
        msg_Log msg;
        m_Logger.ReadMessage(msg);
        if(msg.has_segments() == false){
            dout("Expected segments to be at the beggining of the log file, but the log packet did not have segments. Aborting playback.");
            m_dPlaybackTimer = -1;
            m_bLoggerPlayback = false;
            return;
        }

        //now get the segment samples and populate the segment samples array (this requires sthe arrays to have enough space)
        //m_vSegmentSamples.resize(std::max(m_vSegmentSamples.size(),msg.segments().segments_size()));
        for(int ii = 0; ii < msg.segments().segments_size() ; ii++){
            m_Logger.ReadMotionSample(m_vSegmentSamples[ii],msg.segments().segments(ii));

            Eigen::Vector3dAlignedVec vTraj;
            //calculate the actual trajectory
            for(const VehicleState& state : m_vSegmentSamples[ii].m_vStates) {
                vTraj.push_back(state.m_dTwv.translation());
            }
            m_vTerrainLineSegments[ii].AddVerticesFromTrajectory(vTraj);
        }
        //set the initial playback timer
        m_dPlaybackTimer = msg.timestamp();
    }else{
        msg_Log msg;
        m_dPlaybackTimer += dt;
        msg = m_Logger.GetLastMessage();
        //once we are done with this message, read a new one
        while(msg.timestamp() < m_dPlaybackTimer){
            if(m_Logger.ReadMessage(msg) == false){
                dout("End of log reached.");
                m_dPlaybackTimer = -1;
                m_bLoggerPlayback = false;
            }

            if(msg.has_vehiclestate()){
                dout("Read vehicle state at time " << m_dPlaybackTimer);
                VehicleState state;
                EventLogger::PoseUpdateSource source;
                m_Logger.ReadVehicleState(state,source,msg.vehiclestate());
                m_Gui.SetCarState(m_nDriveCarId,state,true);
            }else if(msg.has_controlplan()){
                dout("Read control plan at time " << m_dPlaybackTimer);
                ControlPlan plan;
                m_Logger.ReadControlPlan(plan,msg.controlplan());
                _UpdateControlPathVisuals(&plan);
                *m_Controller.GetLookaheadTimePtr() = plan.m_dEndTime-plan.m_dStartTime;
                //after reading the control plan,
            }else if(msg.has_controlcommand()){
                dout("sending command accel: " << msg.controlcommand().force() << " | steering: " << msg.controlcommand().phi());
                m_Logger.ReadCommand(m_ControlCommand,msg.controlcommand());
            }
        }
    }
}

////////////////////////////////////////////////////////////////
void MochaGui::_UpdateWaypointFiles()
{
    //get all .xml files in the directory first
    m_vFileNames.clear();

    DIR *d=opendir(".");
     if(d == NULL)
     {
       std::cout << "Couldn't open directory" << std::endl;
       return;
     }

     struct dirent *de=NULL;
     while((de = readdir(d))){
         std::string fileName(de->d_name);
         if(fileName.find(".xml") != std::string::npos ){
             m_vFileNames.push_back(fileName);
         }
     }

     closedir(d);
}

////////////////////////////////////////////////////////////////
void MochaGui::Init(std::string sRefPlane,std::string sMesh, bool bLocalizer, std::string sMode, std::string sLogFile)
{
    m_sPlaybackLogFile = sLogFile;

    m_Node.subscribe("herbie/Imu");

    m_dStartTime = Tic();
    m_bAllClean = false;
    m_bPause = false;
    m_bStep = false;

    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'c', std::bind(CommandHandler, eMochaClear ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + '1', std::bind(CommandHandler, eMochaToggleTrajectory ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + '2', std::bind(CommandHandler, eMochaTogglePlans ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'l', [this] {CarParameters::LoadFromFile(std::string(PARAMS_FILE_NAME),m_mDefaultParameters);} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 's', [this] {CarParameters::SaveToFile(std::string(PARAMS_FILE_NAME),m_mDefaultParameters);} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'r', std::bind(CommandHandler, eMochaRestart ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 't', std::bind(CommandHandler, eMochaSolve ) );
    //pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'v', std::bind(CommandHandler, eMochaPpmControl ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'b', std::bind(CommandHandler, eMochaSimulationControl ) );
    pangolin::RegisterKeyPressCallback( '\r', std::bind(CommandHandler, eMochaStep ) );
    pangolin::RegisterKeyPressCallback( ' ', std::bind(CommandHandler, eMochaPause ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'd', std::bind(&MochaGui::_LoadDefaultParams, this ) );
    pangolin::RegisterKeyPressCallback( 'G', [this] {this->m_pGraphView->Show(!this->m_pGraphView->IsShown());} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'f', [] {g_bFreezeControl = !g_bFreezeControl;} );


    //create CVars
    CVarUtils::CreateCVar("refresh", RefreshHandler, "Refresh all the waypoints.");
    CVarUtils::CreateCVar("SetWaypointVel", SetWaypointVelHandler, "Set a single velocity on all waypoints.");


    //initialize the heightmap
    //m_KHeightMap.LoadMap(DEFAULT_MAP_NAME,DEFAULT_MAP_IMAGE_NAME);
    //m_KHeightMap.SaveMap("room2.heightmap");

    //initialize the car model

    const aiScene *pScene = aiImportFile( sMesh.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );

    if(bLocalizer){
        pScene->mRootNode->mTransformation = aiMatrix4x4(1,0,0,0,
                                                         0,-1,0,0,
                                                         0,0,-1,0,
                                                         0,0,0,1);
    } else {
      pScene->mRootNode->mTransformation = aiMatrix4x4(1,0,0,0,
                                                         0,1,0,0,
                                                         0,0,-1,0,
                                                         0,0,0,1);
    }

    if(sMode == "Simulation"){
        m_eControlTarget = eTargetSimulation;
    }else{
        m_eControlTarget = eTargetExperiment;
    }

    btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);

    btTriangleMesh *pTriangleMesh = new btTriangleMesh();

    /// Using pTriangleMesh and the terrain mesh, fill in the gaps to create a static hull.
    BulletCarModel::GenerateStaticHull(pScene,pScene->mRootNode,pScene->mRootNode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);

    /// Now generate the collision shape from the triangle mesh --- to know where the ground is.
    btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);

    /// Also initialize a GLObject for the terrain mesh.
    m_TerrainMesh.Init(pScene);
    //m_MeshHeightMap.Init("ramp.blend");
    //m_GLHeightMap.Init(&m_ActiveHeightMap);


    /// Initialize the car parameters.
    CarParameters::LoadFromFile(PARAMS_FILE_NAME,m_mDefaultParameters);

    /// Generate as many new cars as we need in order to use MPC.
    m_LearningCarModel.Init(pCollisionShape,dMin,dMax, m_mDefaultParameters, REGRESSOR_NUM_WORLDS );
    m_PlanCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters,LocalPlanner::GetNumWorldsRequired(OPT_DIM) );
    m_ControlCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters, LocalPlanner::GetNumWorldsRequired(OPT_DIM)  );
    //parameters[CarParameters::SteeringCoef] = -700.0;

    /// As well as the car that we're actually driving.
    m_DriveCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters,1 );

    /// Set the two control commands we have to be the offsets calculated
    /// for the car; this should make it set to drive straight and not move.
    m_ControlCommand.m_dForce = m_mDefaultParameters[CarParameters::AccelOffset];
    m_ControlCommand.m_dPhi = m_mDefaultParameters[CarParameters::SteeringOffset];

    /// Set all of the ControlParams in MochaGui to be RegressionParameters that
    /// we grab from the ControlCarModel's CarParameters.
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::DynamicFrictionCoef,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::ControlDelay,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::SteeringCoef,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::SteeringOffset,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::AccelOffset,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::StallTorqueCoef,&m_ControlCarModel));
    //m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::SlipCoefficient,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::StaticSideFrictionCoef,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::MaxSteering,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::MaxSteeringRate,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::CompDamping,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::ExpDamping,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::Stiffness,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::SuspRestLength,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::MagicFormula_B,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::MagicFormula_C,&m_ControlCarModel));
    m_vControlParams.push_back(RegressionParameter(m_ControlCarModel.GetParameters(0),CarParameters::MagicFormula_E,&m_ControlCarModel));

    /// Attach CVars to each one of those parameters so we can manipulate them.
    for(RegressionParameter& param : m_vControlParams){
        std::string name("controller.Params.");
        name += param.m_sName;
        AttachCVar(name,&param,"");
    }

    /// Copy the MochaGui's ControlParams into the PlanCar.
    m_vPlannerParams = m_vControlParams;

    for(RegressionParameter& param : m_vPlannerParams){
        param.m_pModel = &m_PlanCarModel;
        std::string name("planner.Params.");
        name += param.m_sName;
        AttachCVar(name,&param,"");
    }

    m_vDriveParams = m_vControlParams;

    for(RegressionParameter& param : m_vDriveParams){
        param.m_pModel = &m_DriveCarModel;
        std::string name("sim.Params.");
        name += param.m_sName;
        AttachCVar(name,&param,"");
    }

    //initialize the debug drawer
    m_BulletDebugDrawer.Init(&m_DriveCarModel);

    /// Establish number of waypoints and where they are; set them to CVars.
    int numWaypoints = 8;
    for(int ii = 0; ii < numWaypoints ; ii++){
        char buf[100];
        snprintf( buf, 100, "waypnt.%d", ii );
        m_vWayPoints.push_back(&CreateGetCVar(std::string(buf), MatrixXd(8,1)));

        /// Linear waypoints.
        //    (*m_vWayPoints.back()) << ii*0.5,ii*0.5,0,0,0,0, 1,0;

        /// Waypoints in a circle.
        (*m_vWayPoints.back()) << sin(ii*2*M_PI/numWaypoints),
            cos(ii*2*M_PI/numWaypoints),
            0, 0, 0, -ii*2*M_PI/numWaypoints, 1,0;

        /// Load this segment ID into a vector that enumerates the path elements.
        m_Path.push_back(ii);
    }
    //close the loop
    m_Path.push_back(0);

    m_nStateStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);
    m_nLearningStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);
    m_nControlStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);
    m_nGravityStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);

    m_TerrainMesh.SetAlpha(1.0);
    m_Gui.Init(&m_TerrainMesh);

//    m_pGraphView = &pangolin::Plotter(&m_Log)
//            .SetBounds(0.0, 0.3, 0.6, 1.0);
//    pangolin::DisplayBase().AddDisplay(*m_pGraphView);

    m_nDriveCarId = m_Gui.AddCar(m_mDefaultParameters[CarParameters::WheelBase],m_mDefaultParameters[CarParameters::Width]);
    m_Gui.SetCarVisibility(m_nDriveCarId,true);

    //populate all the objects
    _PopulateSceneGraph();

    m_lPlanStates.resize(25);
    for (std::vector<VehicleState>*& vStates: m_lPlanStates) {
        vStates = new std::vector<VehicleState>();
    }


    Eigen::Matrix4d dT_localizer_ref = Eigen::Matrix4d::Identity();
    if(sRefPlane.empty() == false){
        std::string word;
        std::stringstream stream(sRefPlane);
        std::vector<double> vals;
        while( getline(stream, word, ',') ){
            vals.push_back(std::stod(word));
        }
        if(vals.size() != 16){
            std::cout << "Attempted to read in reference plane position, but incorrect number of matrix entries provided. Must be 16 numbers" << std::endl;
        }else{

            for(int ii = 0 ; ii < 4 ; ii++){
                for(int jj = 0 ; jj < 4 ; jj++){
                    dT_localizer_ref(ii,jj) = vals[ii*4 + jj];
                }
            }
            std::cout << "Ref plane matrix successfully read" << std::endl;
        }
    }

    m_sCarObjectName = "NinjaCar";
    m_Localizer.TrackObject(m_sCarObjectName, "posetonode",Sophus::SE3d(dT_localizer_ref).inverse());

    //initialize the panel
    m_GuiPanel.Init();

    //fusion parameters
    m_GuiPanel.SetVar("fusion:FilterSize",m_Fusion.GetFilterSizePtr())
              .SetVar("fusion:RMSE",m_Fusion.GetRMSEPtr())
              .SetVar("fusion:LocalizerFreq",&m_dLocalizerFreq)
              .SetVar("fusion:ImuFreq",&m_dImuFreq)
              .SetVar("fusion:Vel",&m_dVel)
              .SetVar("fusion:Pos",&m_dPos);

    //control parameters
    m_GuiPanel.SetVar("control:ControlOn",&m_bControl3dPath)
              .SetVar("control:ControlTarget",&m_eControlTarget)
              .SetVar("control:MaxControlPlanTime",m_Controller.GetMaxControlPlanTimePtr())
              .SetVar("control:PlansPerS",&m_dControlPlansPerS)
              .SetVar("control:VelocityError",&m_dVelError)
              .SetVar("control:Accel", &m_ControlCommand.m_dForce)
              .SetVar("control:Steering", &m_ControlCommand.m_dPhi)
              .SetVar("control:LookaheadTime",m_Controller.GetLookaheadTimePtr());


    //planner parameters
    m_GuiPanel.SetVar("planner:PlannerOn",&m_bPlannerOn)
              .SetVar("planner:ContinuousPlanning", &g_bContinuousPathPlanning)
              .SetVar("planner:SimulatePath",&m_bSimulate3dPath)
              .SetVar("planner:LoadWaypoints",&m_bLoadWaypoints)
              .SetVar("planner:SaveWaypoints",&m_bSaveWaypoints)
              .SetVar("planner:SaveFileName",(char*)&m_pSaveFileName)
              .SetVar("planner:SaveFileNameLen",&m_nSaveFileNameLen)
              .SetVar("planner:SelectedFileName",&m_nSelectedFileName)
              .SetVar("planner:FileNames",&m_vFileNames)
              .SetVar("planner:InertialControl",&g_bInertialControl);

    m_GuiPanel.SetVar("interface:Paused",&m_bPause)
              .SetVar("interface:Fps",&m_dFps);

    //update the file names
    _UpdateWaypointFiles();

    //CVarUtils::Load("cvars.xml");
    //_RefreshWaypoints();

    m_Fusion.ResetCurrentPose(Sophus::SE3d(),Eigen::Vector3d::Zero(),Eigen::Vector2d::Zero());
    Eigen::Vector6d T_ic;
    T_ic << -0.003988648232, 0.003161519861,  0.02271876324, -0.02824564077, -0.04132003806,   -1.463881523;
    m_Fusion.SetCalibrationPose(Sophus::SE3d(fusion::Cart2T(T_ic)));
    m_Fusion.SetCalibrationActive(false);
    //dout("Sucessfully initialized MochaGui object.");
    m_pControlLine = new GLCachedPrimitives();
}

////////////////////////////////////////////////////////////////
void MochaGui::_StartThreads()
{
    //recreate the threads
    m_pPlannerThread = new boost::thread(std::bind(&MochaGui::_PlannerFunc,this));
    m_pPhysicsThread = new boost::thread(std::bind(&MochaGui::_PhysicsFunc,this));
    m_pControlThread = new boost::thread(std::bind(&MochaGui::_ControlFunc,this));
    if(m_eControlTarget == eTargetExperiment){
        m_pCommandThread = new boost::thread(std::bind(&MochaGui::_ControlCommandFunc,this));
    }
    if(m_eControlTarget == eTargetSimulation){
        if(m_pImuThread != NULL){
            m_pImuThread->interrupt();
            m_pImuThread->join();
        }

        if(m_pLocalizerThread) {
            m_pLocalizerThread->interrupt();
            m_pLocalizerThread->join();
        }
    }else{
        m_Localizer.Start();

        if(m_pImuThread == NULL){
            m_pImuThread = new boost::thread(std::bind(&MochaGui::_ImuReadFunc,this));
        }

        if(m_pLocalizerThread == NULL){
            m_pLocalizerThread = new boost::thread(std::bind(&MochaGui::_LocalizerReadFunc,this));
        }
    }


}

void MochaGui::_KillController()
{
  m_StillControl = false;

    //reset the threads
    if(m_pControlThread) {
        m_pControlThread->join();
    }
    m_bControl3dPath = false;
    m_bControllerRunning = false;
}

////////////////////////////////////////////////////////////////
void MochaGui::_KillThreads()
{
    _KillController();

    m_StillRun = false;

    if(m_pPhysicsThread) {
        m_pPhysicsThread->join();
    }

    if(m_pPlannerThread) {
        m_pPlannerThread->join();
    }

    if(m_pLearningThread){
        m_pLearningThread->join();
    }

    if(m_pImuThread) {
        m_pImuThread->join();
    }

    if(m_pLocalizerThread) {
        m_pLocalizerThread->join();
    }

    if(m_pCommandThread) {
        m_pCommandThread->join();
    }

    delete m_pControlThread;
    delete m_pPhysicsThread;
    delete m_pPlannerThread;
    delete m_pImuThread;
    delete m_pLocalizerThread;
    delete m_pCommandThread;

    m_pControlThread = 0;
    m_pPhysicsThread = 0;
    m_pPlannerThread = 0 ;
    m_pImuThread = 0;
    m_pLocalizerThread = 0;

    m_Localizer.Stop();
}


////////////////////////////////////////////////////////////////
bool MochaGui::_SetWaypointVel(std::vector<std::string> *vArgs)
{
    if(vArgs->size() == 0){
        return false;
    }
    double dNewSpeed = atof(vArgs->at(0).c_str());
    //go through all the waypoints and sset the speed
    for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
        m_Gui.GetWaypoint(ii)->m_Waypoint.SetVelocity(dNewSpeed);
    }
    return true;
}

////////////////////////////////////////////////////////////////
bool MochaGui::_Refresh(std::vector<std::string> *vArgs)
{

    _KillThreads();

    //refresh the waypoints first
    _RefreshWaypoints();

    m_Gui.SetWaypointDirtyFlag(true);

    _StartThreads();
    return true;
}

////////////////////////////////////////////////////////////////
void MochaGui::_RefreshWaypoints()
{
    //first clear all waypoints
    m_Gui.ClearWaypoints();

    //add waypoints for any newly added path points
    for (size_t ii = 0; ii < m_Path.size(); ii++) {
        Eigen::Vector6d dPose = (*m_vWayPoints[m_Path[ii]]).block<6,1>(0,0);
        if(m_Gui.WaypointCount() <= (int)m_Path[ii])
        {
            m_Gui.AddWaypoint(dPose,(*m_vWayPoints[m_Path[ii]])(WAYPOINT_VEL_INDEX));
        }
    }

    for(size_t ii = 0; ii < m_vGLLineSegments.size() ; ii++){
        m_vGLLineSegments[ii].Clear();
    }

    for(size_t ii = 0; ii < m_vTerrainLineSegments.size() ; ii++){
        m_vTerrainLineSegments[ii].Clear();
    }

    m_vSegmentSamples.resize(m_Path.size()-1);
    m_vGLLineSegments.resize(m_Path.size()-1);
    m_vTerrainLineSegments.resize(m_Path.size()-1);
}

////////////////////////////////////////////////////////////////
bool MochaGui::_CommandFunc(MochaCommands command) {

    switch (command){
    case eMochaSolve:
        m_Controller.Reset();
        CVarUtils::Load("cvars.xml");
        _RefreshWaypoints();
        m_bPlannerOn = true;
        m_bAllClean = false;
        m_bControl3dPath = true;

        break;

    case eMochaSimulationControl:
        m_eControlTarget = eTargetSimulation;
        m_bControl3dPath = true;
        m_bSimulate3dPath = true;
        break;

    case eMochaPpmControl:
        m_eControlTarget = eTargetExperiment;
        m_bControl3dPath = true;
        break;

    case eMochaPause:
        m_bPause = !m_bPause;
        break;

    case eMochaStep:
        m_bStep = true;
        break;

    case eMochaRestart:
        m_bControl3dPath = false;
        while(m_bControllerRunning == true) {
            usleep(1000);
        }
        {
            //boost::mutex::scoped_lock lock(m_DrawMutex);
            m_pControlLine->Clear();
            for (list<GLCachedPrimitives*>::iterator iter = m_lPlanLineSegments.begin() ; iter != m_lPlanLineSegments.end() ; iter++) {
                (*iter)->Clear();
            }
        }
        m_bSimulate3dPath = false;

        //restart the controller
        //m_bControl3dPath = true;
        break;

    case eMochaToggleTrajectory:
        //m_CarLineSegments.SetVisible(!m_CarLineSegments.IsVisible());
        break;

    case eMochaTogglePlans:
        {
            //boost::mutex::scoped_lock lock(m_DrawMutex);
            for (list<GLCachedPrimitives*>::iterator iter = m_lPlanLineSegments.begin() ; iter != m_lPlanLineSegments.end() ; iter++) {
                (*iter)->SetVisible(!(*iter)->IsVisible());
            }
        }
        break;

    case eMochaClear:
        {
            //boost::mutex::scoped_lock lock(m_DrawMutex);
            m_Gui.ClearCarTrajectory(m_nDriveCarId);
            m_pControlLine->Clear();
            for (GLCachedPrimitives*& strip: m_lPlanLineSegments) {
                strip->Clear();
            }

            for(std::vector<VehicleState>*& pStates : m_lPlanStates){
                pStates->clear();
            }
        }
        break;
    default:
        break;
    }


    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////

bool MochaGui::_IteratePlanner(
        LocalProblem& problem,
        //const GLWayPoint* pStart,
        MotionSample& sample, //< Output:
        Eigen::Vector3dAlignedVec& vActualTrajectory,
        Eigen::Vector3dAlignedVec& vControlTrajectory,
        bool only2d /*= false*/)
{
    bool res = false;

    //m_Planner.StressTest(problem);

    if(only2d == false) {
        if( m_bPlannerOn == true && m_bSimulate3dPath == false ){
            res = m_Planner.Iterate(problem);            
            m_Planner.SimulateTrajectory(sample,problem,0,true);
        }else{
            res = true;
            m_Planner.SimulateTrajectory(sample,problem,0,true);
            if(problem.m_bInertialControlActive){
                m_Planner.CalculateTorqueCoefficients(problem,&sample);
                m_Planner.SimulateTrajectory(sample,problem,0,true);
            }

        }

        //calculate the actual trajectory
        for(const VehicleState& state : sample.m_vStates) {
            vActualTrajectory.push_back(state.m_dTwv.translation());
        }
    }

    //and also the sample trajectory
    m_Planner.SamplePath(problem,vControlTrajectory);

    return res;
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_UpdateVisuals()
{
    if(g_bPlaybackControlPaths == false || m_bPause == false){
        VehicleState state;
        if(m_bSimulate3dPath == true ){
            m_DriveCarModel.GetVehicleState(0,state);
        }else{
            _UpdateVehicleStateFromFusion(state);
        }

        //offset the chasis by the height offset (constant)
        m_Gui.SetCarState(m_nDriveCarId,state,(m_bSimulate3dPath == true || m_eControlTarget == eTargetExperiment));
        //Eigen::Vector6d pose = fusion::T2Cart(state.m_dTwv.matrix());
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
bool MochaGui::_UpdateControlPathVisuals(const ControlPlan* pPlan)
{
    //boost::mutex::scoped_lock lock(m_DrawMutex);
    //get the current plans and draw them
    Sophus::SE3d dTwv1,dTwv2;
    *m_lPlanStates.front() = pPlan->m_Sample.m_vStates;
    if(m_lPlanStates.front()->size() !=0) {
        //push everything back
        GLCachedPrimitives* pStrip = m_lPlanLineSegments.front();
        m_lPlanLineSegments.pop_front();
        std::vector<VehicleState>* vStates = m_lPlanStates.front();
        m_lPlanStates.pop_front();

        m_lPlanLineSegments.push_back(pStrip);
        m_lPlanStates.push_back(vStates);
        Eigen::Vector3dAlignedVec vPts;
        for(const VehicleState& state : *vStates){
            vPts.push_back(state.m_dTwv.translation());
        }
        m_lPlanLineSegments.back()->AddVerticesFromTrajectory(vPts);
        m_lPlanLineSegments.back()->SetColor(GLColor::HsvColor(m_dCurrentHue,1.0,0.8));
        m_lPlanLineSegments.back()->SetIgnoreDepth(true);
        m_dCurrentHue += 0.05;
        if(m_dCurrentHue > 1.0){
            m_dCurrentHue = 0;
        }



        //create a line that points to the intended destination
        m_pControlLine->Clear();
        m_pControlLine->AddVertex(pPlan->m_StartState.m_dTwv.translation());
        m_pControlLine->AddVertex(pPlan->m_GoalState.m_dTwv.translation());
        m_pControlLine->SetColor(GLColor(1.0f,1.0f,1.0f));
        m_pControlLine->SetIgnoreDepth(true);

        m_DestAxis.SetPose(pPlan->m_GoalState.m_dTwv.matrix());
        m_DestAxis.SetAxisSize(pPlan->m_GoalState.m_dV.norm());
        return true;
    }else{
        return false;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_UpdateVehicleStateFromFusion(VehicleState& currentState)
{
    if(m_bFuseImu == true){
        if(g_bProcessModelActive == false){
            fusion::PoseParameter currentPose = m_Fusion.GetCurrentPose();
            currentState.m_dTwv = currentPose.m_dPose;
            currentState.m_dV = currentPose.m_dV;
            currentState.m_dW = currentPose.m_dW;
            m_dVel = currentState.m_dV.norm();
            m_dPos = currentState.m_dTwv.translation();
        }else{
            m_Fusion.GetVehicleState(currentState);
        }

    }else {
        currentState.m_dTwv = m_Fusion.GetLastGlobalPose().m_dPose;
    }

    currentState.m_dV = GetBasisVector(currentState.m_dTwv,0).normalized()*currentState.m_dV.norm();
    currentState.m_dW[0] = currentState.m_dW[1] = 0;

    Eigen::Vector3d dIntersect;
    if(m_DriveCarModel.RayCast(currentState.m_dTwv.translation(),GetBasisVector(currentState.m_dTwv,2)*0.2,dIntersect,true)){
       currentState.m_dTwv.translation() = dIntersect;
    }

    //update the wheels
    if(g_bProcessModelActive == false){
        currentState.UpdateWheels(m_DriveCarModel.GetWheelTransforms(0));
    }

    if(m_bLoggerEnabled && m_Logger.IsReady()){
        m_Logger.LogPoseUpdate(currentState,EventLogger::eLocalizer);
    }

    //dout("Theta: " << currentState.GetTheta());
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_LocalizerReadFunc()
{
    double lastTime = Tic();
    int numPoses = 0;
    int counter = 1;
    //double localizerTime = 0;
    m_Localizer.Start();

    while(1){
        //this is a blocking call
        double localizerTime;
        const Sophus::SE3d Twb = m_Localizer.GetPose(m_sCarObjectName,true,&localizerTime);

        double sysTime = Tic();

        double dt = Tic()-lastTime;
        if(dt > 0.5){
            m_dLocalizerFreq = numPoses/dt;
            lastTime = Tic();
            numPoses = 0;
        }
        numPoses++;

        if(g_bImuIntegrationOnly == false){
            counter++;
            //double dTemp = Tic();
            //if(counter > 1){
            //std::cout << "Global pose received at:" << sysTime << "seconds." << std::endl;
            if(g_bProcessModelActive){
                ControlCommand command;
                {
                    boost::mutex::scoped_lock lock(m_ControlMutex);
                    command = m_ControlCommand;
                }
                m_Fusion.RegisterGlobalPoseWithProcessModel(Twb,sysTime,sysTime,command);
            }else{
                m_Fusion.RegisterGlobalPose(Twb,sysTime,sysTime);
            }
            //dout("Fusion process took" << Toc(dTemp) << " seconds.");
            counter = 0;



            if(m_FusionLogger.IsReady() && m_bFusionLoggerEnabled){
                m_FusionLogger.LogLocalizerData(Tic(),localizerTime,Twb);
            }

        }
        //}
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_ImuReadFunc()
{
    double lastTime = Tic();
    int numPoses = 0;

    std::cout << "ImuReadFunc currently disabled." << std::endl;

    /*
    while(1){
        Imu_Accel_Gyro Msg;
        //this needs to be a blocking call .. not sure it is!!!
        if(m_Node.receive("herbie/Imu",Msg)){
            double dImuTime = (double)Msg.timer()/62500.0;
            double sysTime = Tic();
            m_Fusion.RegisterImuPose(Msg.accelx()*G_ACCEL,Msg.accely()*G_ACCEL,Msg.accelz()*G_ACCEL,
                                     Msg.gyrox(),Msg.gyroy(),Msg.gyroz(),sysTime,sysTime);

            if(m_FusionLogger.IsReady() && m_bFusionLoggerEnabled){
                m_FusionLogger.LogImuData(sysTime,dImuTime,Eigen::Vector3d(Msg.accelx(),Msg.accely(),Msg.accelz()),Eigen::Vector3d(Msg.gyrox(),Msg.gyroy(),Msg.gyroz()));
            }
            //std::cout << "IMU pose received at:" << sysTime << "seconds [" << Msg.accely() << " " <<  -Msg.accelx() << " " << Msg.accelz() << std::endl;
//            if(m_bLoggerEnabled && m_Logger.IsReady()){
//                VehicleState state;
//                _UpdateVehicleStateFromFusion(state);
//                m_Logger.LogPoseUpdate(state,EventLogger::eIMU);
//            }

            double dt = Tic()-lastTime;
            if(dt > 0.5){
                m_dImuFreq = numPoses/dt;
                lastTime = Tic();
                numPoses = 0;
            }
            numPoses++;
        }
    }
    */
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_ControlCommandFunc()
{
    double dLastTime = Tic();
    while(1)
    {
        //only go ahead if the controller is running, and we are targeting the real vehicle
        if(m_eControlTarget == eTargetExperiment && m_bControllerRunning == true && m_bSimulate3dPath == false){
            //get commands from the controller, apply to the car and update the position
            //of the car in the controller
            {
                boost::mutex::scoped_lock lock(m_ControlMutex);
                double dCurrentTime = Tic();
                m_Controller.GetCurrentCommands(dCurrentTime,
                                                m_ControlCommand,
                                                m_vTargetVel,
                                                m_dTargetPose);

                //TODO: This is in fact incorrect. The dT specified here is from the last command to the
                //current command, not from the current command to the next
                m_ControlCommand.m_dT = dCurrentTime - dLastTime;
                dLastTime = dCurrentTime;
            }

            if(m_bLoggerEnabled && m_Logger.IsReady()){
                m_Logger.LogControlCommand(m_ControlCommand);
            }

            if(g_bProcessModelActive){
                m_Fusion.RegisterInputCommands(m_ControlCommand);
            }else{
                //this update does not actually run the simulation. It only serves to push in
                //a delayed command and update the steering (so we know where the steering value
                //is when we pass the state to the controller)
                m_DriveCarModel.UpdateState(0,m_ControlCommand,m_ControlCommand.m_dT,false,true);
            }

            //send the commands to the car via node
            m_nNumPoseUpdates++;
            CommandMsg Req;
            CommandReply Rep;

            Req.set_accel(std::max(std::min(m_ControlCommand.m_dForce,500.0),0.0));
            Req.set_phi(m_ControlCommand.m_dPhi);
            double time = Tic();
            //m_Node.call_rpc( "ninja_commander/ProgramControlRpc",Req,Rep,100 );
            m_dControlDelay = Toc(time);
        }

        //about 100 commands/sec
        usleep(1E6 * 0.008);
        //usleep(1E6 * 0.016);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_ControlFunc()
{
    int nNumConrolPlans = 0;
    m_dControlPlansPerS = 0;
    double dLastTime = Tic();

    std::vector<std::string> vLabels = {"Lookahead","Accel","Norm", "Vel.", "Z Vel.", "Target Vel."};
    m_Log.SetLabels(vLabels);

    SetThreadName("Control thread");
    try
    {
        m_bControllerRunning = false;

        while(m_StillControl) {

            m_pControlLine->Clear();
            for (GLCachedPrimitives*& pStrip: m_lPlanLineSegments) {
                pStrip->Clear();
            }
            m_bControllerRunning = false;

            //unlock waypoints
            //lock all the waypoints
            for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
                m_Gui.GetWaypoint(ii)->m_Waypoint.SetLocked(false);
            }

            //wait for the start command
            //        dout("Waiting for control input and completed plan");
            while(m_bControl3dPath == false || m_bAllClean == false ||
                  (m_bSimulate3dPath == false && m_eControlTarget != eTargetExperiment)){
                usleep(1000);
            }

            //write the segments to the log
            if(m_bLoggerEnabled){
                //create a new log and close the last if need be
                m_sPlaybackLogFile = m_Logger.OpenNewLogFile("");
                m_Logger.WriteSegmentSamples(m_vSegmentSamples);
                m_Logger.SetIsReady(true);
                dout("Created new log file at " << m_sPlaybackLogFile << ". Enabling logging.");
            }

            if(m_bFusionLoggerEnabled){
                m_FusionLogger.OpenNewLogFile("","fusion_");
                m_FusionLogger.SetIsReady(true);
            }

            //make the car visible
            m_Gui.SetCarVisibility(m_nDriveCarId,true);

            //put the drive car at the beginning of the trajectory
            VehicleState startingState = m_vSegmentSamples[g_nStartSegmentIndex].m_vStates[0];
            startingState.m_dV = Eigen::Vector3d::Zero();
            startingState.m_dW = Eigen::Vector3d::Zero();
            startingState.m_dTwv.translation() += GetBasisVector(startingState.m_dTwv,1)*0.25;
            m_DriveCarModel.SetState(0,startingState);
            m_DriveCarModel.ResetCommandHistory(0);

            m_Controller.Init(m_vSegmentSamples,&m_Planner,&m_ControlCarModel,m_dTimeInterval);

            //lock all the waypoints
            for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
                m_Gui.GetWaypoint(ii)->m_Waypoint.SetLocked(true);
            }

            m_dCurrentHue = 0;
            //reset the control commands
            //m_ControlCommand = ControlCommand();
            m_bControllerRunning = true;
            VehicleState currentState;
            while(m_StillControl)
            {
                while((m_eControlTarget != eTargetExperiment && m_bSimulate3dPath == false
                       && m_StillControl )){
                    usleep(1000);
                }



                if(m_bSimulate3dPath ){
                    m_DriveCarModel.GetVehicleState(0,currentState);
                    //get the current position and pass it to the controller
                    m_Controller.SetCurrentPoseFromCarModel(&m_DriveCarModel,0);
                }else if(m_eControlTarget == eTargetExperiment){
                    //get current pose from the fusion routine
                    _UpdateVehicleStateFromFusion(currentState);
                    //set the command history and current pose on the controller
                    {
                        boost::mutex::scoped_lock lock(m_ControlMutex);
                        m_Controller.SetCurrentPose(currentState,&m_DriveCarModel.GetCommandHistoryRef(0));
                    }
                }else{
                    //we have no business being here
                    assert(false);
                }

                //now run the controller to crete a plan
                ControlPlan* pPlan;
                if(m_bPause == false && g_bInfiniteTime == false){
                    m_dPlanTime = Tic();
                }
                m_Controller.PlanControl(m_dPlanTime,pPlan);

                if(pPlan != NULL && m_bLoggerEnabled && m_Logger.IsReady()){
                    m_Logger.LogControlPlan(*pPlan);
                }



//                //calculate error metrics
//                m_dTargetVel = dTargetVel.norm();
                m_dVelError = m_vTargetVel.norm() - currentState.m_dV.norm();
//                m_Log.Log(*m_Controller.GetLookaheadTimePtr(),m_ControlCommand.m_dForce/500.0,pPlan != NULL ? pPlan->m_dNorm : -1,
//                          currentState.m_dV.norm(),currentState.m_dV[2],m_vTargetVel.norm());
                 m_Log.Log(m_ControlCommand.m_dPhi/500.0);

//                m_dPoseError = (dTargetPose*currentState.m_dTwv.inverse()).log().norm();

                if(pPlan != NULL) {
                    _UpdateControlPathVisuals(pPlan);
                    //update control plan statistics if there is a new control plan available
                    nNumConrolPlans++;
                    if(Toc(dLastTime) > 0.5){
                        m_dControlPlansPerS = (double)nNumConrolPlans / (Toc(dLastTime));
                        nNumConrolPlans = 0;
                        dLastTime =Tic();
                    }
                }

                //if the controller has been terminated, exit the inner loop
                if( m_bControl3dPath == false ){
                    m_ControlCommand.m_dForce = m_mDefaultParameters[CarParameters::AccelOffset];
                    m_ControlCommand.m_dPhi = m_mDefaultParameters[CarParameters::SteeringOffset];
                    //close the log file if need be
                    m_Logger.CloseLogFile();
                    m_FusionLogger.CloseLogFile();
                    m_Controller.Reset();
                    m_bControllerRunning = false;
                    dout("Controller terminated. Closing log file if needed.");
                    break;
                }
            }

            //unlock all the waypoints
            for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
                m_Gui.GetWaypoint(ii)->m_Waypoint.SetLocked(false);
            }

            usleep(10000);
        }
    }catch(...){
        dout("Control thread exception caught...");
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_LoadDefaultParams()
{
    for(RegressionParameter& param : m_vControlParams){
        param.m_dVal = m_mDefaultParameters[param.m_nKey];
    }
    m_ControlCarModel.UpdateParameters(m_vControlParams);

    for(RegressionParameter& param : m_vPlannerParams){
        param.m_dVal = m_mDefaultParameters[param.m_nKey];
    }
    m_PlanCarModel.UpdateParameters(m_vPlannerParams);

    for(RegressionParameter& param : m_vDriveParams){
        param.m_dVal = m_mDefaultParameters[param.m_nKey];
    }
    m_DriveCarModel.UpdateParameters(m_vDriveParams);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_PhysicsFunc()
{
    SetThreadName("Physics thread");

    double currentDt;
    int nCurrentSample = 0;
    int nCurrentSegment = 0;
    double dCurrentTic = -1;
    double dPlanTimer = 0;

    while(m_StillRun){

        while(m_bSimulate3dPath == false && m_StillRun){
            dCurrentTic = Tic();
            usleep(10000);
        }

        //this is so pausing doesn't shoot the car in the air
        double pauseStartTime = Tic();
        while(m_bPause == true && m_bSimulate3dPath == true && m_StillRun) {
            usleep(1000);

            //This section will play back control paths, if the user has elected to do so
            if(g_bPlaybackControlPaths){
                //then we need to play through the control paths
                for(std::vector<VehicleState>*& pStates: m_lPlanStates){
                    for(const VehicleState& state: *pStates){
                        while(m_bPause){
                            usleep(10000);
                            if(m_bStep){
                                m_bStep = false;
                                break;
                            }
                        }
                        m_Gui.SetCarState(m_nDriveCarId,state);
                        //m_DriveCarModel.SetState(0,state);
                        usleep(1E6*0.01);
                    }
                }
                m_bPause = true;
                continue;
            }
            if(m_bStep == true){
                m_bStep = false;
                break;
            }
        }
        dCurrentTic += Toc(pauseStartTime); //subtract the pause time

        if(m_bSimulate3dPath == true) {
            //m_Gui.SetCarVisibility(m_nDriveCarId,true);

            //calculate the current dT
            if(dCurrentTic < 0  && m_bControl3dPath == false ) {
                dCurrentTic = Tic();
                VehicleState startState = m_vSegmentSamples[0].m_vStates[0];
                //startState.m_dV = Eigen::Vector3d::Zero();
                m_DriveCarModel.SetState(0,startState);

            }


            //update the drive car position based on the car model
            ControlCommand currentCommand;
            if(m_bControl3dPath) {
                if(m_bControllerRunning){

                    while((Toc(dCurrentTic)) < 0.002) {
                        usleep(100);
                    }
                    currentDt = Toc(dCurrentTic);
                    dCurrentTic = Tic();

                    if(m_bPause == true){
                        currentDt = m_dTimeInterval;
                    }

                    if(g_bInfiniteTime == true){
                        double timeDelta = (m_dPlanTime-m_Controller.GetLastPlanStartTime());
                        if(timeDelta < 0){
                            m_dPlanTime = m_Controller.GetLastPlanStartTime();
                        }
                        double lookaheadLimit = 1.0/10.0;//(*m_Controller.GetLookaheadTimePtr()/3.0);
                        if(timeDelta > lookaheadLimit){
                            continue;
                        }
                    }

                    //push forward the plan time
                    dPlanTimer += currentDt;

                    //get the current commands
                    {
                        boost::mutex::scoped_lock lock(m_ControlMutex);

                        if(m_bPause == false && g_bInfiniteTime == false){
                            m_dPlanTime = Tic();
                        }


                        m_Controller.GetCurrentCommands(m_dPlanTime,
                                                        m_ControlCommand,
                                                        m_vTargetVel,
                                                        m_dTargetPose);

                        currentCommand = m_ControlCommand;
                    }
                    //dout("Sending accel: "<< currentCommand.m_dForce << " and steering: " << currentCommand.m_dPhi);

                    if(m_bLoggerEnabled && m_Logger.IsReady()){
                        m_Logger.LogControlCommand(currentCommand);
                    }

                    m_DriveCarModel.UpdateState(0,currentCommand,currentDt);

                    m_dPlanTime = m_dPlanTime + currentDt;

                    currentCommand.m_dT = currentDt;
                    VehicleState currentState;
                    m_DriveCarModel.GetVehicleState(0,currentState);

                    if(m_bLoggerEnabled && m_Logger.IsReady()){
                        m_Logger.LogPoseUpdate(currentState,EventLogger::eSimulation);
                    }
                }
            }else{
                //dout("Simulating with force " << m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample].m_dForce <<
                //     " and accel " << m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample].m_dPhi <<
                //     " and torque " << m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample].m_dTorques.transpose());
                m_DriveCarModel.UpdateState(0,m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample],
                                            m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample].m_dT,
                                            true); //no delay for simulation
                currentCommand = m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample];

                //wait until we have reached the correct dT to
                //apply the next set of commands
                while((Toc(dCurrentTic)) < m_vSegmentSamples[nCurrentSegment].m_vCommands[nCurrentSample].m_dT) {
                    usleep(100);
                }
                dCurrentTic = Tic();

                //progress to the next pose
                nCurrentSample++;
                if( nCurrentSample >= (int)m_vSegmentSamples[nCurrentSegment].m_vCommands.size()) {
                    nCurrentSegment++;
                    //we can progress to the next segment
                    if(nCurrentSegment >= (int)m_vSegmentSamples.size()) {
                        nCurrentSegment = 0;
                    }

                    nCurrentSample = 0;
                    m_DriveCarModel.SetState(0,m_vSegmentSamples[nCurrentSegment].m_vStates[0]);
                    //reset the command history
                    m_DriveCarModel.ResetCommandHistory(0);
                }
            }

        }else {
            //m_GLDriveCar.SetInVisible();
            nCurrentSegment = 0;
            nCurrentSample = 0;
            dCurrentTic = -1;
            usleep(1000);
        }
    }
}

////////////////////////////////////////////////////////////////
void MochaGui::_PlannerFunc() {
    SetThreadName("Planning thread");
    int numInterations = 0;
    m_bPlanning = false;
    // now add line segments
    bool previousOpenLoopSetting = m_bPlannerOn;
    while (1) {
        boost::this_thread::interruption_point();

        //if the use has changed the openloop setting, dirty all
        //the waypoints
        if( previousOpenLoopSetting != m_bPlannerOn) {
            m_Gui.SetWaypointDirtyFlag(true);
            previousOpenLoopSetting = m_bPlannerOn;
        }

        std::vector<int> vDirtyIds;
        bool allClean = true;
        for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
            int nStartIdx = m_Path[ii];
            int nEndIdx = m_Path[ii + 1];
            GLWayPoint* a = &m_Gui.GetWaypoint(nStartIdx)->m_Waypoint;
            GLWayPoint* b = &m_Gui.GetWaypoint(nEndIdx)->m_Waypoint;

            //make sure both waypoints have well defined poses (no nans)
            if(std::isfinite(a->GetPose5d().norm()) == false || std::isfinite(b->GetPose5d().norm()) == false )
            {
                continue;
            }

            if (a->GetDirty() || b->GetDirty() ) {
                allClean = false;
                //if the controller is running, we have to stop it
                if(m_bControllerRunning){
                    m_bControl3dPath = false;
                    while(m_bControllerRunning == true) {
                        usleep(1000);
                    }
                }

                m_bPlanning = true;
                Eigen::Vector3d dIntersect;
                Eigen::Vector6d newPose;

                {
                    //lock the drawing look as we will be modifying things here
                    boost::mutex::scoped_lock lock(m_DrawMutex);
                    if(a->GetDirty()) {
                        Sophus::SE3d pose(a->GetPose4x4_po());
                        if(m_DriveCarModel.RayCast(pose.translation(),GetBasisVector(pose,2)*0.2,dIntersect,true)){
                            pose.translation() = dIntersect;
                            a->SetPose(pose.matrix());
                        }
                        *m_vWayPoints[nStartIdx] << a->GetPose(), a->GetVelocity(), a->GetAerial();
                    }
                    if(b->GetDirty()) {
                        Sophus::SE3d pose(b->GetPose4x4_po());
                        if(m_DriveCarModel.RayCast(pose.translation(),GetBasisVector(pose,2)*0.2,dIntersect,true)){
                            pose.translation() = dIntersect;
                            b->SetPose(pose.matrix());
                        }
                        *m_vWayPoints[nEndIdx] << b->GetPose(), b->GetVelocity(), a->GetAerial();
                    }
                }

                //iterate the planner
                VehicleState startState(Sophus::SE3d(a->GetPose4x4_po()),a->GetVelocity(),0);
                VehicleState goalState(Sophus::SE3d(b->GetPose4x4_po()),b->GetVelocity(),0);

//                //do pre-emptive calculation of start/end curvatures by looking at the prev/next states
//                GLWayPoint* pPrevWaypoint = &m_Gui.GetWaypoint((ii ==  0) ? m_Path[m_Path.size()-2] : m_Path[ii-1])->m_Waypoint;
//                VehicleState prevState = VehicleState(Sophus::SE3d(pPrevWaypoint->GetPose4x4_po()),pPrevWaypoint->GetVelocity());
//                GLWayPoint* pNextWaypoint = &m_Gui.GetWaypoint((ii >=  m_Path.size()-1) ? m_Path[0] : m_Path[ii+2])->m_Waypoint;
//                VehicleState nextState = VehicleState(Sophus::SE3d(pNextWaypoint->GetPose4x4_po()),pNextWaypoint->GetVelocity());

//                //now calculate curvatures for each side
//                double startToGoalCurvature = rpg::AngleWrap(goalState.GetTheta()-startState.GetTheta())/(goalState.m_dTwv.translation().head(2) - startState.m_dTwv.translation().head(2)).norm() ;
//                startState.m_dCurvature = (rpg::AngleWrap(startState.GetTheta()-prevState.GetTheta())/(startState.m_dTwv.translation().head(2) - prevState.m_dTwv.translation().head(2)).norm() +
//                                          startToGoalCurvature)/2;
//                goalState.m_dCurvature = (startToGoalCurvature +
//                                         rpg::AngleWrap(nextState.GetTheta()-goalState.GetTheta())/(nextState.m_dTwv.translation().head(2) - goalState.m_dTwv.translation().head(2)).norm())/2;


//                startState.m_dCurvature = std::isfinite(startState.m_dCurvature) ? startState.m_dCurvature : 0;
//                goalState.m_dCurvature = std::isfinite(goalState.m_dCurvature) ? goalState.m_dCurvature : 0;

                //do connected planning if possibles
                if(g_bContinuousPathPlanning == true){
                    if( !(ii == 0 || m_vSegmentSamples[ii-1].m_vStates.empty() == true || m_bPlannerOn == false) ){
                        startState = m_vSegmentSamples[ii-1].m_vStates.back();
                        startState.m_dCurvature = m_vSegmentSamples[ii-1].m_vCommands.back().m_dCurvature;
                    }
                }

                ApplyVelocitesFunctor5d f(&m_PlanCarModel, Eigen::Vector3d::Zero(), NULL);
                f.SetNoDelay(true);
                LocalProblem problem(&f,startState,goalState,m_dTimeInterval);
                m_Planner.InitializeLocalProblem(problem,0,NULL,eCostPoint);
                problem.m_bInertialControlActive = g_bInertialControl;
                //problem.m_lPreviousCommands = previousCommands;

                bool success = false;
                while(success == false && m_StillRun){
                    //if we are paused, then wait
                    while(m_bPause == true && m_StillRun) {
                        usleep(1000);
                        if(m_bStep == true){
                            m_bStep = false;
                            break;
                        }
                    }

                    Eigen::Vector3dAlignedVec vActualTrajectory, vControlTrajectory;
                    bool res;
                    if(numInterations > g_nIterationLimit){
                        dout("Reached iteration limit. Skipping");
                        success = true;
                        numInterations = 0;
                    }else{
                        //dout("Distance delta is " << problem.m_dDistanceDelta);
                        res = _IteratePlanner(problem,m_vSegmentSamples[ii],vActualTrajectory,vControlTrajectory);
                        numInterations++;

                        //lock the mutex as this next bit will modify object
                        boost::mutex::scoped_lock lock(m_DrawMutex);

                        if (m_bCompute3dPath == true ) {
                            success =  res;
                            m_vGLLineSegments[ii].AddVerticesFromTrajectory(vControlTrajectory);
                            m_vTerrainLineSegments[ii].AddVerticesFromTrajectory(vActualTrajectory);
                        }else {
                            success = true;
                        }

                        // draw simple 2d path projected onto surface first since it's fast
                        if (m_bShowProjectedPath) {
                            m_vGLLineSegments[ii].AddVerticesFromTrajectory(vControlTrajectory);
                        }else {
                            m_vGLLineSegments[ii].Clear();
                        }
                    }

                }

                if( success == true  ){
                    numInterations = 0;
                    vDirtyIds.push_back(nStartIdx);
                    vDirtyIds.push_back(nEndIdx);
                }
            }
        }

        //if there are no more to be done, mark the all clean
        if(allClean == true) {
            numInterations = 0;
            m_bAllClean = true;
            m_bPlanning = false;
        }else{
            m_bAllClean = false;
        }

        // mark all moved waypints as clean
        m_Gui.SetWaypointDirtyFlag(false);
        vDirtyIds.clear();

        if(m_bPlanning == false) {
            usleep(10000);
        }
    }
}

////////////////////////////////////////////////////////////////
void MochaGui::_PopulateSceneGraph() {
  _RefreshWaypoints();

  m_vGLLineSegments.resize(m_Path.size() - 1);
  for (size_t ii = 0; ii < m_vGLLineSegments.size(); ii++) {
    m_vGLLineSegments[ii].SetColor(GLColor(0.0f,0.0f,0.0f));
    m_vGLLineSegments[ii].SetIgnoreDepth(true);
    m_Gui.AddGLObject(&m_vGLLineSegments[ii],true);
  }

  // also add segments for curve that takes terrain into account
  m_vTerrainLineSegments.resize(m_Path.size() - 1);
  for (size_t ii = 0; ii < m_vTerrainLineSegments.size(); ii++) {
    m_Gui.AddGLObject(&m_vTerrainLineSegments[ii]);
    m_vTerrainLineSegments[ii].SetColor(GLColor(1.0f, 0.0f, 0.0f));
    m_vTerrainLineSegments[ii].SetIgnoreDepth(true);
  }

  //maximum number of plans
  m_lPlanLineSegments.resize(25);
  for (GLCachedPrimitives*& pStrip: m_lPlanLineSegments) {
    pStrip = new GLCachedPrimitives();
    m_Gui.AddGLObject(pStrip);
    pStrip->SetColor(GLColor(0.0f, 1.0f, 0.0f));
  }

  m_Gui.AddGLObject(&m_DestAxis);
  //m_Gui.AddGLObject(m_pControlLine);
  m_Gui.AddPanel(&m_GuiPanel);
}
