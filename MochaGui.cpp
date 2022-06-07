#include <stdio.h>
#include "MochaGui.h"

//car control variables
// float g_fTurnrate = 0;
// float g_fSpeed = 0;
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
    m_Path( CreateCVar("path", vector<int>(), "Path vector.") ),
    m_StillRun( true ),
    m_StillControl( true ),
    m_bPlannerOn( CreateUnsavedCVar("planner.PlannerOn", false, "if true, the car model will only follow the control law from the 2D path.") ),
    m_bShowProjectedPath( CreateCVar("planner.ShowProjected2dPath", true, "how the 2D path z-projected onto the surface") ),
    m_bCompute3dPath( CreateCVar("planner.Compute3DPath", true, "Compute path using terrain driven simulation") ),
    m_bSimulate3dPath( CreateUnsavedCVar("planner.Simulate3DPath", false, "Drive the car over the path to see the results in real-time") ),
    m_eControlTarget( CreateCVar("controller.ControlTarget", (int)eTargetSimulation, "Drive the car over the path to see the results in real-time") ),
    m_bControl3dPath( CreateUnsavedCVar("planner.Control3DPath", false, "Drive the car over the path to see the results in real-time") ),
    m_nPathSegments( CreateCVar("planner.PathSegments", 20u, "how many segments to draw") ),
    m_dTimeInterval( CreateCVar("planner.TimeInterval", 0.01, "") ), /* changed TimeInteval to TimeInterval (also in opt.cpp) */
    m_pPlannerThread( 0 ),
    m_pPhysicsThread( 0 ),
    m_pControlThread( 0 ),
    m_pCommandThread( 0 ),
    m_pLocalizerThread( 0 ),
    m_bLoggerEnabled( CreateUnsavedCVar("logger.Enabled", false, "")),
    m_sLogFileName( CreateUnsavedCVar("logger.FileName", std::string(), "/Users/corinsandford/ARPG/MochaGui/logs/mocha.log")),
    m_bFusionLoggerEnabled( CreateUnsavedCVar("logger.FusionEnabled", false, "")),
    m_bLoggerPlayback( CreateUnsavedCVar("logger.Playback", false, "")),
    m_nNumControlSignals( 0 ),
    m_nNumPoseUpdates( 0 ),
    m_dPlaybackTimer( -1 ),
    m_Fusion( 30, &m_DriveCarModel ),
    m_bLoadWaypoints( CreateCVar("planner:LoadWaypoints", false, "Load waypoints on start.") ),
    m_dPlanTime( Tic() ), //the size of the fusion sensor
    m_vLearningParams( std::vector<RegressionParameter>() ),
    m_vControlParams( std::vector<RegressionParameter>() ),
    m_vPlannerParams( std::vector<RegressionParameter>() ),
    m_vDriveParams( std::vector<RegressionParameter>() ),
    m_mDefaultParameters( CarParameterMap() )
{
    
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
    //create the list of cvars not to save
    std::vector<std::string> filter = { "not", "debug.MinLookaheadTime", "debug.MaxPlanTimeMultiplier", "debug.FreezeControl", "debug.PointCost", "debug.Show2DResult", "debug.Optimize2DOnly", "debug.ForceZeroStartingCurvature", "debug.UseCentralDifferences", "debug.UseGoalPoseStepping", "debug.DisableDamping", "debug.MonotonicCost", "debug.LocalizerDownsampler", /*"debug.ImuIntegrationOnly",*/ "debug.ShowJacobianPaths", "debug.SkidCompensationActive", "debug.PlaybackControlPaths", "debug.MaxLookaheadTime", "debug.InertialControl", "debug.StartSegmentIndex", "planner.PointCostWeights", "planner.TrajCostWeights", "planner.Epsilon"};

    int frameCount = 0;
    double lastTime = Tic();
    m_dDiagTime = Tic();
    SetThreadName("Main Thread");
    LOG(INFO) << "Starting Threads";
    _StartThreads();
    double lastFrameTime = Tic();
    while( !pangolin::ShouldQuit() && ros::ok() )
    {

      {
        boost::mutex::scoped_lock lock(m_DrawMutex);
        m_Gui.Render();
      }

        std::this_thread::sleep_for(std::chrono::microseconds(10));

        //calculate Fps
        frameCount++;
        if(Toc(lastTime) > 0.5){
            m_dFps = (double)frameCount / (Toc(lastTime));
            frameCount = 0;
            lastTime = Tic();
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
void MochaGui::Init(const std::string& sRefPlane, const std::string& sMesh, bool bLocalizer,
                    const std::string sMode, const std::string sLogFile, const std::string& sParamsFile,
                    const std::string& sCarMesh, const std::string& sWheelMesh)
{
    m_sPlaybackLogFile = sLogFile;
    m_sParamsFile = sParamsFile;
    m_bSIL = false;

    //m_Node.subscribe("herbie/Imu"); // Should change to match TestNode/CarPoseSim.cpp (NinjaCar/Compass)? This is a separate Imu channel that I think is currently unused. See _ImuReadFunc

    m_dStartTime = Tic();
    m_bAllClean = false;
    m_bPause = false;
    m_bStep = false;

    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'c', std::bind(CommandHandler, eMochaClear ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + '1', std::bind(CommandHandler, eMochaToggleTrajectory ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + '2', std::bind(CommandHandler, eMochaTogglePlans ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'l', [this] {CarParameters::LoadFromFile(m_sParamsFile,m_mDefaultParameters);} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 's', [this] {CarParameters::SaveToFile(m_sParamsFile,m_mDefaultParameters);} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'r', std::bind(CommandHandler, eMochaRestart ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 't', std::bind(CommandHandler, eMochaSolve ) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'e', std::bind(CommandHandler, eMochaPpmControl ) );
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

    // Moved this code to simulation/experiment since that what it seems to be
    // i.e. the localizer is running when in Experiment mode
    if(bLocalizer){
        // Experiment?
        pScene->mRootNode->mTransformation = aiMatrix4x4(1,0,0,0,
                                                         0,-1,0,0,
                                                         0,0,-1,0,
                                                         0,0,0,1);
    } else {
        // Simulation?
        pScene->mRootNode->mTransformation = aiMatrix4x4(1,0,0,0,
                                                         0,1,0,0,
                                                         0,0,-1,0,
                                                         0,0,0,1);
    }

    if(sMode == "Simulation"){
        m_eControlTarget = eTargetSimulation;
    //     pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
    //                                                       0, 1, 0, 0,
    //                                                       0, 0,-1, 0,
    //                                                       0, 0, 0, 1 );
    }else{
        m_eControlTarget = eTargetExperiment;
    //     pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
    //                                                       0, 1, 0, 0,
    //                                                       0, 0,-1, 0,
    //                                                       0, 0, 0, 1 );
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
    CarParameters::LoadFromFile(m_sParamsFile,m_mDefaultParameters);

    /// Generate as many new cars as we need in order to use MPC.
    m_LearningCarModel.Init(pCollisionShape,dMin,dMax, m_mDefaultParameters, REGRESSOR_NUM_WORLDS );
    m_PlanCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters,LocalPlanner::GetNumWorldsRequired(OPT_DIM) );
    m_ControlCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters, LocalPlanner::GetNumWorldsRequired(OPT_DIM)  );
    //parameters[CarParameters::SteeringCoef] = -700.0;

    /// As well as the car that we're actually driving.
    if ( m_bSIL )
        m_DriveCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters,1, true );
    else
        m_DriveCarModel.Init( pCollisionShape,dMin,dMax, m_mDefaultParameters,1, false );


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

    {
    boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    /// Establish number of waypoints and where they are; set them to CVars.
    /*
    float radius = 1.25;
    Eigen::Vector3d offset(1.,0.,0.);
    int numWaypoints = 8;
    for(int ii = 0; ii < numWaypoints ; ii++){
        char buf[100];
        snprintf( buf, 100, "waypnt.%d", ii );
        m_vWayPoints.push_back(&CreateGetCVar(std::string(buf), MatrixXd(8,1)));

        /// Linear waypoints.
        //    (*m_vWayPoints.back()) << ii*0.5,ii*0.5,0,0,0,0, 1,0;

        /// Waypoints in a circle.
        /// 8-vector < x, y, z, roll, pitch, yaw (radians), velocity, air?
        (*m_vWayPoints.back()) << radius*sin((numWaypoints-ii-1)*2*M_PI/numWaypoints)+offset[0],
            radius*cos((numWaypoints-ii-1)*2*M_PI/numWaypoints)+offset[1],
            0+offset[2], 0, 0, M_PI-(numWaypoints-ii-1)*2*M_PI/numWaypoints, 1, 0;
        /// Load this segment ID into a vector that enumerates the path elements.
        m_Path.push_back(ii);
    }
    //close the loop
    m_Path.push_back(0);
    */
    Eigen::Vector3d offset(1.,0.,0.);
    int numWaypoints = 2;
    for(int ii = 0; ii < numWaypoints ; ii++){
        char buf[100];
        snprintf( buf, 100, "waypnt.%d", ii );
        m_vWayPoints.push_back(&CreateGetCVar(std::string(buf), MatrixXd(8,1)));

        /// Linear waypoints.
        (*m_vWayPoints.back()) << ii*1+offset(0), 0+offset(1), 0+offset(2), 0, 0, 0, 1, 0;

        /// Waypoints in a circle.
        /// 8-vector < x, y, z, roll, pitch, yaw (radians), velocity, air?
        // (*m_vWayPoints.back()) << radius*sin((numWaypoints-ii-1)*2*M_PI/numWaypoints)+offset[0],
        //     radius*cos((numWaypoints-ii-1)*2*M_PI/numWaypoints)+offset[1],
        //     0+offset[2], 0, 0, M_PI-(numWaypoints-ii-1)*2*M_PI/numWaypoints, 1, 0;
        /// Load this segment ID into a vector that enumerates the path elements.
        m_Path.push_back(ii);
    }
    //close the loop
    // m_Path.push_back(0);
    }

    m_nStateStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);
    m_nLearningStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);
    m_nControlStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);
    m_nGravityStatusId = m_Gui.AddStatusLine(PlannerGui::eTopLeft);

    m_TerrainMesh.SetAlpha(1.0);
    m_Gui.Init(&m_TerrainMesh);

  /// m_pGraphView cannot be resized in old_pangolin and crashes everything.
//    m_pGraphView = &pangolin::Plotter(&m_Log)
//            .SetBounds(0.0, 0.3, 0.6, 1.0);
//    pangolin::DisplayBase().AddDisplay(*m_pGraphView);

    // Add Car to Car Planner
    m_nDriveCarId = m_Gui.AddCar( m_mDefaultParameters[CarParameters::WheelBase], m_mDefaultParameters[CarParameters::Width], sCarMesh, sWheelMesh );
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

    // Changed "NinjaCar" to "Compass"
    m_sCarObjectName = "base_link";
    // if ( m_bSIL ) {
    //     // Changed "posetonode" to "BulletCarModel" to match BulletCarModel.cpp
    //     m_Localizer.TrackObject( m_sCarObjectName, Sophus::SE3d(dT_localizer_ref).inverse() );
    //     LOG(INFO) << "Localizer initialized to track BulletCarModel at " << m_sCarObjectName;
    // } else {
    //     // Changed "posetonode" to "NinjaCar" to match CarPosSim.cpp
    //     m_Localizer.TrackObject( m_sCarObjectName, Sophus::SE3d(dT_localizer_ref).inverse() );
    //     LOG(INFO) << "Localizer initialized to track NinjaCar at " << m_sCarObjectName;
    // }        
    m_Localizer.TrackObject( m_sCarObjectName, Sophus::SE3d(dT_localizer_ref).inverse() );

    //initialize the panel
    m_GuiPanel.Init();

    //fusion parameters
    m_GuiPanel.SetVar("fusion:FilterSize",m_Fusion.GetFilterSizePtr())
              .SetVar("fusion:RMSE",m_Fusion.GetRMSEPtr())
              .SetVar("fusion:LocalizerFreq",&m_dLocalizerFreq)
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

    m_nh = new ros::NodeHandle("~");  

    ros::param::param<double>("raycast_len", raycast_len, raycast_len);

    m_terrainMeshSub = m_nh->subscribe<mesh_msgs::TriangleMeshStamped>("/input_terrain_mesh", 1, boost::bind(&MochaGui::_terrainMeshCallback, this, _1));
    m_waypointsSub = m_nh->subscribe("/input_waypoints", 1, &MochaGui::_waypointsCb, this);

    m_pMeshPubThread = new boost::thread(std::bind(&MochaGui::_MeshPubFunc,this));
    m_pStatePubThread = new boost::thread(std::bind(&MochaGui::_StatePubFunc,this));
    m_pWaypointPubThread = new boost::thread(std::bind(&MochaGui::_WaypointPubFunc,this));
    m_pPathPubThread = new boost::thread(std::bind(&MochaGui::_PathPubFunc,this));

    m_statePub = m_nh->advertise<carplanner_msgs::VehicleState>("state",1);
    m_terrainMeshPub = m_nh->advertise<mesh_msgs::TriangleMeshStamped>("/output_terrain_mesh",1);
    m_groundplaneMeshPub = m_nh->advertise<mesh_msgs::TriangleMeshStamped>("/output_groundplane_mesh",1);
    m_simPathPub = m_nh->advertise<visualization_msgs::MarkerArray>("sim_path",1);
    m_ctrlPathPub = m_nh->advertise<visualization_msgs::MarkerArray>("ctrl_path",1);
    m_waypointPub = m_nh->advertise<geometry_msgs::PoseArray>("waypoints",1);
}

///////////////////////////////////////////////////////////////
void MochaGui::_pubTerrainMesh(uint nWorldId)
{

    BulletWorldInstance* pWorld = m_DriveCarModel.GetWorldInstance(nWorldId);
    boost::unique_lock<boost::mutex> lock(*pWorld);
    // time_t t0 = clock();

    if (pWorld->m_pTerrainBody != NULL)
        _pubMesh(pWorld->m_pTerrainBody->getCollisionShape(), &(pWorld->m_pTerrainBody->getWorldTransform()), &m_terrainMeshPub);
    
    if (pWorld->m_pGroundplaneBody != NULL)
        _pubMesh(pWorld->m_pGroundplaneBody->getCollisionShape(), &(pWorld->m_pGroundplaneBody->getWorldTransform()), &m_groundplaneMeshPub);
    
    // time_t t1 = clock();
    // uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(pWorld->m_pTerrainBody->getCollisionShape())->getMeshInterface())->getNumTriangles();
    // ROS_INFO_THROTTLE(0.5,"pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
}

/////////////////////////////////////////////////////////////////
void MochaGui::_pubMesh(btCollisionShape* collisionShape, ros::Publisher* pub)
{
    btTransform* parentTransform = new btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
    _pubMesh(collisionShape, parentTransform, pub);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubMesh(btCollisionShape* collisionShape, btTransform* parentTransform, ros::Publisher* pub)
{
     // ROS_INFO("about to pub mesh with %d faces", dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(collisionShape)->getMeshInterface())->getNumTriangles());

      time_t t0 = std::clock();

      btTransform x180(
        btQuaternion(1.0, 0.0, 0.0, 0.0),
        btVector3(0.0, 0.0, 0.0)
      );
    //   (*parentTransform) = x180 * (*parentTransform);// * x180;
      btTransform* tr = new btTransform(btQuaternion(1,0,0,0),btVector3(0,0,0)); // rot 180 x
      (*tr) = (*tr) * (*parentTransform);

      mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
      convertCollisionShape2MeshMsg(collisionShape, tr, &mesh);

      mesh_msgs::TriangleMeshStamped::Ptr mesh_stamped(new mesh_msgs::TriangleMeshStamped);
      mesh_stamped->mesh = *mesh;
      mesh_stamped->header.frame_id = "map";
      mesh_stamped->header.stamp = ros::Time::now();

      time_t t1 = std::clock();

      pub->publish(mesh_stamped);
      // ros::Rate(10).sleep();
    //   ROS_INFO("pub'ed mesh, %d faces, %d vertices, %.2f sec", mesh->triangles.size(), mesh->vertices.size(), std::difftime(t1,t0)/CLOCKS_PER_SEC); 
}

void MochaGui::_terrainMeshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg)
{
  static tf::StampedTransform Twm;
  try
  {
    m_tflistener.waitForTransform("map", "infinitam", ros::Time::now(), ros::Duration(1.0));
    m_tflistener.lookupTransform("map", "infinitam", ros::Time(0), Twm);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    usleep(10000);
    return;
  }

  tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));
  Twm.setData(rot_180_x*Twm);

  time_t t0 = std::clock();

  btCollisionShape* meshShape;// = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
  convertMeshMsg2CollisionShape_Shared(mesh_msg, meshShape);

  for (uint i=0; i<m_PlanCarModel.GetWorldCount(); i++)
  {
    m_PlanCarModel.setTerrainMesh(i, meshShape, Twm);
  }
  for (uint i=0; i<m_LearningCarModel.GetWorldCount(); i++)
  {
    m_LearningCarModel.setTerrainMesh(i, meshShape, Twm);
  }
  for (uint i=0; i<m_DriveCarModel.GetWorldCount(); i++)
  {
    m_DriveCarModel.setTerrainMesh(i, meshShape, Twm);
  }
  for (uint i=0; i<m_ControlCarModel.GetWorldCount(); i++)
  {
    m_ControlCarModel.setTerrainMesh(i, meshShape, Twm);
  }

//   aiMesh* sceneMesh;
//   convertMeshMsgToAssimpMesh(mesh_msg, sceneMesh);
//   m_TerrainMesh.Init(scene);
//   m_TerrainMesh.SetAlpha(1.0);
//   m_Gui.Init(&m_TerrainMesh);

  time_t t1 = std::clock();
  ROS_INFO("got mesh, %d faces, %d vertices, %.2f sec", mesh_msg->mesh.triangles.size(), mesh_msg->mesh.vertices.size(), difftime(t1,t0)/CLOCKS_PER_SEC);

}

/////////////////////////////////////////////////////////////////////////////////////////
bool MochaGui::Raycast(const Eigen::Vector3d& dSource,const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool& biDirectional, int index /*= 0*/)
{
    btVector3 source(dSource[0],dSource[1],dSource[2]);
    btVector3 vec(dRayVector[0],dRayVector[1],dRayVector[2]);
    btVector3 target = source + vec;
    BulletWorldInstance* pInstance = m_DriveCarModel.GetWorldInstance(index);

    btVehicleRaycaster::btVehicleRaycasterResult results,results2;
    if( biDirectional ){
        source = source - vec;
    }

    if(pInstance->m_pVehicleRayCaster->castRay(source,target,results) == 0){
        return false;
    }else{
        Eigen::Vector3d dNewSource(source[0],source[1],source[2]);
        dIntersect = dNewSource + results.m_distFraction* (biDirectional ? (Eigen::Vector3d)(dRayVector*2) : dRayVector);
        return true;
    }
}

////////////////////////////////////////////////////////////////////
void MochaGui::SE3dFromWaypoint(Sophus::SE3d& Twv, const Eigen::MatrixXd& wp)
{
    std::cout << "getting se3d from wp: " << wp << std::endl;
    Eigen::AngleAxisd rollAngle(wp.col(0)[WAYPOINT_ROLL_INDEX], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(wp.col(0)[WAYPOINT_PITCH_INDEX], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(wp.col(0)[WAYPOINT_YAW_INDEX], Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quat = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d mat = quat.matrix();

    Twv.translation() = wp.col(0).head(3);
    Twv.setRotationMatrix(mat);
}

/////////////////////////////////////////////////////////////////////////
void MochaGui::WaypointFromOdomMsg(Eigen::MatrixXd & wp, const nav_msgs::Odometry & odom_msg)
{
    Eigen::Quaterniond quat(odom_msg.pose.pose.orientation.w,
                            odom_msg.pose.pose.orientation.x,
                            odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z);
    Eigen::Vector3d eul = quat.toRotationMatrix().eulerAngles(0, 1, 2);

    wp.col(0)[WAYPOINT_X_INDEX] = odom_msg.pose.pose.position.x;
    wp.col(0)[WAYPOINT_Y_INDEX] = odom_msg.pose.pose.position.y;
    wp.col(0)[WAYPOINT_Z_INDEX] = odom_msg.pose.pose.position.z;
    wp.col(0)[WAYPOINT_ROLL_INDEX] = eul[0];
    wp.col(0)[WAYPOINT_PITCH_INDEX] = eul[1];
    wp.col(0)[WAYPOINT_YAW_INDEX] = eul[2];
    wp.col(0)[WAYPOINT_VEL_INDEX] = odom_msg.twist.twist.linear.x;
    wp.col(0)[WAYPOINT_AIR_INDEX] = 0;
}

////////////////////////////////////////////////////////////////////
void MochaGui::_waypointsCb(const carplanner_msgs::WayPoints& wp_msg)
{
    // if (!m_bServersInitialized) { DLOG(INFO) << "Aborting waypoints callback bc servers not initialized."; return; }

    if (wp_msg.odom_arr.odoms.size() != 2)
        ROS_WARN("Tried to set waypoints of length %d. Need 2. Ignoring...", wp_msg.odom_arr.odoms.size());

    // ROS_INFO("Got %d waypoints. Replacing %d.", wp_msg.odom_arr.odoms.size(), m_vWayPoints.size());

    EigenTransform x180 = EigenTransform::Identity();
    x180.translate(Eigen::Vector3d(0.0, 0.0, 0.0));
    x180.rotate(Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0));
        
    boost::mutex::scoped_lock lock( m_DrawMutex );
    for (uint idx=0; idx<2; idx++) {
        // Eigen::MatrixXd wp(8,1);
        // WaypointFromOdomMsg(wp, wp_msg.odom_arr.odoms[idx]);
        nav_msgs::Odometry odom_msg = wp_msg.odom_arr.odoms[idx];
        EigenTransform wpt = EigenTransform::Identity();
        wpt.translate(Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
        wpt.rotate(Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z));
        wpt = x180 * wpt * x180;
        // m_Gui.GetWaypoint(idx)->m_Waypoint.SetPose(Eigen::Vector6d(wp.col(0).head(6)));        
        m_Gui.GetWaypoint(idx)->m_Waypoint.SetPose(wpt.matrix());

        Eigen::Vector3d dIntersect;
        Sophus::SE3d pose( m_Gui.GetWaypoint(idx)->m_Waypoint.GetPose4x4_po() );
        if( m_DriveCarModel.RayCast(pose.translation(), GetBasisVector(pose,2)*raycast_len, dIntersect, true) ){
            pose.translation() = dIntersect;
            m_Gui.GetWaypoint(idx)->m_Waypoint.SetPose( pose.matrix() );
        }

        *m_vWayPoints[idx] << m_Gui.GetWaypoint(idx)->m_Waypoint.GetPose(), 
                              m_Gui.GetWaypoint(idx)->m_Waypoint.GetVelocity(), 
                              m_Gui.GetWaypoint(idx)->m_Waypoint.GetAerial();

        m_Gui.GetWaypoint(idx)->m_Waypoint.SetDirty(true);
    }
    

    // // std::vector<Eigen::MatrixXd*> vWayPoints;
    // // std::vector<int> Path;
    // m_vWayPoints.clear();
    // m_Path.clear();
    // for(uint i=0; i<wp_msg.odom_arr.odoms.size(); i++)
    // {
    //     ROS_INFO("adding waypoint %d", i);
    //     float s = sqrt(wp_msg.odom_arr.odoms[i].pose.pose.orientation.x*wp_msg.odom_arr.odoms[i].pose.pose.orientation.x +
    //                     wp_msg.odom_arr.odoms[i].pose.pose.orientation.y*wp_msg.odom_arr.odoms[i].pose.pose.orientation.y +
    //                     wp_msg.odom_arr.odoms[i].pose.pose.orientation.z*wp_msg.odom_arr.odoms[i].pose.pose.orientation.z +
    //                     wp_msg.odom_arr.odoms[i].pose.pose.orientation.w*wp_msg.odom_arr.odoms[i].pose.pose.orientation.w);
    //     if (s>.1)
    //     {
    //         Eigen::MatrixXd wp(8,1);
    //         WaypointFromOdomMsg(wp, wp_msg.odom_arr.odoms[i]);
    //         if (!_AddWaypoint(wp, m_vWayPoints))
    //         {
    //             ROS_ERROR("Failed to add waypoint.");
    //             return;
    //         }
    //         m_Path.push_back(i);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to add waypoints bc of zero norm quaternion.");
    //         return;
    //     }
        
    // }
    
    // if (wp_msg.loop)
    //     m_Path.push_back(0);

    // m_vSegmentSamples.resize(m_vWaypoints.size()-1);

    // ROS_INFO("transfering");

    // {
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    // m_vWayPoints = vWayPoints;
    // m_Path = Path;

    // ROS_INFO("Refreshing wps... wps %d path %d", m_vWayPoints.size(), m_Path.size());

    // {
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    // boost::mutex::scoped_lock lock(m_DrawMutex);
    // // _RefreshWaypoints();
    // _PopulateSceneGraph();
    // }

    // ROS_INFO("Done setting %d waypoints with loop %s.", m_vWayPoints.size(), (wp_msg.loop?"enabled":"disabled"));
}

bool MochaGui::_AddWaypoint(Eigen::MatrixXd& wp, std::vector<Eigen::MatrixXd*> & wp_arr, bool raycast)
{
    std::cout << "adding wp: " << wp << std::endl;
    Sophus::SE3d Twv;
    SE3dFromWaypoint(Twv, wp);
    Eigen::Vector3d dIntersect;
    if (raycast && Raycast(wp.col(0).head(3), -GetBasisVector(Twv,2)*raycast_len, dIntersect, true))
    {
        wp.col(0).head(3) = dIntersect;// + Eigen::Vector3d(0,0,-0.1); // in ned
    }

    char buf[100];
    snprintf( buf, 100, "waypnt.%d", wp_arr.size());
    wp_arr.push_back(&CreateGetCVar(std::string(buf), wp));

    // ROS_INFO("Added %dth waypoint:\n Pwv %.2f %.2f %.2f\n Qwv %.2f %.2f %.2f %.2f\n Vwv %.2f %.2f %.2f\n Wwv %.2f %.2f %.2f", 
    //     m_vWaypoints.size(), 
    //     wp_ptr->state.m_dTwv.translation().x(), wp_ptr->state.m_dTwv.translation().y(), wp_ptr->state.m_dTwv.translation().z(),
    //     wp_ptr->state.m_dTwv.unit_quaternion().x(), wp_ptr->state.m_dTwv.unit_quaternion().y(), wp_ptr->state.m_dTwv.unit_quaternion().z(), wp_ptr->state.m_dTwv.unit_quaternion().w(),
    //     wp_ptr->state.m_dV[0], wp_ptr->state.m_dV[1], wp_ptr->state.m_dV[2], 
    //     wp_ptr->state.m_dW[0], wp_ptr->state.m_dW[1], wp_ptr->state.m_dW[2]);

    return true;
}

bool MochaGui::_SetWaypoint(uint idx, Eigen::MatrixXd& wp, bool raycast)
{
    if (idx >= m_vWayPoints.size())
        return false; 

    Sophus::SE3d Twv;
    SE3dFromWaypoint(Twv, wp);
    Eigen::Vector3d dIntersect;
    if (raycast && Raycast(wp.col(0).head(3), -GetBasisVector(Twv,2)*raycast_len, dIntersect, true))
    {
        wp.col(0).head(3) = dIntersect;// + Eigen::Vector3d(0,0,-0.1); // in ned
    }

    char buf[100];
    snprintf( buf, 100, "waypnt.%d", idx);
    m_vWayPoints.at(idx) = &CreateGetCVar(std::string(buf), wp);

    // ROS_INFO("Setting %dth waypoint to\n Pwv %.2f %.2f %.2f\n Qwv %.2f %.2f %.2f %.2f\n Vwv %.2f %.2f %.2f\n Wwv %.2f %.2f %.2f", 
    //     idx+1, 
    //     wp_ptr->state.m_dTwv.translation().x(), wp_ptr->state.m_dTwv.translation().y(), wp_ptr->state.m_dTwv.translation().z(),
    //     wp_ptr->state.m_dTwv.unit_quaternion().x(), wp_ptr->state.m_dTwv.unit_quaternion().y(), wp_ptr->state.m_dTwv.unit_quaternion().z(), wp_ptr->state.m_dTwv.unit_quaternion().w(),
    //     wp_ptr->state.m_dV[0], wp_ptr->state.m_dV[1], wp_ptr->state.m_dV[2], 
    //     wp_ptr->state.m_dW[0], wp_ptr->state.m_dW[1], wp_ptr->state.m_dW[2]);

    return true;
}

/////////////////////////////////////////////////////////////////////////
void MochaGui::_MeshPubFunc()
{
    std::cout << "Starting Mesh Publisher Thread" << std::endl;
    while( ros::ok() && m_StillRun )
    {
        _pubTerrainMesh(0);
        ros::Rate(100).sleep();
    }
}

/////////////////////////////////////////////////////////////////////////
void MochaGui::_StatePubFunc()
{
    std::cout << "Starting State Publisher Thread" << std::endl;
    while( ros::ok() && m_StillRun )
    {
        _pubState();
        ros::Rate(100).sleep();
    }
}

/////////////////////////////////////////////////////////////////////////
void MochaGui::_WaypointPubFunc()
{
    std::cout << "Starting Waypoint Publisher Thread" << std::endl;
    while( ros::ok() && m_StillRun )
    {
        _pubWaypoints();
        ros::Rate(100).sleep();
    }
}

/////////////////////////////////////////////////////////////////////////
void MochaGui::_PathPubFunc()
{
    std::cout << "Starting Path Publisher Thread" << std::endl;
    while( ros::ok() )
    {
        _pubPath();
        ros::Rate(100).sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// void MochaGui::_pubCommand()
// {
//     _pubCommand(m_ControlCommand);
// }

// //////////////////////////////////////////////////////////////////////////////////
// void MochaGui::_pubCommand(ControlCommand& cmd)
// {
//     carplanner_msgs::Command cmd_msg;
//     cmd_msg.force       = cmd.m_dForce;
//     cmd_msg.curvature   = cmd.m_dCurvature;
//     cmd_msg.dt          = cmd.m_dTime;
//     cmd_msg.phi         = cmd.m_dPhi;
//     for(unsigned int i=0; i<3; i++)
//     {
//         cmd_msg.torques.push_back(cmd.m_dTorque[i]);
//     }

//     m_commandPub.publish(cmd_msg);
//     ros::spinOnce();
// }

////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubPath()
{
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);

    // if(!m_lPlanStates.empty())
    // {
    //     _pubPath(m_lPlanStates);
    // }
    // if(!m_vTerrainLineSegments.empty())
    // {
    //     _pubPath(m_vTerrainLineSegments);
    // }
    /* Publish path as produced by PlannerThread for whole trajectory.
       When planning is off, the traj is slightly inward of circular. When planning is on, it becomes circular. */
    if(!m_vSegmentSamples.empty())
    {
        // ROS_INFO("Pubbing m_vSegmentSamples of size %d", m_vSegmentSamples.size());
        _pubPathArr(&m_simPathPub, m_vSegmentSamples);
    }
    if(!m_lPlanStates.empty())
    {
        // ROS_INFO("Pubbing m_lPlanStates of size %d", m_lPlanStates.size());
        _pubPathArr(&m_ctrlPathPub, m_lPlanStates);
    }
    /* Same as above except only 1 path segment that iterates through all segments until the end during planning. 
       When planning is off, the traj is slightly inward of circular. When planning is on, it becomes circular. */
    // if(!m_vActualTrajectory.empty())
    // {
    //     _pubPath(&m_pathPub, m_vActualTrajectory);
    // }
    /* Same as above except When planning is off, the traj is circular. When planning is on, it is slightly outward of circular. */
    // if(!m_vControlTrajectory.empty())
    // {
    //     _pubPath(&m_pathPub, m_vControlTrajectory);
    // }
    
}

//////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubPath(ros::Publisher* pub, list<std::vector<VehicleState> *>& path_in)
{   
    list<std::vector<VehicleState> *> some_path = path_in;
    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(some_path, &path_msg);
    pub->publish(path_msg);
    ros::spinOnce();
}

///////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubPathArr(ros::Publisher* pub, list<std::vector<VehicleState> *>& path_in)
{   
    list<std::vector<VehicleState> *> some_path = path_in;
    carplanner_msgs::PathArray patharr_msg;
    convertSomePath2PathArrayMsg(some_path, &patharr_msg);
    visualization_msgs::MarkerArray markarr_msg;
    carplanner_msgs::MarkerArrayConfig markarr_config = carplanner_msgs::MarkerArrayConfig("",0.01,0,0,0.0,0.0,1.0,1.0);
    convertPathArrayMsg2LineStripArrayMsg(patharr_msg,&markarr_msg,markarr_config);
    pub->publish(markarr_msg);
    ros::spinOnce();
}

///////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubPath(ros::Publisher* pub, Eigen::Vector3dAlignedVec& path_in)
{   
    Eigen::Vector3dAlignedVec some_path = path_in;
    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(some_path, &path_msg);
    pub->publish(path_msg);
    ros::spinOnce();
}

///////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubPath(ros::Publisher* pub, std::vector<MotionSample>& path_in)
{   
    std::vector<MotionSample> some_path = path_in;
    nav_msgs::Path path_msg;
    convertSomePath2PathMsg(some_path, &path_msg);
    pub->publish(path_msg);
    ros::spinOnce();
}

/////////////////////////////////////////////////////////////////////
void MochaGui::_pubPathArr(ros::Publisher* pub, std::vector<MotionSample>& path_in)
{   
    std::vector<MotionSample> some_path = path_in;
    carplanner_msgs::PathArray patharr_msg;
    convertSomePath2PathArrayMsg(some_path, &patharr_msg);
    visualization_msgs::MarkerArray markarr_msg;
    carplanner_msgs::MarkerArrayConfig markarr_config = carplanner_msgs::MarkerArrayConfig("",0.01,0,0,0.0,1.0,0.0,1.0);
    convertPathArrayMsg2LineStripArrayMsg(patharr_msg,&markarr_msg, markarr_config);
    pub->publish(markarr_msg);
    ros::spinOnce();
}

///////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubState()
{
    // ROS_INFO("Pubbing state");
    VehicleState state;
    m_DriveCarModel.GetVehicleState(0,state);
    state.FlipCoordFrame();
    _pubState(state);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubState(VehicleState& state)
{
  carplanner_msgs::VehicleState state_msg = state.toROS();

  m_statePub.publish(state_msg);
  // m_tfcaster.sendTransform(state_msg.pose);
  ros::spinOnce();
}

////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubWaypoints()
{
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    _pubWaypoints(m_vWayPoints);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void MochaGui::_pubWaypoints(std::vector<Eigen::MatrixXd*> pts)
{
  tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));

  geometry_msgs::PoseArray pts_msg;
  pts_msg.header.frame_id = "map";
  pts_msg.header.stamp = ros::Time::now();

  for( uint ii=0; ii<pts.size(); ii++ )
  {
    Eigen::MatrixXd* pt = pts[ii];
    double roll =   (*pt)(3);
    double pitch =  (*pt)(4);
    double yaw =    (*pt)(5);
    Eigen::Quaterniond quat;
    convertRPY2Quat(Eigen::Vector3d(roll, pitch, yaw), &quat);

    tf::Transform pt_tf(tf::Quaternion(quat.x(),quat.y(),quat.z(),quat.w()),
                        tf::Vector3((*pt)(0),(*pt)(1),(*pt)(2)));
    pt_tf = rot_180_x*pt_tf*rot_180_x;

    geometry_msgs::Pose pt_msg;
    pt_msg.position.x = pt_tf.getOrigin().x();
    pt_msg.position.y = pt_tf.getOrigin().y();
    pt_msg.position.z = pt_tf.getOrigin().z();
    pt_msg.orientation.x = pt_tf.getRotation().x();
    pt_msg.orientation.y = pt_tf.getRotation().y();
    pt_msg.orientation.z = pt_tf.getRotation().z();
    pt_msg.orientation.w = pt_tf.getRotation().w();

    pts_msg.poses.push_back(pt_msg);
  }

  m_waypointPub.publish(pts_msg);
  // m_tfcaster.sendTransform(state_msg.pose);
  ros::spinOnce();
}

////////////////////////////////////////////////////////////////
void MochaGui::_StartThreads()
{
    //recreate the threads
    m_pPlannerThread = new boost::thread(std::bind(&MochaGui::_PlannerFunc,this));
    m_pPhysicsThread = new boost::thread(std::bind(&MochaGui::_PhysicsFunc,this));
    m_pControlThread = new boost::thread(std::bind(&MochaGui::_ControlFunc,this));

    if(m_eControlTarget == eTargetSimulation){
        LOG(INFO) << "Stopping localizer thread";

        if(m_pLocalizerThread) {
            m_pLocalizerThread->interrupt();
            m_pLocalizerThread->join();
        }
    }else{
        m_pCommandThread = new boost::thread(std::bind(&MochaGui::_ControlCommandFunc,this));
        if(m_pLocalizerThread == NULL){
            LOG(INFO) << "Starting Localizer Thread";
            m_pLocalizerThread = new boost::thread(std::bind(&MochaGui::_LocalizerReadFunc,this));
        }
    }
}

void MochaGui::_KillController()
{
  m_StillControl = false;

    //reset the threads
    if(m_pControlThread) {
        m_pControlThread->interrupt();
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
        m_pPhysicsThread->interrupt();
        m_pPhysicsThread->join();
    }

    if(m_pPlannerThread) {
        m_pPlannerThread->interrupt();
        m_pPlannerThread->join();
    }

    if(m_pLearningThread){
        m_pLearningThread->interrupt();
        m_pLearningThread->join();
    }

    if(m_pLocalizerThread) {
        m_pLocalizerThread->interrupt();
        m_pLocalizerThread->join();
    }

    if(m_pCommandThread) {
        m_pCommandThread->interrupt();
        m_pCommandThread->join();
    }

    delete m_pControlThread;
    delete m_pPhysicsThread;
    delete m_pPlannerThread;
    delete m_pLocalizerThread;
    delete m_pCommandThread;

    m_pControlThread = 0;
    m_pPhysicsThread = 0;
    m_pPlannerThread = 0 ;
    m_pLocalizerThread = 0;

    m_Localizer.Stop();
}

////////////////////////////////////////////////////////////////
bool MochaGui::_SetWaypointVel(std::vector<std::string> *vArgs)
{
    if(vArgs->size() == 0){
        return false;
    }
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    double dNewSpeed = atof(vArgs->at(0).c_str());
    //go through all the waypoints and sset the speed
    for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
        m_Gui.GetWaypoint(ii)->m_Waypoint.SetVelocity(dNewSpeed);
    }
    return true;
}

////////////////////////////////////////////////////////////////
bool MochaGui::_SetWaypointVel(float speed)
{
    // boost::mutex::scoped_lock waypointMutex(m_mutexWaypoints);
    //go through all the waypoints and sset the speed
    for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
        m_Gui.GetWaypoint(ii)->m_Waypoint.SetVelocity(speed);
    }
    return true;
}

////////////////////////////////////////////////////////////////
bool MochaGui::_SetWaypointVel(uint idx, float speed)
{
    if(idx >= m_Path.size()){
        return false;
    }
    m_Gui.GetWaypoint(idx)->m_Waypoint.SetVelocity(speed);
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

bool MochaGui::_Refresh()
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
            //LOG(INFO) << "m_bSimulate3dPath == true";
            m_DriveCarModel.GetVehicleState(0,state);
        }else{
            _UpdateVehicleStateFromFusion(state);
            //LOG(INFO) << "Updated Vehicle State from Fusion";
        }

        //offset the chasis by the height offset (constant)
        m_Gui.SetCarState( m_nDriveCarId, state, (m_bSimulate3dPath == true || m_eControlTarget == eTargetExperiment) );
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

    // g_bProcessModelActive set to false in initial CVars at top of MochaGui 6/9/16
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

    currentState.m_dV = GetBasisVector(currentState.m_dTwv,0).normalized()*currentState.m_dV.norm();
    currentState.m_dW[0] = currentState.m_dW[1] = 0;

    Eigen::Vector3d dIntersect;
    if(m_DriveCarModel.RayCast(currentState.m_dTwv.translation(),GetBasisVector(currentState.m_dTwv,2)*raycast_len,dIntersect,true)){
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

   LOG(INFO) << "Starting Localizer Thread.";

    while(1){
        boost::this_thread::interruption_point();
        //this is a blocking call
        double localizerTime;
        const Sophus::SE3d Twb = m_Localizer.GetPose(m_sCarObjectName,true,&localizerTime);
        Sophus::Vector6d Vwb;
        if (m_Localizer.GetLocalizerType()=="ROSLocalizer")
            Vwb = m_Localizer.GetVelocity(m_sCarObjectName,false,&localizerTime);

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
                m_Fusion.RegisterGlobalPose(Twb,localizerTime,sysTime);
                if (m_Localizer.GetLocalizerType()=="ROSLocalizer")
                    m_Fusion.RegisterGlobalVelocity(Vwb);
                //LOG(INFO) << "Set pose";
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
void MochaGui::_ControlCommandFunc()
{
    ROSCommander commander;

    double dLastTime = Tic();
    //if ( !m_Node.advertise( "Commands" ) ) LOG(ERROR) << "'Commands' topic not advertised on 'MochaGui' node.";
    while(1)
    {
        boost::this_thread::interruption_point();
        //only go ahead if the controller is running, and we are targeting the real vehicle
        if(m_eControlTarget == eTargetExperiment && m_bControllerRunning == true/* && m_bSimulate3dPath == false*/){
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
                if ( !m_bSIL )
                    m_DriveCarModel.UpdateState( 0, m_ControlCommand, m_ControlCommand.m_dT, false, true );
            }

            //send the commands to the car via udp
            m_nNumPoseUpdates++;
            
            double time = Tic();

            //if we are not currently simulating, send these to the ppm
            // if( m_bSimulate3dPath == false )
            // {
                commander.SendCommand(m_ControlCommand, m_bSIL);
            // }

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
    int nNumControlPlans = 0;
    m_dControlPlansPerS = 0;
    double dLastTime = Tic();

    std::vector<std::string> vLabels = {"Lookahead","Accel","Norm", "Vel.", "Z Vel.", "Target Vel."};
    m_Log.SetLabels(vLabels);

    SetThreadName("Control thread");
    LOG(INFO) << "Starting Control Thread";
    try
    {
        m_bControllerRunning = false;

        while(m_StillControl) {
            boost::this_thread::interruption_point();

            m_pControlLine->Clear();
            for (GLCachedPrimitives*& pStrip: m_lPlanLineSegments) {
                pStrip->Clear();
            }
            m_bControllerRunning = false;

            //unlock waypoints
            //lock all the waypoints (? 6/8/16)
            for (size_t ii = 0; ii < m_Path.size() - 1; ii++) {
                m_Gui.GetWaypoint(ii)->m_Waypoint.SetLocked(false);
            }

            //wait for the start command
            //        dout("Waiting for control input and completed plan");
            while(m_bControl3dPath == false || m_bAllClean == false ||
                  (m_bSimulate3dPath == false && m_eControlTarget != eTargetExperiment)){
                boost::this_thread::interruption_point();
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
            startingState.m_dTwv.translation() += (GetBasisVector(startingState.m_dTwv,1)*0.25);
            Eigen::Matrix3d rm;
            int yaw = 90;
            rm << cos(yaw), -sin(yaw),  0,
                  sin(yaw), cos(yaw),   0,
                  0,        0,          1;
            if ( m_eControlTarget == eTargetExperiment ) startingState.m_dTwv.setRotationMatrix(rm);
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
                boost::this_thread::interruption_point();
                while((m_eControlTarget != eTargetExperiment && m_bSimulate3dPath == false
                       && m_StillControl )){
                    boost::this_thread::interruption_point();
                    usleep(1000);
                }



                if(m_bSimulate3dPath ){
                    m_DriveCarModel.GetVehicleState(0,currentState);
                    //get the current position and pass it to the controller
                    m_Controller.SetCurrentPoseFromCarModel(&m_DriveCarModel,0);
                }else if(m_eControlTarget == eTargetExperiment){
                    //get current pose from the Localizer
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

                //now run the controller to create a plan
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

                    nNumControlPlans++;
                    if(Toc(dLastTime) > 0.5){
                        m_dControlPlansPerS = (double)nNumControlPlans / (Toc(dLastTime));
                        nNumControlPlans = 0;
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
    LOG(INFO) << "Starting Physics Thread.";

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
            // Currently set to default false 6/8/16
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

                    m_DriveCarModel.UpdateState(0,currentCommand,currentDt, false, false );

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
                m_DriveCarModel.UpdateState( 0, m_vSegmentSamples[ nCurrentSegment ].m_vCommands[ nCurrentSample ], m_vSegmentSamples[ nCurrentSegment ].m_vCommands[ nCurrentSample ].m_dT, true ); //no delay for simulation
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
    LOG(INFO) << "Starting Planner Thread";
    int numInterations = 0;
    m_bPlanning = false;
    // now add line segments
    bool previousOpenLoopSetting = m_bPlannerOn;
    while (1) {
        boost::this_thread::interruption_point();

        //if the user has changed the openloop setting, dirty all
        //the waypoints
        // 6/8/16 this will not run because of line above while(1) "bool previousOpenLoopSetting = m_bPlannerOn"
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
                    //lock the drawing lock as we will be modifying things here
                    boost::mutex::scoped_lock lock( m_DrawMutex );
                    if( a->GetDirty() ) {
                        Sophus::SE3d pose( a->GetPose4x4_po() );
                        if( m_DriveCarModel.RayCast(pose.translation(), GetBasisVector(pose,2)*raycast_len, dIntersect, true) ){
                            pose.translation() = dIntersect;
                            a->SetPose( pose.matrix() );
                        }
                        *m_vWayPoints[nStartIdx] << a->GetPose(), a->GetVelocity(), a->GetAerial();
                    }
                    if( b->GetDirty() ) {
                        Sophus::SE3d pose( b->GetPose4x4_po() );
                        if( m_DriveCarModel.RayCast(pose.translation(), GetBasisVector(pose,2)*raycast_len, dIntersect, true) ){
                            pose.translation() = dIntersect;
                            b->SetPose( pose.matrix() );
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
