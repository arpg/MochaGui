#include "LearningGui.h"

LearningGui *g_pLearningGuiInstance = NULL;

//static int& g_nLocalizerDownsampler = CVarUtils::CreateGetCVar("debug.LocalizerDownsampler",0);
/////////////////////////////////////////////////////////////////////////////////////////
LearningGui::LearningGui() :
    m_sCarObjectName("NinjaCar"),
    m_dT(CreateCVar("learning.TimeInteval", 0.005, "")),
    m_dImuRate(0),
    m_dLocalizerRate(0),
    m_bLearn(CreateCVar("learning.Active", false, "")),
    m_bLearningRunning(false),
    m_pRegressionSample(NULL),
    m_Regressor(),
    m_dLearnEps(CreateCVar("learning.Epsilon", 1e-4, "")),
    m_nCollectedRegressionSamples(0),
    m_nTotalRegressionSamples(CreateCVar("learning.TotalSamples", 8000, "")),

    m_pLearningThread(NULL),
    m_LearningPanel(),
    m_Fusion(50,&m_FusionCarModel),
    m_bPlayback(CreateCVar("learning.Playback", false, "")),
    m_bStep(false),
    m_bPaused(false),    
    m_bProcessModelEnabled(false),
    m_bRefresh(false),
    m_bRegress(false)
{
    m_Node.init("LearningGui");
    m_bPlayback = false;
}

////////////////////////////////////////////////////////////////
LearningGui *LearningGui::GetInstance()
{
    if(g_pLearningGuiInstance == NULL) {
        g_pLearningGuiInstance = new LearningGui();
    }

    return g_pLearningGuiInstance;
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::Run()
{

    while( !pangolin::ShouldQuit() )
    {
        //update the car state here
        if(m_eMode == Mode_Simulation && m_bPlayback == false){
            VehicleState currentState;
            m_DriveCarModel.GetVehicleState(0,currentState);
            m_Gui.SetCarState(m_nDriveCarId,currentState,m_bLearn);
        }

        {
            boost::mutex::scoped_lock renderMutex(m_RenderkMutex);
            m_Gui.Render();
        }

        if(m_bRefresh){
            if(m_pRegressionSample != NULL && m_bLearningRunning == false && m_bLearn == false){
                //the user has requested we refresh the trajectory
                m_vSamples.clear();
                m_vSampleIndices.clear();
                ApplyVelocitesFunctor5d functor(&m_LearningCarModel,Eigen::Vector3d::Zero());
                m_Regressor.CalculateParamNorms(functor,(MotionSample&)*m_pRegressionSample,m_vTweakParams,&m_vSamples,&m_vSampleIndices);
                _UpdateTrajectories();

            }
            m_bRefresh = false;
        }


        usleep(1e6/100.0);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_UpdateTrajectories()
{
    //clear all line segmenst
    for (GLLineStrip& strip: m_lGLLineSegments) {
        strip.ClearLines();
    }

    std::list<GLLineStrip>::iterator iter = m_lGLLineSegments.begin();
    //now update the GLLines to sohw the trajectory
    for(size_t ii = 0 ; ii < m_vSamples.size(); ii++ ){
        if(m_lGLLineSegments.size() <= ii){
            boost::mutex::scoped_lock renderMutex(m_RenderkMutex);
            m_lGLLineSegments.push_back(GLLineStrip());
            m_lGLLineSegments.back().SetColor(GLColor(1.0f,0.0f,0.0f,1.0f));
            m_lGLLineSegments.back().SetIgnoreDepth(true);
            m_Gui.AddGLObject(&m_lGLLineSegments.back());
            iter = m_lGLLineSegments.end();
            iter--;
        }

        const std::vector<Sophus::SE3d> v3dPts = m_vSamples[ii].GetMotionSample();
        Eigen::Vector3dAlignedVec vPts;
        for(const VehicleState& state : m_vSamples[ii].m_vStates){
            vPts.push_back(state.m_dTwv.translation());
        }
        (*iter).SetPointsFromTrajectory(vPts);
        iter++;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_JoystickReadFunc()
{
    while(1){
        //update the joystick
        m_Joystick.UpdateJoystick();
        ControlCommand command;
        {

            command.m_dForce = (((double)m_Joystick.GetAxisValue(1)/JOYSTICK_AXIS_MAX)*-100.0);
            command.m_dPhi = (((double)m_Joystick.GetAxisValue(2)/(double)JOYSTICK_AXIS_MAX)*m_DriveCarModel.GetParameters(0)[CarParameters::MaxSteering] );
        }

        command.m_dForce += m_DriveCarModel.GetParameters(0)[CarParameters::AccelOffset]*SERVO_RANGE;
        command.m_dCurvature = tan(command.m_dPhi)/m_DriveCarModel.GetParameters(0)[CarParameters::WheelBase];
        command.m_dPhi = command.m_dPhi*m_DriveCarModel.GetParameters(0)[CarParameters::SteeringCoef]*SERVO_RANGE +
                        m_DriveCarModel.GetParameters(0)[CarParameters::SteeringOffset]*SERVO_RANGE;
        //command.m_dPhi = std::max(0.0,std::min(500.0,command.m_dPhi));
        //command.m_dForce = std::max(0.0,std::min(500.0,command.m_dForce));

        //if we are in experiment mode, send these to the ppm
        if(m_eMode  == Mode_Experiment){
            //send the messages to the control-daemon
            CommandMsg Req;
            CommandReply Rep;

            Req.set_accel(command.m_dForce);
            Req.set_phi(command.m_dPhi);
            m_Node.call_rpc("herbie/ProgramControlRpc",Req,Rep);
        }

        {
            boost::mutex::scoped_lock lock(m_JoystickMutex);
            m_JoystickCommand = command;
        }

        if(m_bProcessModelEnabled){
            m_Fusion.RegisterInputCommands(command);
        }

        //sleep for a little while
        usleep(1000);
    }

}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_SetPoseFromFusion()
{
    //update state from localizer
    VehicleState state;
    if(m_bProcessModelEnabled == false){
        fusion::PoseParameter currentPose = m_Fusion.GetCurrentPose();
        state.m_dTwv = currentPose.m_dPose;
        state.m_dV = currentPose.m_dV;
        state.m_dW = currentPose.m_dW;
        state.UpdateWheels(m_DriveCarModel.GetWheelTransforms(0));
    }else{
        m_Fusion.GetVehicleState(state);
    }
    //fix the localizer offset
    if(m_bPlayback == false){
        m_Gui.SetCarState(m_nDriveCarId,state,m_bLearn);
    }

    //update learning
    double time = CarPlanner::Tic();
    if(m_dLastPoseTime == -1){
        m_dLastPoseTime = time;
    }else{
        ControlCommand commands;
        {
            boost::mutex::scoped_lock lock(m_JoystickMutex);
            commands = m_JoystickCommand;
        }
        commands.m_dT = time - m_dLastPoseTime;
        m_dLastPoseTime = time;

        //move along the z axis to offset the vehicle position (this places it on the ground)
        //state.m_dTwv.block<3,1>(0,3) -= state.m_dTwv.block<3,1>(0,2)*0.03;
        m_dTotalLearningTime += commands.m_dT;
        if(m_dTotalLearningTime >= 0.005){
            commands.m_dT = m_dTotalLearningTime;
            _UpdateLearning(commands,state);
            m_dTotalLearningTime = 0;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_ImuReadFunc()
{
    int nNumImu = 0;
    double lastTime = -1;
    while(1){
        Imu_Accel_Gyro Msg;
        if(m_Node.receive("herbie/Imu",Msg)){ // TODO we do NOT need the IMU anymore -- Juan will use that on the ninja car.
            //double time = (double)Msg.timer()/62500.0;
            double sysTime = CarPlanner::Tic();
            m_Fusion.RegisterImuPose(Msg.accelx()*G_ACCEL,Msg.accely()*G_ACCEL,Msg.accelz()*G_ACCEL,
                                     Msg.gyrox(),Msg.gyroy(),Msg.gyroz(),sysTime,sysTime);

            if(lastTime == -1){
                lastTime = sysTime;
                nNumImu = 0;
            }else if( (sysTime - lastTime) > 1 ){
                m_dImuRate = nNumImu / (sysTime - lastTime);
                nNumImu = 0;
                lastTime = sysTime;
            }
            nNumImu++;
            //std::cout << "IMU pose received at:" << CarPlanner::Tic()-m_dStartTime << "seconds [" << Msg.accely() << " " <<  -Msg.accelx() << " " << Msg.accelz() << std::endl;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_LocalizerReadFunc()
{
    int nNumLocalizer = 0;
    double lastTime = -1;
    int nLocalizerSkip = 0;
    while(1){
        //this is a blocking call
        double localizerTime;
        Sophus::SE3d pose = m_Localizer.GetPose(m_sCarObjectName,true,&localizerTime);
        nLocalizerSkip++;

        //offset the localizer measurements
        double sysTime = CarPlanner::Tic();

        ControlCommand command;
        {
            boost::mutex::scoped_lock lock(m_JoystickMutex);
            command = m_JoystickCommand;
        }

        if(m_bProcessModelEnabled){
            m_Fusion.RegisterGlobalPoseWithProcessModel(pose,sysTime,sysTime,command);
        }else{
            m_Fusion.RegisterGlobalPose(pose,sysTime,sysTime);
        }

        if(lastTime == -1){
            lastTime = sysTime;
            nNumLocalizer = 0;
        }else if( (sysTime - lastTime) > 1 ){
            m_dLocalizerRate = nNumLocalizer / (sysTime - lastTime);
            //dout("Localizer rate is " << m_dLocalizerRate << " based on " << nNumLocalizer);
            nNumLocalizer = 0;
            lastTime = sysTime;
        }
        nNumLocalizer++;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
bool LearningGui::_SaveData(std::vector<std::string> *vArgs)
{
    if(m_pRegressionSample == NULL)
    {
        dout("Cannot save as there is no sample.");
        return false;
    }

    //open a new log file
    m_Logger.OpenNewLogFile("","learning_");
    m_Logger.SetIsReady(true);

    for(size_t ii = 0 ; ii < m_pRegressionSample->m_vStates.size() && ii < m_pRegressionSample->m_vCommands.size() ; ii++){
        m_Logger.LogPoseUpdate(m_pRegressionSample->m_vStates[ii],EventLogger::eLocalizer);
        m_Logger.LogControlCommand(m_pRegressionSample->m_vCommands[ii]);
    }
    m_Logger.CloseLogFile();

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool LearningGui::_LoadData(std::vector<std::string> *vArgs)
{
    if(m_pRegressionSample == NULL) {
        m_pRegressionSample = new MotionSample();
        m_nCollectedRegressionSamples = 0;
    }else{
        m_pRegressionSample->m_vStates.clear();
        m_pRegressionSample->m_vCommands.clear();
    }
    //clear all the lines
    _CommandHandler(eMochaClear);

    int counter = 0;
    m_Logger.ReadLogFile(vArgs->at(0));
    msg_Log msg;
    while(m_Logger.ReadMessage(msg))
    {
        if(msg.has_vehiclestate() == false || msg.has_controlcommand() == false){
            dout("A log message without both vehicle state and control command found. Aborting read.");
            return false;
        }else{
            VehicleState state;
            EventLogger::PoseUpdateSource source;
            m_Logger.ReadVehicleState(state,source,msg.vehiclestate());

            ControlCommand command;
            m_Logger.ReadCommand(command,msg.controlcommand());

            m_pRegressionSample->m_vStates.push_back(state);
            m_pRegressionSample->m_vCommands.push_back(command);

            //add this state to the trajectory
            m_Gui.SetCarState(m_nDriveCarId,state,true);

            dout("Loaded " << counter << " lines so far.");
            counter++;
        }
    }
    m_PlaybackSample = *m_pRegressionSample;
    m_bRefresh = true;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::Init(const std::string& sRefPlane, const std::string& sMeshName,
                       bool bLocalizerTransform, const Mode eMode,
                       const std::string& sLearningParamsFile,
                       const std::string& sCarMesh, const std::string& sWheelMesh)
{
    m_eMode = eMode;
    m_sLearningParamsFile = sLearningParamsFile;

    //initialize the scene
    //const aiScene *pScene = aiImportFile( "jump.blend", aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    const aiScene *pScene = aiImportFile( sMeshName.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    if(bLocalizerTransform){
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


    m_TerrainMesh.Init(pScene);

    m_Gui.Init(&m_TerrainMesh);

    //initialize the car parameters
    CarParameters::LoadFromFile(m_sLearningParamsFile,m_mParameters);

    m_FusionCarModel.Init(pScene, m_mParameters,1 );
    m_DriveCarModel.Init( pScene, m_mParameters,1 );
    m_nDriveCarId = m_Gui.AddCar(m_mParameters[CarParameters::WheelBase],m_mParameters[CarParameters::Width],
        sCarMesh, sWheelMesh);
    m_nPlayBackCarId = m_Gui.AddCar(m_mParameters[CarParameters::WheelBase],m_mParameters[CarParameters::Width],
        sCarMesh, sWheelMesh);
    m_Gui.SetCarVisibility(m_nDriveCarId,false);


    //parameters[CarParameters::Mass] = VEHICLE_DEFAULT_MASS*0.75;
    //parameters[CarParameters::MaxSteeringDegrees] = 25;
    m_mParameters[CarParameters::ControlDelay] *= 1.1;
    m_mParameters[CarParameters::Mass] *= 1.2;
    m_mParameters[CarParameters::SteeringCoef] *= 0.8;
    m_mParameters[CarParameters::AccelOffset] *= 0.9;


    m_LearningCarModel.Init(pScene, m_mParameters, REGRESSOR_NUM_WORLDS );


    //add all the parameters so they are tweakable
    for(const CarParameterPair& pair : m_LearningCarModel.GetParameters(0)){
        m_vTweakParams.push_back(RegressionParameter(m_LearningCarModel.GetParameters(0),pair.first,&m_LearningCarModel));
    }


    for(RegressionParameter& param : m_vTweakParams){
        std::string name("learning.Params.");
        name += param.m_sName;
        AttachCVar(name,&param,"");
    }

    CVarUtils::CreateCVar("learning.Save", SaveDataHandler, "");
    CVarUtils::CreateCVar("learning.Load", LoadDataHandler, "");

    //initialize the fusion
    if(m_eMode == Mode_Experiment){
        Eigen::Matrix4d dT_localizer_ref = Eigen::Matrix4d::Identity();
        if(sRefPlane.empty() == false){
            std::string word;
            std::stringstream stream(sRefPlane);
            std::vector<double> vals;
            while( getline(stream, word, ',') ){
                vals.push_back(boost::lexical_cast<double>(word));
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

        m_Fusion.ResetCurrentPose(Sophus::SE3d(),Eigen::Vector3d::Zero(),Eigen::Vector2d::Zero());
        Eigen::Vector6d T_ic;
        T_ic << 0.02095005375,  0.07656248358, -0.02323858462,   0.0433091783,  0.02323399838,    1.574031975;
        m_Fusion.SetCalibrationPose(Sophus::SE3d(fusion::Cart2T(T_ic)));
        m_Fusion.SetCalibrationActive(false);

        m_Node.subscribe("herbie/Imu");
        //dT_localizer_ref.block<3,3>(0,0).transposeInPlace();
        m_Localizer.TrackObject(m_sCarObjectName, "posetonode",Sophus::SE3d(dT_localizer_ref).inverse(),true);
        m_Localizer.Start();

        m_pImuThread = new boost::thread(boost::bind(&LearningGui::_ImuReadFunc,this));
        m_pLocalizerThread = new boost::thread(boost::bind(&LearningGui::_LocalizerReadFunc,this));
        m_pLearningCaptureThread = new boost::thread(boost::bind(&LearningGui::_LearningCaptureFunc,this));

        m_Gui.SetCarVisibility(m_nDriveCarId,true);
    }

    m_pPhysicsThread = new boost::thread(boost::bind(&LearningGui::_PhysicsFunc,this));


    //initialize the debug drawer
    //m_BulletDebugDrawer.Init(&m_DriveCarModel);

    //populate all the objects
    _PopulateSceneGraph();

    //set default vehicle state
    VehicleState state;
    state.m_dTwv = Sophus::SE3d(fusion::Cart2T(-5,0,-0.05,0,0,0));
    m_DriveCarModel.SetState(0,state);

    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'r', std::bind(&LearningGui::_CommandHandler, this, eMochaRestart) );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'c', std::bind(&LearningGui::_CommandHandler, this, eMochaClear) );
    pangolin::RegisterKeyPressCallback( '\r', [this]() {this->m_bStep = true;} );
    pangolin::RegisterKeyPressCallback( ' ', [this]() {this->m_bPaused = !this->m_bPaused;} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'l', [this] {CarParameters::LoadFromFile(m_sLearningParamsFile,m_mParameters);} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 's', [this] {CarParameters::SaveToFile(m_sLearningParamsFile,m_mParameters);} );


    m_pJoystickThread = new boost::thread(boost::bind(&LearningGui::_JoystickReadFunc,this));

    m_Regressor.Init(m_dLearnEps);


    //initialize the joystick
    if(m_Joystick.InitializeJoystick()) {
        std::cout << "Successfully initialized joystick" << std::endl;
    }else{
        std::cout << "Failed to initialized joystick" << std::endl;
    }

    //setup learning parameters
    //setup the learning parameters
    m_vDriveLearningParams.clear();
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::DynamicFrictionCoef));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::StaticSideFrictionCoef));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::ControlDelay));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::SteeringCoef));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::SteeringOffset));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::AccelOffset));

    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::Mass));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::TorqueSpeedSlope));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::StallTorqueCoef));
    //m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::SlipCoefficient));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::MaxSteering));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::Stiffness));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::CompDamping));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::ExpDamping));

    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::MagicFormula_B));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::MagicFormula_C));
    m_vDriveLearningParams.push_back(RegressionParameter(m_DriveCarModel.GetParameters(0),CarParameters::MagicFormula_E));

    /////////////////////////
    for(const RegressionParameter& parameter: m_vDriveLearningParams){
        m_vLearningParams.push_back(RegressionParameter(m_LearningCarModel.GetParameters(0),parameter.m_nKey));
    }

     m_LearningPanel.SetVar("learning:LearnOn",&m_bLearn)
                    .SetVar("learning:Playback",&m_bPlayback)
                    .SetVar("learning:MapSteering",&m_bMapSteering)
                    .SetVar("learning:dt",&m_dT)
                    .SetVar("learning:TotalRegressionSamples",&m_nTotalRegressionSamples)
                    .SetVar("learning:CollectedRegressionSamples",&m_nCollectedRegressionSamples)
                    .SetVar("learning:LearningParams",&m_vLearningParams)
                    .SetVar("learning:DriveParams",&m_vDriveLearningParams)
                    .SetVar("learning:ImuRate",&m_dImuRate)
                    .SetVar("learning:LocalizerRate",&m_dLocalizerRate)
                    .SetVar("learning:ProcessModelEnabled",&m_bProcessModelEnabled)
                    .SetVar("learning:Refresh",&m_bRefresh)
                    .SetVar("learning:Regress",&m_bRegress);

    m_LearningPanel.Init();
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_PopulateSceneGraph()
{
    //m_Gui.AddGLObject(&m_BulletDebugDrawer);
    m_Gui.AddPanel(&m_LearningPanel);
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_PhysicsFunc()
{
    double dCurrentTime = CarPlanner::Tic();
    double dT;
    int playBackSegment = 0, playBackSample = 0;
    m_Gui.SetCarVisibility(m_nDriveCarId,true);
    m_Gui.SetCarVisibility(m_nPlayBackCarId,false);
    while(1){
        if(m_bPlayback){
            m_Gui.SetCarVisibility(m_nPlayBackCarId,true);
            while(m_bPaused == true){
                if(m_bStep == true){
                    m_bStep = false;
                    break;
                }
                usleep(1000);
            }
            playBackSample++;
            MotionSample::FixSampleIndexOverflow(m_vSamples,playBackSegment,playBackSample);
            //show the playback vehicle
            int dataIndex = m_vSampleIndices[playBackSegment] + playBackSample;
            //wait for the dT
            while((CarPlanner::Toc(dCurrentTime)) < m_PlaybackSample.m_vCommands[dataIndex].m_dT) {
                usleep(100);
            }

            dCurrentTime = CarPlanner::Tic();

            //now set the position of the two cars
            m_Gui.SetCarState(m_nDriveCarId,m_PlaybackSample.m_vStates[dataIndex]);
            m_Gui.SetCarState(m_nPlayBackCarId,m_vSamples[playBackSegment].m_vStates[playBackSample]);

        }else{
            playBackSegment = 0;
            playBackSample = 0;
            //m_Gui.SetCarVisibility(m_nPlayBackCarId,false);
            if(m_eMode == Mode_Simulation){
                boost::this_thread::interruption_point();
                //update the drive car position based on the car model
                ControlCommand currentCommand;
                while((CarPlanner::Toc(dCurrentTime)) < m_dT) {
                    usleep(100);
                }

                dT = CarPlanner::Toc(dCurrentTime);
                //std::cout << "dt = " << dT << std::endl;
                dCurrentTime = CarPlanner::Tic();

                {
                    boost::mutex::scoped_lock lock(m_JoystickMutex);
                    currentCommand = m_JoystickCommand;
                }

                //get the current commands from the joystick
                currentCommand.m_dT = dT;

                VehicleState state;
                m_DriveCarModel.GetVehicleState(0,state);
                //update the learning system
                _UpdateLearning(currentCommand,state);

                //dout("Sending accel: "<< currentCommand.m_dForce << " and steering: " << currentCommand.m_dPhi);
                m_DriveCarModel.UpdateState(0,currentCommand,dT);



                if(m_Joystick.IsButtonPressed(6)){
                    _CommandHandler(eMochaRestart);
                }
            }else{
                usleep(100000);
            }
        }
    }
}

////////////////////////////////////////////////////////////////
void LearningGui::_UpdateLearning(ControlCommand command, VehicleState& state)
{
    if(m_bMapSteering == true){
        Eigen::Vector6d vec = fusion::T2Cart(state.m_dTwv.matrix());
        if(m_vSteeringPairs.size() == 0){
            m_dLastSteeringPose = vec;
            m_dLastCurv = 0;
            m_vSteeringPairs.push_back(std::pair<double,double>(0,0));
        }else{
            //calculate the curvature
            double curv = rpg::AngleWrap(fabs(vec[5]-m_dLastSteeringPose[5]))/(vec.head(3)-m_dLastSteeringPose.head(3)).norm();
            //m_dLastCurv = 0.1*curv + 0.9*m_dLastCurv;
            m_vSteeringPairs.push_back(std::pair<double,double>(command.m_dPhi,curv));
            if(m_vSteeringPairs.size() > 15000){
                std::fstream logFile;
                logFile.open("steeringmap.csv", std::ios::out | std::ios::in | std::ios::trunc);
                for(size_t ii = 0 ; ii < m_vSteeringPairs.size() ; ii++){
                    std::pair<double,double> pair = m_vSteeringPairs[ii];
                    logFile << std::setprecision(15) << pair.first << "," << pair.second << std::endl;
                }
                logFile.close();
                m_vSteeringPairs.clear();
                m_bMapSteering = false;
            }
            m_dLastSteeringPose = vec;
        }
    }

    bool bCanRegress = false;
    if(m_bLearn == true && m_bLearningRunning == false){
        //reset the regression sample
        if(m_nCollectedRegressionSamples == 0 && m_pRegressionSample != NULL){
            delete m_pRegressionSample;
            m_pRegressionSample = NULL;
        }

        if(m_pRegressionSample == NULL) {
            m_pRegressionSample = new MotionSample();
            m_nCollectedRegressionSamples = 0;
            //if we don't have a current regression plan, then create one
            m_pRegressionSample->m_vCommands.reserve(m_nTotalRegressionSamples);
            m_pRegressionSample->m_vStates.reserve(m_nTotalRegressionSamples);
        }

        //add this command to the regression sample
        if((int)m_pRegressionSample->m_vStates.size() < m_nTotalRegressionSamples && m_bLearningRunning == false) {
            //raycast the state to the ground
            Eigen::Vector3d dIntersect;
            if(m_DriveCarModel.RayCast(state.m_dTwv.translation(),GetBasisVector(state.m_dTwv,2)*0.2,dIntersect,true)){
               state.m_dTwv.translation() = dIntersect;
            }

            m_pRegressionSample->m_vStates.push_back(state);
            m_pRegressionSample->m_vCommands.push_back(command);
            m_nCollectedRegressionSamples++;
            //dout("Updating learning with dt" << command.m_dT);
        }else {
            bCanRegress = true;
        }
    }

    if(bCanRegress || m_bRegress){
        m_PlaybackSample = *m_pRegressionSample;
        if(m_pLearningThread == NULL){
            m_bLearningRunning = true;
            m_pLearningThread = new boost::thread(boost::bind(&LearningGui::_LearningFunc,this,m_pRegressionSample));
        }else if(m_bLearningRunning == false) {
            delete m_pLearningThread;
            m_bLearningRunning = true;
            m_pLearningThread = new boost::thread(boost::bind(&LearningGui::_LearningFunc,this,m_pRegressionSample));
        }

        //this is required to push back the button on the UI
        m_bRegress = false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_LearningCaptureFunc()
{
    while(1)
    {
        _SetPoseFromFusion();
        usleep(1E6*0.005);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_LearningFunc(MotionSample* pRegressionPlan)
{
    SetThreadName("Regression thread");
    m_bLearningRunning = true;

    //Update from the tweaked values to the learning params
    for(RegressionParameter& param: m_vTweakParams){
        //find the tweak param with the same tag
        for(RegressionParameter& learnParam: m_vLearningParams){
            if(learnParam.m_nKey == param.m_nKey){
                learnParam.m_dVal = param.m_dVal;
                break;
            }
        }
    }

    //at the end of it all, delete the plan
    std::vector<RegressionParameter> newParams;

    std::vector<RegressionParameter> currentParams = m_vLearningParams;

//    if(m_bLoggerEnabled){
//        _CreateLogFilesIfNeeded();
//        //m_fLearningLogFile << CarPlanner::Tic() - m_dStartTime << " " << currentParams.transpose() << " " <<  pRegressionPlan->m_vCommands.size() << std::endl;
//    }

    //m_Gui.SetStatusLineText(m_nLearningStatusId,(boost::format("Learning: starting with [%s]") % currentParams.transpose()).str());
    ApplyVelocitesFunctor5d functor(&m_LearningCarModel,Eigen::Vector3d::Zero());

    //run the regressor
    m_Regressor.Regress(functor,(MotionSample&)*pRegressionPlan,currentParams,newParams);
    m_vLearningParams = newParams;

    m_vSamples.clear();
    m_vSampleIndices.clear();
    m_Regressor.CalculateParamNorms(functor,(MotionSample&)*pRegressionPlan,newParams,&m_vSamples,&m_vSampleIndices);



    _UpdateTrajectories();

    //and now set the params on the car
    //dout("New parameters are " << newParams.transpose());
    //m_Gui.SetStatusLineText(m_nLearningStatusId,(boost::format("Learning: concluded with [%s]") % newParams.transpose()).str());

    //write to the log file
//    if(m_bLoggerEnabled){
//        _CreateLogFilesIfNeeded();
//        //m_fLearningLogFile << CarPlanner::Tic() - m_dStartTime << " " << newParams.transpose() << " " <<  pRegressionPlan->m_vCommands.size() << std::endl;
//    }

    m_LearningCarModel.UpdateParameters(newParams);

    //disable learning
    m_bLearn = false;
    m_bLearningRunning = false;
    m_nCollectedRegressionSamples = 0;

    //delete the regression plan
    //delete pRegressionPlan;

    //update all learnt value so that they can be tweaked
    for(RegressionParameter& param: m_vLearningParams){
        //find the tweak param with the same tag
        for(RegressionParameter& tweakParam: m_vTweakParams){
            if(tweakParam.m_nKey == param.m_nKey){
                tweakParam.m_dVal = param.m_dVal;
                break;
            }
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void LearningGui::_CommandHandler(const MochaCommands& command)
{
    VehicleState state;
    switch(command){
        case eMochaRestart:
            state.m_dTwv = Sophus::SE3d(fusion::Cart2T(0,-1.5,-0.01,0,0,0));
            m_DriveCarModel.SetState(0,state);
            break;

        case eMochaLearn:
            m_bLearn = !m_bLearn;
            break;

        case eMochaClear:
            m_Gui.ClearCarTrajectory(m_nDriveCarId);
            for (GLLineStrip& lineStrip : m_lGLLineSegments) {
                lineStrip.ClearLines();
            }
            break;

        default:
            break;
    }
}
