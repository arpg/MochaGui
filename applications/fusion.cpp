#include "CarPlanner/CarPlannerCommon.h"
#include "SensorFusionCeres.h"
#include<fstream>
#include <fenv.h>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "GetPot"
#include "EventLogger.h"

//enable floating point exceptions
#include <xmmintrin.h>
std::vector<SceneGraph::GLAxis*> vParamAxes;
std::vector<SceneGraph::GLLineStrip*> vImuStrips;
boost::mutex m_DrawMutex;
// Scenegraph to hold GLObjects and relative transformations
SceneGraph::GLSceneGraph glGraph;
SceneGraph::GLAxis glpos;
SceneGraph::GLGroup axisGroup;
int globalStartIndex = 0;
Eigen::IOFormat CleanFmt(10, 0, ", ", "\n" , "[" , "]");
bool m_bPaused = true;
bool m_bStep = false;
bool m_bFF;
bool m_bViconActive = true;
int m_nFilterSize = 50;
int m_bCalibrateActive = false;
pangolin::DataLog m_Log;
EventLogger m_Logger;
fusion::SensorFusionCeres g_fusion(m_nFilterSize);
int m_nImuSkip = 0;
static int& g_nGlobalSkip = CVarUtils::CreateGetCVar("debug.GlobalSkip",0);

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768


using namespace fusion;

enum FusionCommands
{
    Pause = 1,
    Step = 2,
    FastForward = 3
};

void CommandHandler(FusionCommands command)
{
    switch(command)
    {
        case Pause:
            m_bPaused = !m_bPaused;
        break;

        case Step:
            m_bStep = true;
        break;

        case FastForward:
            m_bFF = true;
        break;
    }
}


void DoFusion()
{
    //now that we've loaded all the poses, add them to the sensor fusion

    g_fusion.ResetCurrentPose(Sophus::SE3d(),Eigen::Vector3d::Zero(),Eigen::Vector2d::Zero());
    g_fusion.SetCalibrationActive(m_bCalibrateActive);
    Eigen::Vector6d T_ic;
    if(m_bCalibrateActive){
        T_ic.setZero();
    }else{
        T_ic << -0.003988648232, 0.003161519861,  0.02271876324, -0.02824564077, -0.04132003806,   -1.463881523;
    }
    g_fusion.SetCalibrationPose(Sophus::SE3d(mvl::Cart2T(T_ic)));
    Sophus::SE3d dT_ic = g_fusion.GetCalibrationPose();

    int imuIndex = 0, globalIndex = globalStartIndex;
    msg_Log msg;

    double time = -1;
    //g_fusion.m_bTimeCalibrated = true;
    //g_fusion.ResetCurrentPose(globalData[globalIndex].tail(6),Eigen::Vector3d(0,0,0),Eigen::Vector2d::Zero());
    while(1)
    {       
        if(m_bPaused){
            while(m_bStep == false && m_bPaused == true && m_bFF == false){
                usleep(1000);
            }
            //this is done only if we have read a new message and can proceed
            //m_bStep = false;
        }

        //set the timestamp if we don't already have one
        if(time == -1){
            //read the first message
            m_Logger.ReadMessage(msg);
            time = msg.timestamp();
        }else{
            msg = m_Logger.GetLastMessage();
        }

        bool bReadNextMessage = false;

        if(msg.has_imu()){
            const msg_ImuLog& imuMsg = msg.imu();
            if(time >= imuMsg.systemtime()){
                bReadNextMessage = true;
                imuIndex++;
                //push the imu data in
                if(imuIndex > m_nImuSkip){
                    Eigen::MatrixXd accel,gyro;
                    m_Logger.ReadMatrix(accel,imuMsg.accel());
                    m_Logger.ReadMatrix(gyro,imuMsg.gyro());
                    g_fusion.RegisterImuPose(accel(0)*G_ACCEL,accel(1)*G_ACCEL,accel(2)*G_ACCEL,gyro(0),gyro(1),gyro(2),imuMsg.devicetime(),imuMsg.systemtime());
                    m_Log.Log(accel(0)*G_ACCEL,accel(1)*G_ACCEL,accel(2)*G_ACCEL);
                    //PoseParameter param = g_fusion.GetCurrentPose();
                    //std::cout << "Imu data at time " << imuMsg.systemtime() << " is [" << accel.transpose() << "] [" << gyro.transpose() << "] vel " << param.m_dV.norm() << std::endl;
                    imuIndex = 0;
                }
            }
        }else if(msg.has_vicon()){
            const msg_ViconLog& viconMsg = msg.vicon();
            if(time >= viconMsg.systemtime()){
                bReadNextMessage = true;
                //push the imu data in
                globalIndex++;
                if(m_bViconActive && globalIndex >= g_nGlobalSkip){
                    globalIndex = 0;
                    m_bFF = false;

                    if( m_bPaused) {
                        while(m_bStep == false && m_bPaused == true && m_bFF == false){
                            usleep(1000);
                        }
                        m_bStep = false;
                        m_bFF = false;
                    }


                    //std::cout << "Global pose found at time " << t << ": " << globalData[globalIndex].tail(6).transpose().format(CleanFmt) << std::endl;
                    Eigen::MatrixXd pose7d;
                    m_Logger.ReadMatrix(pose7d,viconMsg.pose_7d());
                    g_fusion.RegisterGlobalPose(m_Logger.ReadPoseVector(pose7d),viconMsg.devicetime(),viconMsg.systemtime());

                    //print the calibration if needed
                    if(m_bCalibrateActive){
                        dT_ic = g_fusion.GetCalibrationPose();
                        std::cout << "Calibration: " << mvl::T2Cart(dT_ic.matrix()).transpose().format(CleanFmt) << std::endl;
                    }

                    //add an axis for this pose
                    std::list<PoseParameter>::iterator currentParam;
                    int count = 0;
                    for( currentParam = g_fusion.m_lParams.begin() ; currentParam != g_fusion.m_lParams.end() ; currentParam++ )
                    {
                        m_DrawMutex.lock();
                        std::list<PoseParameter>::iterator nextParam = currentParam;
                        std::vector<Sophus::SE3d> posesOut;
                        nextParam++;
                        if(nextParam != g_fusion.m_lParams.end()){
                            g_fusion._IntegrateImu(*currentParam,(*currentParam).m_dTime,(*nextParam).m_dTime,g_fusion.GetGravityVector(g_fusion.m_dG), &posesOut);


                            if((int)vImuStrips.size() == count){
                                vImuStrips.push_back(new SceneGraph::GLLineStrip());
                                glGraph.AddChild(vImuStrips.back());
                            }
                            //finish this
                            //rotate the poses out
                            Eigen::Vector3dAlignedVec v3dPoses(posesOut.size());
                            for(size_t ii = 0 ; ii < posesOut.size() ;ii++){
                                v3dPoses[ii] = (posesOut[ii] * dT_ic).translation();
                            }
                            vImuStrips[count]->SetPointsFromTrajectory(v3dPoses);
                        }

                        if((int)vParamAxes.size() == count*2){
                            vParamAxes.push_back(new SceneGraph::GLAxis(0.15));
                            glGraph.AddChild(vParamAxes.back());
                            vParamAxes.push_back(new SceneGraph::GLAxis(0.125));
                            glGraph.AddChild(vParamAxes.back());
                        }

                        Eigen::Matrix3d R = (*currentParam).m_dPose.so3().matrix();
                        //vParamAxes[count*2]->SetPose(mvl::Cart2T((*currentParam).m_dPose));
                        Eigen::Matrix4d global_pose = ((*currentParam).m_dPose * dT_ic ).matrix();
                        //vParamAxes[count*2]->SetPose(global_pose);
                        Eigen::Vector3d x = (*currentParam).m_dV.normalized();
                        Eigen::Vector3d y = x.cross(R.block<3,1>(0,2)).normalized();
                        Eigen::Vector3d z = y.cross(x).normalized();
                        R.block<3,1>(0,0) = x;
                        R.block<3,1>(0,1) = y;
                        R.block<3,1>(0,2) = z;
                        Eigen::Vector6d pose = mvl::T2Cart(global_pose);
                        pose.tail(3) = mvl::R2Cart(R);
                        vParamAxes[count*2 + 1]->SetPose(mvl::Cart2T(pose));
                        vParamAxes[count*2 + 1]->SetAxisSize((*currentParam).m_dV.norm()*0.1);
                        count++;
                        m_DrawMutex.unlock();
                    }

                    if(g_fusion.m_lParams.size() > 2){
                        currentParam = g_fusion.m_lParams.end();
                        currentParam--;currentParam--;currentParam--;
                    }

                //poseData.push_back(g_fusion.GetCurrentPose().m_dPose);
                    std::cout << " Gravity: " << g_fusion.GetGravityVector(g_fusion.m_dG).transpose().format(CleanFmt) <<
                                 " G: " << g_fusion.m_dG.transpose().format(CleanFmt) <<
                                 "Vel: " << g_fusion.GetCurrentPose().m_dV.transpose().format(CleanFmt) <<
                                 "LastVel: " << (*currentParam).m_dV.transpose().format(CleanFmt) << std::endl;
                }
            }

        }else if(msg.has_controlcommand()){
            //skip commands for now
            bReadNextMessage = true;
        }else{
            dout("Message does not have vicon, imu or control command. Aborting.");
        }

        if(bReadNextMessage){
            m_bStep = false;
            //read the next message or exit if we have finished
            if(m_Logger.ReadMessage(msg) == false){
                return;
            }
        }

        m_DrawMutex.lock();
        glpos.SetPose((g_fusion.m_CurrentPose.m_dPose*g_fusion.m_dTic).matrix());
        m_DrawMutex.unlock();

        time += 0.001;
        //usleep(1E6/4000.0);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    //enable floating point exceptions
    //_MM_SET_EXCEPTION_MASK(_MM_GET_EXCEPTION_MASK() & ~_MM_MASK_INVALID);

    GetPot cl( argc, argv );
    std::string sLog = cl.follow("20130226_142837",1,"-log");
    m_nFilterSize = cl.follow(50,1,"-length");
    m_bCalibrateActive = (bool)cl.follow(0,1,"-calib");

    g_fusion.SetFilterSize(m_nFilterSize);

    CVarUtils::AttachCVar("debug.FilterSize",g_fusion.GetFilterSizePtr(),"");
    CVarUtils::AttachCVar("debug.MaxIterations",g_fusion.GetIterationNumPtr(),"");


    //load the csv files for the IMU and Global poses (timestamped)
    //read in IMU data which should be organized as [syst t ax ay az wx wy wz] per row
    //imuData = LoadCsv("Matlab/imu_data.csv");
    //imuData = LoadCsv("/Users/nimski/Code/Build/MochaGui/logs_imu_20121211_205530.txt");
    char imuLog[500];
    sprintf(imuLog,"/Users/nimski/Code/Build/MochaGui/logs/%s",sLog.c_str());
    m_Logger.ReadLogFile(imuLog);


    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind("Main",WINDOW_WIDTH,WINDOW_HEIGHT);
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glewInit();

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState stacks3d(
        pangolin::ProjectionMatrix(WINDOW_WIDTH,WINDOW_HEIGHT,420,420,WINDOW_WIDTH/2,WINDOW_HEIGHT/2,0.1,1000),
        pangolin::ModelViewLookAt(2,0,-4, 0,1,0, pangolin::AxisNegZ)
    );
    pangolin::View view3d;
    view3d.SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetHandler(new SceneGraph::HandlerSceneGraph(glGraph,stacks3d,pangolin::AxisNegZ))
          .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));

    pangolin::View& graphView = pangolin::Plotter( &m_Log)
            .SetBounds(0.0, 0.3, 0.6, 1.0);

    pangolin::RegisterKeyPressCallback( '\r' , boost::bind(CommandHandler, Step ) );
    pangolin::RegisterKeyPressCallback( ' ', boost::bind(CommandHandler, Pause ) );
    pangolin::RegisterKeyPressCallback( 'f', boost::bind(CommandHandler, FastForward ) );
    pangolin::RegisterKeyPressCallback( 'v', [] { m_bViconActive = !m_bViconActive;
                                                                 std::cout << "Vicon poses " << (m_bViconActive ? "active" : "inactive") << std::endl; });


    pangolin::DisplayBase().AddDisplay(view3d);
    pangolin::DisplayBase().AddDisplay(graphView);
    std::vector<std::string> vLabels = {std::string("Ax"), std::string("Ay"), std::string("Az")};
    m_Log.SetLabels(vLabels);


    SceneGraph::GLGrid glGrid(50,2.0,false);
    glGrid.SetPlaneColor(SceneGraph::GLColor(90,90,90,128));
    glGraph.AddChild(&glGrid);

    //the current position

    glGraph.AddChild(&glpos);

    SceneGraph::GLLineStrip gllinestrip;
    glGraph.AddChild(&gllinestrip);

    glClearColor(0,0,0,0);


    //add all the global poses
    msg_Log logMsg;
    while(m_Logger.ReadMessage(logMsg)){
        if(logMsg.has_vicon()){
            Eigen::MatrixXd poseVec;
            m_Logger.ReadMatrix(poseVec,logMsg.vicon().pose_7d());
            SceneGraph::GLAxis* axis = new SceneGraph::GLAxis(0.02);
            axis->SetPose(m_Logger.ReadPoseVector(poseVec).matrix());
            axisGroup.AddChild(axis);
        }
    }

    m_Logger.ReadLogFile(imuLog);

    axisGroup.CompileAsGlCallList();
    glGraph.AddChild(&axisGroup);

    boost::thread fusionThread(DoFusion);

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        m_DrawMutex.lock();

        {

            view3d.ActivateScissorAndClear();

            // Swap frames and Process Events

            pangolin::FinishGlutFrame();
        }
        m_DrawMutex.unlock();

        // Pause for 1/60th of a second.
        usleep(1E6/100.0);
    }
    return 0;
}


