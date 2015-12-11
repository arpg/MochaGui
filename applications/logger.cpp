#include <fenv.h>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/lexical_cast.hpp>
#include <Node/Node.h>
#include <CarPlanner/BulletCarModel.h>
#include <CarPlanner/Vicon.h>

#include "config.h"
#include "GetPot"
#include "Messages.pb.h"
#include "SE3.h"
#include "JoystickHandler.h"
#include "EventLogger.h"

double g_dStartTime = Tic();
Vicon g_vicon;
node::node g_node;
EventLogger logger;
bool g_bLog = false;
JoystickHandler joystick;

#define DEFAULT_ACCEL_COEF 5.65
#define DEFAULT_STEERING_COEF -650
#define DEFAULT_ACCEL_OFFSET 245.35
#define DEFAULT_STEERING_OFFSET 255


void JoystickFunc()
{
    while(1){
        boost::this_thread::interruption_point();
        joystick.UpdateJoystick();
        double joystickAccel,joystickPhi;
        joystickAccel = (((double)joystick.GetAxisValue(1)/JOYSTICK_AXIS_MAX)*-40.0);
        joystickPhi = (((double)joystick.GetAxisValue(2)/(double)JOYSTICK_AXIS_MAX) * (MAX_SERVO_ANGLE*M_PI/180.0)*0.5);

        joystickAccel = joystickAccel*DEFAULT_ACCEL_COEF + DEFAULT_ACCEL_OFFSET;
        joystickPhi = joystickPhi*DEFAULT_STEERING_COEF + DEFAULT_STEERING_OFFSET;

        ControlCommand command;
        command.m_dForce = joystickAccel;
        command.m_dPhi = joystickPhi;
        if(g_bLog ){
            logger.LogControlCommand(command);
            std::cout << "Joystick commands logged at:" << Tic()-g_dStartTime << "seconds [" << joystickAccel << " " << joystickPhi << "]" << std::endl;
        }

        CommandMsg Req;
        CommandReply Rep;

        Req.set_accel(command.m_dForce);
        Req.set_phi(command.m_dPhi);
        g_node.call_rpc("herbie","ProgramControlRpc",Req,Rep);

        usleep(2000);
    }
}

void ImuReadFunc()
{
    while(1){
        boost::this_thread::interruption_point();
        Imu_Accel_Gyro Msg;
        if(g_node.receive("herbie/Imu",Msg)){
            double time = (double)Msg.timer()/62500.0;
            if(g_bLog ){
                std::cout << "IMU pose received at:" << time << "seconds [" << Msg.accely() << " " <<  -Msg.accelx() << " " << Msg.accelz() << "]" << std::endl;
                fflush(stdout);
                logger.LogImuData(Tic(),time,Eigen::Vector3d(Msg.accelx(),Msg.accely(),Msg.accelz()),Eigen::Vector3d(Msg.gyrox(),Msg.gyroy(),Msg.gyroz()));
            }
        }
    }

}

void ViconReadFunc()
{
    while(1){
        boost::this_thread::interruption_point();
        //this is a blocking call
        double viconTime;
        Sophus::SE3d Twb = g_vicon.GetPose("CAR",true,&viconTime);
        Eigen::Vector6d pose = fusion::T2Cart(Twb.matrix());

        if(g_bLog ){
            std::cout << "Vicon pose received at:" << viconTime-g_dStartTime << "seconds [" << pose[0] << " " <<  pose[1] << " " << pose[2] << "]" <<  std::endl;
            fflush(stdout);
            logger.LogViconData(Tic(),viconTime,Twb);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{


    GetPot cl( argc, argv );
    std::string sRef = cl.follow( "", 1, "-ref" );
    bool bLogCommands = cl.search("-logCommands");

    Eigen::Matrix4d dT_vicon_ref = Eigen::Matrix4d::Identity();
    if(sRef.empty() == false){
        std::string word;
        std::stringstream stream(sRef);
        std::vector<double> vals;
        while( getline(stream, word, ',') ){
            vals.push_back(boost::lexical_cast<double>(word));
        }
        if(vals.size() != 16){
            std::cout << "Attempted to read in reference plane position, but incorrect number of matrix entries provided. Must be 16 numbers" << std::endl;
        }else{

            for(int ii = 0 ; ii < 4 ; ii++){
                for(int jj = 0 ; jj < 4 ; jj++){
                    dT_vicon_ref(ii,jj) = vals[ii*4 + jj];
                }
            }
            std::cout << "Ref plane matrix successfully read" << std::endl;
        }
    }

    g_node.init("logger");

    g_node.subscribe("herbie/Imu");
    g_vicon.TrackObject("CAR", "192.168.10.1",Sophus::SE3d(dT_vicon_ref).inverse(),true);
    g_vicon.Start();

    boost::thread* pImuThread = new boost::thread(boost::bind(ImuReadFunc));
    boost::thread* pViconThread = new boost::thread(boost::bind(ViconReadFunc));
    boost::thread* pJoystickThread = NULL;
    if(bLogCommands){
        //initialize the joystick
        if(joystick.InitializeJoystick()) {
            std::cout << "Successfully initialized joystick" << std::endl;
        }else{
            std::cout << "Failed to initialized joystick" << std::endl;
        }

         pJoystickThread = new boost::thread(boost::bind(JoystickFunc));
    }



    std::cout << "Press enter to start logging." << std::endl;
    getchar();

    g_dStartTime = Tic();

    std::string logFile = logger.OpenNewLogFile("","fusion_");
    std::cout << "Opened log file " << logFile << std::endl;

    //start logging
    g_bLog = true;

    std::cout << "Press enter to stop logging." << std::endl;

    getchar();
    g_bLog = false;

    pImuThread->interrupt();
    pImuThread->join();

    pViconThread->interrupt();
    pViconThread->join();

    pJoystickThread->interrupt();
    pJoystickThread->join();

    //g_vicon.Stop();


    return 0;
}

