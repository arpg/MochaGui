#include <fenv.h>
#include <iostream>
#include "Node/Node.h"
#include "CarPlanner/Vicon.h"
#include "SensorFusion/SE3.h"

#include "config.h"
#include "MochaGui/GetPot"
#include "Messages.pb.h"

Vicon g_vicon;

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    std::cout << "Fix refplane.cpp" << std::endl;
    GetPot cl( argc, argv );

    g_vicon.TrackObject("Ref_Plane", "192.168.10.1",false);
    g_vicon.Start();
    //wait for the pose
    Eigen::Vector6d pose = fusion::T2Cart(g_vicon.GetPose("Ref_Plane",true).matrix());
    //now show the pose
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(10, 0, ",", ",", "", "");
    ss << fusion::Cart2T(pose).format(CleanFmt);
    std::string res = ss.str();
    //boost::erase_all(res," "); //crh
    std::cout << res  << std::endl;

    return 0;
}


