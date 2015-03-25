#include <fenv.h>
#include <iostream>
#include <Node/Node.h>
#include <CarPlanner/Vicon.h>

#include "config.h"
#include "GetPot"
#include "Messages.pb.h"
#include "SE3.h"

Vicon g_vicon;

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    std::cout << "Fix refplane.cpp" << std::endl;
    GetPot cl( argc, argv );

    g_vicon.TrackObject("Ref_Plane", "192.168.10.1",false);
    g_vicon.Start();
    //wait for the pose
    //Eigen::Vector6d pose = mvl::T2Cart(g_vicon.GetPose("Ref_Plane",true).matrix()); //crh
    //now show the pose
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(10, 0, ",", ",", "", "");
    //ss << mvl::Cart2T(pose).format(CleanFmt); //crh
    std::string res = ss.str();
    //boost::erase_all(res," "); //crh
    std::cout << res  << std::endl;

    return 0;
}


