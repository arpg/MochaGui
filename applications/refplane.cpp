#include <fenv.h>
#include <iostream>
#include "Node/Node.h"
#include "CarPlanner/Vicon.h"
#include "MochaGui/SE3.h"

#include "config.h"
#include "MochaGui/GetPot"
#include "Messages.pb.h"

Vicon g_vicon;

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    GetPot cl( argc, argv );

    g_vicon.TrackObject("Ref_Plane", "192.168.10.1",false); //crh vicon
    g_vicon.Start();
    //wait for the pose
    Eigen::Vector6d pose = T2Cart(g_vicon.GetPose("Ref_Plane",true).matrix());
    //now show the pose
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(10, 0, ",", ",", "", "");
    ss << Cart2T(pose).format(CleanFmt);
    std::string res = ss.str();
    std::string::iterator end_pos = std::remove(res.begin(), res.end(), ' ');
    res.erase(end_pos, res.end());
    std::cout << res  << std::endl;

    return 0;
}


