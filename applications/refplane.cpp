#include "config.h"
#include <fenv.h>
#include "GetPot"
#include "Vicon.h"
#include "../Hermes1.0/Node.h"
#include "Messages.pb.h"
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/lexical_cast.hpp>
#include <Mvlpp/Mvl.h>

Vicon g_vicon;

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{

    GetPot cl( argc, argv );

    g_vicon.TrackObject("Ref_Plane", "192.168.10.1",false);
    g_vicon.Start();
    //wait for the pose
    Eigen::Vector6d pose = mvl::T2Cart(g_vicon.GetPose("Ref_Plane",true).matrix());
    //now show the pose
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(10, 0, ",", ",", "", "");
    ss << mvl::Cart2T(pose).format(CleanFmt);
    std::string res = ss.str();
    boost::erase_all(res," ");
    std::cout << res  << std::endl;

    return 0;
}


