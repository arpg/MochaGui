#include <fenv.h>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/lexical_cast.hpp>
#include <node/Node.h>
#include <CarPlanner/RpgUtils.h>
#include <CarPlanner/Localizer.h>
#include <CarPlanner/SE3.h>

#include "config.h"
#include "GetPot"
#include "CarMessages.pb.h"

Localizer g_localizer;

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{

    GetPot cl( argc, argv );

    /*
    g_localizer.TrackObject("Ref_Plane", "posetonode",false);
    g_localizer.Start();
    //wait for the pose
    Eigen::Vector6d pose = rpg::T2Cart(g_localizer.GetPose("Ref_Plane",true).matrix());
    //now show the pose
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(10, 0, ",", ",", "", "");
    ss << rpg::Cart2T(pose).format(CleanFmt);
    std::string res = ss.str();
    boost::erase_all(res," ");
    std::cout << res  << std::endl;
    */

    return 0;
}


