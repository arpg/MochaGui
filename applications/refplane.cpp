#include <fenv.h>
#include <iostream>
#include <Node/Node.h>
#include "MochaGui/Localizer.h"
#include "MochaGui/SE3.h"

#include "config.h"
#include "Messages.pb.h"


/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    Localizer localizer;

    localizer.TrackObject("object_tracker", "refplane",false); //crh node call
    localizer.Start();
    //wait for the pose
    Eigen::Vector6d pose = T2Cart(localizer.GetPose("refplane",true).matrix());
    //now show the pose
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(10, 0, ",", ",", "", "");
    ss << Cart2T(pose).format(CleanFmt);
    std::string res = ss.str();
    std::string::iterator end_pos = std::remove(res.begin(), res.end(), ' ');
    res.erase(end_pos, res.end());
    LOG(INFO) << "Residual is: " << res;

    return 0;
}


