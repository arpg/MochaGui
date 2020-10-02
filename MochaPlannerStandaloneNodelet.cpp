#include "ros/ros.h"
#include "nodelet/loader.h"
#include "MochaPlanner.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "mocha_manager");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "mochapc/MochaPlannerNodelet", remap, nargv);
    ros::spin();
    return 0;
}