#include "ros/ros.h"
#include "nodelet/loader.h"
#include "MochaVehicle.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "mochavehicle_nodelet");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "mochagui/MochaVehicleNodelet", remap, nargv);
    ros::spin();
    return 0;
}