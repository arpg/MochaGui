
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MochaVehicle.h"
#include <pluginlib/class_list_macros.h>

namespace mochapc{

class MochaVehicleNodelet : public nodelet::Nodelet
{
public:
    // MochaVehicleNodelet(){}
    // ~MochaVehicleNodelet(){}

    virtual void onInit()
    {
        ros::NodeHandle& nh = this->getMTNodeHandle();
        ros::NodeHandle& private_nh = this->getMTPrivateNodeHandle();
        // ros::NodeHandle& nh = this->getNodeHandle();
        // ros::NodeHandle& private_nh = this->getPrivateNodeHandle();

        vehicle_.reset(new MochaVehicle(private_nh, nh));
      };
private:
  boost::shared_ptr<MochaVehicle> vehicle_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochapc::MochaVehicleNodelet, nodelet::Nodelet);
// PLUGINLIB_DECLARE_CLASS(mochagui, MochaPlannerNodelet, mochagui::MochaPlannerNodelet, nodelet::Nodelet)
