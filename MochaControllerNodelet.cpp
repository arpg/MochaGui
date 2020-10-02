
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MochaController.h"
#include <pluginlib/class_list_macros.h>

namespace mochapc{

class MochaControllerNodelet : public nodelet::Nodelet
{
public:
    // MochaControllerNodelet(){}
    // ~MochaControllerNodelet(){}

    virtual void onInit()
    {
        ROS_INFO("Init'ing Controller Nodelet.");

        ros::NodeHandle& nh = this->getMTNodeHandle();
        ros::NodeHandle& private_nh = this->getMTPrivateNodeHandle();

        controller_.reset(new MochaController(private_nh, nh));

        ROS_INFO("Done init'ing Controller Nodelet.");
      };
private:
  boost::shared_ptr<MochaController> controller_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochapc::MochaControllerNodelet, nodelet::Nodelet);
