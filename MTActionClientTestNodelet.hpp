
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MTActionClientTest.hpp"
#include <pluginlib/class_list_macros.h>

namespace mochapc{

class MTActionClientTestNodelet : public nodelet::Nodelet
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

        clinet_.reset(new MTActionClientTest(private_nh, nh));
      };
private:
  boost::shared_ptr<MTActionClientTest> client_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochapc::MTActionClientTestNodelet, nodelet::Nodelet);
