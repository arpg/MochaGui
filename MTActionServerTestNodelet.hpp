
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MTActionServerTest.hpp"
#include <pluginlib/class_list_macros.h>

namespace mochapc{

class MTActionServerTestNodelet : public nodelet::Nodelet
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

        server_.reset(new MTActionServerTest(private_nh, nh));
      };
private:
  boost::shared_ptr<MTActionServerTest> server_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochapc::MTActionServerTestNodelet, nodelet::Nodelet);
