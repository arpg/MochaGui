
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MochaPlanner.h"
#include <pluginlib/class_list_macros.h>

namespace mochapc{

class MochaPlannerNodelet : public nodelet::Nodelet
{
public:
    // MochaPlannerNodelet(){}
    // ~MochaPlannerNodelet(){}

    virtual void onInit()
    {
        ros::NodeHandle& nh = this->getMTNodeHandle();
        ros::NodeHandle& private_nh = this->getMTPrivateNodeHandle();

        planner_.reset(new MochaPlanner(private_nh, nh));

        // ros::Duration(3.0).sleep();
    };
private:
  boost::shared_ptr<MochaPlanner> planner_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochapc::MochaPlannerNodelet, nodelet::Nodelet);
// PLUGINLIB_DECLARE_CLASS(mochagui, MochaPlannerNodelet, mochagui::MochaPlannerNodelet, nodelet::Nodelet)
