
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "MochaPlanner.h"
#include <pluginlib/class_list_macros.h>

namespace mochagui{

class MochaPlannerNodelet : public nodelet::Nodelet
{
public:
    // MochaPlannerNodelet(){}
    // ~MochaPlannerNodelet(){}

    virtual void onInit()
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& private_nh = this->getPrivateNodeHandle();

        planner_.reset(new MochaPlanner(private_nh, nh));
    };
private:
  boost::shared_ptr<MochaPlanner> planner_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochagui::MochaPlannerNodelet, nodelet::Nodelet);
// PLUGINLIB_DECLARE_CLASS(mochagui, MochaPlannerNodelet, mochagui::MochaPlannerNodelet, nodelet::Nodelet)
