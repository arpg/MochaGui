
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <infinitam/UIEngine.h>

namespace mochagui{

class MochaInfiniTAMNodelet : public nodelet::Nodelet
{
public:
    // MochaPlannerNodelet(){}
    // ~MochaPlannerNodelet(){}

    virtual void onInit()
    {
        ros::NodeHandle& nh = this->getNodeHandle();
        ros::NodeHandle& private_nh = this->getPrivateNodeHandle();

        infinitam_.reset(new InfiniTAM::Engine::UIEngine());
        infinitam_->SetNodeHandles(private_nh, nh);
        infinitam_->InitialiseWithDefaults(0,NULL);
        infinitam_->Run();
        infinitam_->Shutdown();
    };
private:
  boost::shared_ptr<InfiniTAM::Engine::UIEngine> infinitam_;
};

} // namespace @(namespace)

PLUGINLIB_EXPORT_CLASS(mochagui::MochaInfiniTAMNodelet, nodelet::Nodelet);
// PLUGINLIB_DECLARE_CLASS(mochagui, MochaPlannerNodelet, mochagui::MochaPlannerNodelet, nodelet::Nodelet)
