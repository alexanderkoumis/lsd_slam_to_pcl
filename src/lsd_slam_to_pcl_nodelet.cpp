#include "lsd_slam_to_pcl/lsd_slam_to_pcl_nodelet.hpp"

LSDSLAMToPCLNodelet::LSDSLAMToPCLNodelet() {}
LSDSLAMToPCLNodelet::~LSDSLAMToPCLNodelet() {}

void LSDSLAMToPCLNodelet::onInit()
{
    nh_ptr_ = boost::make_shared<ros::NodeHandle>(this->getPrivateNodeHandle());

    std::string name = nh_ptr_->getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos + 1);

    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
    controller_.reset(new LSDSLAMToPCL(*nh_ptr_, name));

    if (controller_->Init())
    {
        NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
    }
    else
    {
        NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
    }
}

PLUGINLIB_EXPORT_CLASS(LSDSLAMToPCLNodelet, nodelet::Nodelet);
