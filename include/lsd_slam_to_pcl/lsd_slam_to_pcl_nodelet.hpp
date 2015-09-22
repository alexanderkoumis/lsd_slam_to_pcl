#ifndef LSD_SLAM_TO_PCL_NODELET_HPP
#define LSD_SLAM_TO_PCL_NODELET_HPP

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lsd_slam_to_pcl/lsd_slam_to_pcl.hpp"

class LSDSLAMToPCLNodelet : public nodelet::Nodelet
{
public:
    LSDSLAMToPCLNodelet();
    ~LSDSLAMToPCLNodelet();

    virtual void onInit();

private:
    boost::shared_ptr<LSDSLAMToPCL> controller_;
    boost::shared_ptr<ros::NodeHandle> nh_ptr_;

};

#endif