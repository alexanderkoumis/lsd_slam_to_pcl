#ifndef LSD_SLAM_TO_PCL_HPP
#define LSD_SLAM_TO_PCL_HPP

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "third_party/sophus/sim3.hpp"

#include "lsd_slam_to_pcl/keyframeMsg.h"

struct InputPointDense
{
    float idepth;
    float idepth_var;
    unsigned char color[4];
};

class LSDSLAMToPCL
{
public:
    LSDSLAMToPCL(ros::NodeHandle& nh, std::string& name);
    ~LSDSLAMToPCL();

    bool Init();

private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_subscriber_;
    ros::Publisher cloud_publisher_;
    ros::Publisher indices_publisher_;

    pcl::PointXYZRGB point_invalid_;
    std::string node_name_;
    int sparsify_factor_;
    int min_near_support_;
    float scaled_depth_var_thresh_;
    float abs_depth_var_thresh_;

    void depthCB(const lsd_slam_to_pcl::keyframeMsgConstPtr msg);
};

#endif