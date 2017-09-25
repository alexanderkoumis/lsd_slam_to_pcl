# LSD-SLAM to PCL

This package is a nodelet that converts [LSD_SLAM](https://github.com/tum-vision/lsd_slam) keyframes into PCL pointclouds (`pcl::PointCloud<pcl::PointXYZRGB>`).

To build:

```bash
cd ~/catkin_ws/src
git clone https://github.com/alexanderkoumis/lsd_slam_to_pcl
cd ..
catkin_make
```

Using it in a launch file:
```xml
<launch>

    <!-- Nodelet manager -->
    <node pkg="nodelet" type="nodelet" name="nodelet_manager" args="manager" output="screen"/>

    <!-- LSD SLAM -> PCL pointcloud converstion -->
    <node pkg="nodelet" type="nodelet" name="lsd_slam_to_pcl" args="load LSDSLAMToPCLNodelet nodelet_manager" output="screen">
         <remap from="~input" to="/lsd_slam/keyframes"/>
    </node>

</launch>
```

Topics `lsd_slam_to_pcl/output_points` and `lsd_slam_to_pcl/output_indices` can now be subscribed to as `pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr` and `pcl::PointIndicesConstPtr`, respectively.

