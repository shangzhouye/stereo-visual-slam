#ifndef VISUAL_INCLUDE_GUARD_HPP
#define VISUAL_INCLUDE_GUARD_HPP
/// \file
/// \brief Visualization module

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace vslam
{

class VslamVisual
{
    
public:
    sensor_msgs::PointCloud2 feature_map_;
    ros::Publisher feature_map_publisher_;

public:
    VslamVisual() = default;

    VslamVisual(ros::NodeHandle &nh)
    {
        feature_map_publisher_ =
            nh.advertise<sensor_msgs::PointCloud2>("vslam/feature_map", 1);
    }

    /*! \brief Convert opencv point3f 3D points to point cloud
    *
    *  \param points_3d - points generated in Opencv
    *  \param feature_map - the variable to store feature map
    *  \return if successful
    */
    int points_to_feature_map(const std::vector<cv::Point3f> &point_3d);

    /*! \brief Publish the feature map
    *
    *  \param points_3d - points generated in Opencv
    *  \return if successful
    */
    int publish_feature_map(const std::vector<cv::Point3f> &point_3d);

    /*! \brief Publish the transformation
    *
    *  \param T_c_w - T_current(camera)_world
    *  \return if successful
    */
    int publish_transform(const SE3 &T_c_w);
};

} // namespace vslam

#endif