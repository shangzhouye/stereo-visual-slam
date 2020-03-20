#ifndef VISUAL_INCLUDE_GUARD_HPP
#define VISUAL_INCLUDE_GUARD_HPP
/// \file
/// \brief Visualization module

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace vslam
{

class VslamVisual
{

public:
    sensor_msgs::PointCloud2 feature_map_;
    ros::Publisher feature_map_publisher_;
    ros::Publisher fixed_pose_pub_;
    ros::Publisher pose_array_pub_;

public:
    VslamVisual() = default;

    VslamVisual(ros::NodeHandle &nh)
    {
        feature_map_publisher_ =
            nh.advertise<sensor_msgs::PointCloud2>("vslam/feature_map", 1);
        fixed_pose_pub_ = nh.advertise<visualization_msgs::Marker>("fixed_pose", 100);
        pose_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("keyframes", 100);
    }

    /*! \brief Convert opencv point3f 3D points to point cloud
    *
    *  \param points_3d - points generated in Opencv
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

    /*! \brief Publish fixed pose
    *
    *  \param frame - the frame of its pose to be published
    */
    void publish_fixed_pose(const Frame &frame);

    /*! \brief Create a marker for a frame
    *
    *  \param frame - the frame to be marked
    *  \return returns the marker created
    */
    visualization_msgs::Marker create_pose_marker(const Frame &frame);
};

} // namespace vslam

#endif