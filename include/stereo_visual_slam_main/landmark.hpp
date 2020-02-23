#ifndef LANDMARK_INCLUDE_GUARD_HPP
#define LANDMARK_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of landmark structure

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>

namespace vslam
{

struct Landmark
{

public:
    double id_;
    cv::Point3f pts_3d_; // 3d position in the world frame;
    cv::Mat descriptor_; // feature descriptor of this landmark

public:
    Landmark(double id, cv::Point3f pts_3d, cv::Mat descriptor)
        : id_(id), pts_3d_(pts_3d), descriptor_(descriptor) {}
};

} // namespace vslam

#endif