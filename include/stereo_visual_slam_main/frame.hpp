#ifndef FRAME_INCLUDE_GUARD_HPP
#define FRAME_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of frame struct

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>

using namespace std;
using namespace Eigen;

namespace vslam
{

struct Frame
{

public:
    // eigen macro for fixed-size vectorizable Eigen types
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // data memebers
    int id_;
    double time_stamp_;
    cv::Mat left_img_, right_img_;
    cv::Mat disparity_;

    // camera intrinsic parameters
    double fx_ = 718.856, fy_ = 718.856, cx_ = 607.1928, cy_ = 185.2157;
    double b_ = 0.573;

public:
    // functions
    Frame() = default;

    Frame(int id, double timestamp, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(timestamp), left_img_(left), right_img_(right) {}

    Vector3d find_3d(const cv::KeyPoint& kp);
};

} // namespace vslam

#endif