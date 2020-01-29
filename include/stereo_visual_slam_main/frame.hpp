#ifndef FRAME_INCLUDE_GUARD_HPP
#define FRAME_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of frame struct

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>

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

public:
    // functions
    Frame() = default;
    Frame(int id, double timestamp, const Mat &left, const Mat &right);


};

} // namespace vslam

#endif