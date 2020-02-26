/// \file
/// \brief Definition of frame struct

#include <stereo_visual_slam_main/types_def.hpp>

namespace vslam
{

Eigen::Vector3d Frame::find_3d(const cv::KeyPoint &kp)
{
    double x = (kp.pt.x - cx_) / fx_;
    double y = (kp.pt.y - cy_) / fy_;
    double depth = fx_ * b_ / (disparity_.at<float>(kp.pt.y, kp.pt.x));

    return Eigen::Vector3d(x * depth, y * depth, depth);
}

} // namespace vslam
