/// \file
/// \brief Definition of frame struct

#include <stereo_visual_slam_main/frame.hpp>

namespace vslam
{

// bug narrowed down: when a class needs to be initialized
// without parameters (Frame new_frame;), the custom default
// constructor does not work.
// practice: have a default constructor defined by Frame() = default;

Eigen::Vector3d Frame::find_3d(const cv::KeyPoint &kp)
{
    double x = (kp.pt.x - cx_) / fx_;
    double y = (kp.pt.y - cy_) / fy_;
    double depth = fx_ * b_ / (disparity_.at<float>(kp.pt.y, kp.pt.x));

    return Eigen::Vector3d(x * depth, y * depth, depth);
}

} // namespace vslam
