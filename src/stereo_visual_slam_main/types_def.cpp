/// \file
/// \brief Definition of frame struct

#include <stereo_visual_slam_main/types_def.hpp>

namespace vslam
{

Eigen::Vector3d Frame::find_3d(const cv::KeyPoint &kp, Eigen::Vector3d &relative_pt3d)
{
    double x = (kp.pt.x - cx_) / fx_;
    double y = (kp.pt.y - cy_) / fy_;
    double depth = fx_ * b_ / (disparity_.at<float>(kp.pt.y, kp.pt.x));

    relative_pt3d = Eigen::Vector3d(x * depth, y * depth, depth);

    return T_c_w_.inverse() * relative_pt3d;
}

void Frame::fill_frame(SE3 T_c_w, bool is_keyframe, int keyframe_id)
{
    T_c_w_ = T_c_w;
    is_keyframe_ = is_keyframe;
    if (is_keyframe)
    {
        keyframe_id_ = keyframe_id;
    }
}

} // namespace vslam
