/// \file
/// \brief Definition of frame struct

#include <stereo_visual_slam_main/frame.hpp>

namespace vslam
{

Frame::Frame() {}

Frame::Frame(int id, double timestamp, const Mat &left, const Mat &right)
    : id_(id), time_stamp_(timestamp), left_img_(left), right_img_(right) {}

} // namespace vslam
