#ifndef TYPES_INCLUDE_GUARD_HPP
#define TYPES_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of frame struct

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>

namespace vslam
{

struct Frame;
struct Landmark;
struct Feature;

struct Feature
{
public:
    double id_;
    Frame *frame_ = nullptr;
    Landmark *landmark_ = nullptr;
    cv::KeyPoint position_; // 2d position in the pixel frame
    cv::Mat descriptor_;    // feature descriptor of this landmark

public:
    Feature() {}

    Feature(double id, cv::KeyPoint position, cv::Mat descriptor)
        : id_(id), position_(position), descriptor_(descriptor) {}
};

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
    
    SE3 T_c_w_ = SE3(); // T_current(camera)_world

    bool is_keyframe_;
    int keyframe_id_;
    std::vector<Feature> features_;


    // camera intrinsic parameters
    double fx_ = 718.856, fy_ = 718.856, cx_ = 607.1928, cy_ = 185.2157;
    double b_ = 0.573;

public:
    // functions
    Frame() = default;

    Frame(int id, double timestamp, const cv::Mat &left, const cv::Mat &right)
        : id_(id), time_stamp_(timestamp), left_img_(left), right_img_(right) {}

    Eigen::Vector3d find_3d(const cv::KeyPoint &kp);

    /*! \brief fill in the remaining information needed for the frame
    *
    */
    void fill_frame(SE3 T_c_w, bool is_keyframe, int keyframe_id);

    /*! \brief add features to the frame
    *
    */
    void add_feature(Feature feature);
};

struct Landmark
{

public:
    double id_;
    cv::Point3f pt_3d_;      // 3d position in the world frame;
    cv::Mat descriptor_;     // feature descriptor of this landmark
    int observed_times_ = 0; // number of times being observed
    std::list<Feature*> observations_;

public:
    Landmark() {}

    Landmark(double id, cv::Point3f pt_3d, cv::Mat descriptor)
        : id_(id), pt_3d_(pt_3d), descriptor_(descriptor) {}
};

} // namespace vslam

#endif