#ifndef STRUCTURELESSVO_INCLUDE_GUARD_HPP
#define STRUCTURELESSVO_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for structureless stereo visual odometry

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <vector>
#include <string>
#include <unistd.h>

using namespace std;
using namespace Eigen;

namespace vslam
{

class StructurelessVO
{

public:
    // eigen macro for fixed-size vectorizable Eigen types
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Frame frame_last_;
    Frame frame_current_;

    vector<cv::Point3f> pts_3d_last_;
    cv::Mat descriptors_last_;
    cv::Mat descriptors_curr_;
    vector<cv::DMatch> feature_matches_;

    string dataset_;
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_crosscheck_;

public:
    StructurelessVO();

    /*! \brief initialize StructurelessVO
    *
    *  \param dataset - the address of the dataset
    */
    StructurelessVO(string dataset);

    /*! \brief read left and right images into a frame
    *
    *  \param id - id of the frame, starts from zero
    *  \return if the reading is successful
    */
    int read_img(int id, cv::Mat &left_img, cv::Mat &right_img);

    /*! \brief calculate the disparity map from two left-right images
    *
    *  \param frame - a frame includes both L/R images
    *  \return if the calculation is successful
    */
    int disparity_map(const Frame &frame, cv::Mat &disparity);

    /*! \brief initialization pipline
    *
    * The pipeline includes:
    * 1. read two frames
    * 2. calculate the disparity map
    * 3. feature detection and matching
    * 4. calculate 3D position using camera model
    * 5. solve PnP problem
    * 
    *  \return if successful
    */
    int initialization();

    /*! \brief feature detection
    *
    *  \param img - the image to detect features
    *  \param descriptors - the descriptors detected
    *  \return if_successful
    */
    int feature_detection(const cv::Mat &img, cv::Mat &descriptors);

    /*! \brief feature matching
    *
    *  \param descriptors_1 - the descriptors in first image
    *  \param descriptors_2 - the descriptors in second image
    *  \param feature_matches - feature matches
    *  \return if successful
    */
    int feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, vector<cv::DMatch> &feature_matches);

    /*! \brief set reference 3d positions for the last frame
    *
    *  \return if successful
    */
    int set_ref_3d_position();

};

} // namespace vslam

#endif