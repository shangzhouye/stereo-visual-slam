#ifndef VO_INCLUDE_GUARD_HPP
#define VO_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for stereo visual odometry

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <stereo_visual_slam_main/visualization.hpp>
#include <stereo_visual_slam_main/optimization.hpp>

namespace vslam
{

enum TrackState
{
    Init,
    Track,
    Lost
};

class VO
{

public:
    // eigen macro for fixed-size vectorizable Eigen types
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame frame_last_;
    Frame frame_current_;

    std::vector<cv::Point3f> pts_3d_last_;
    std::vector<cv::KeyPoint> keypoints_last_;
    std::vector<cv::KeyPoint> keypoints_curr_;
    cv::Mat descriptors_last_;
    cv::Mat descriptors_curr_;
    std::vector<cv::DMatch> feature_matches_;

    std::string dataset_;
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_crosscheck_;

    int num_inliers_ = 0; // number of inliers after RANSAC

    SE3 T_c_l_ = SE3(); // T_current(camera)_last(camera)
    SE3 T_c_w_ = SE3(); // T_current(camera)_world

    TrackState state_ = Init; // current tracking state
    int seq_ = 1;             // sequence number
    int num_lost_ = 0;        // number of continuous lost frames
    // visualization module
    VslamVisual my_visual_;

public:
    VO(ros::NodeHandle &nh);

    /*! \brief initialize VO
    *
    *  \param dataset - the address of the dataset
    *  \param nh - the node handle
    */
    VO(std::string dataset, ros::NodeHandle &nh);

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
    * 2. disparity map of the last frame
    * 3. feature detection of the last frame
    * 4. filter features of the last frame with valid depth
    * 5. feature detection of the current frame
    * 6. feature matching
    * 7. solve PnP
    * 8. check if successful
    * 9. move frame
    * 10. Tcw and seq++
    * 
    *  \return if successful
    */
    bool initialization();

    /*! \brief tracking pipline
    *
    * The pipeline includes:
    * 1. read current frame
    * 2. disparity map of the last frame
    * 3. filter features of the last frame with valid depth
    * 4. feature detection of the current frame
    * 5. feature matching
    * 6. solve PnP
    * 7. check if successful
    * 8. move frame
    * 9. Tcw and seq++
    * 
    *  \return if successful
    */
    bool tracking();

    /*! \brief feature detection
    *
    *  \param img - the image to detect features
    *  \param descriptors - the descriptors detected
    *  \return if_successful
    */
    int feature_detection(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

    /*! \brief feature matching
    *
    *  \param descriptors_1 - the descriptors in first image
    *  \param descriptors_2 - the descriptors in second image
    *  \param feature_matches - feature matches
    *  \return if successful
    */
    int feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, std::vector<cv::DMatch> &feature_matches);

    /*! \brief set reference 3d positions for the last frame
    *
    * filter the descriptors in the reference frame, calculate the 3d positions of those features
    * 
    *  \return if successful
    */
    int set_ref_3d_position();

    /*! \brief visualize the feature detection and matching
    */
    void feature_visualize();

    /*! \brief estimate the motion using PnP
    */
    void motion_estimation();

    /*! \brief check if the estimated motion is valid
    * 
    *  \return if valid
    */
    bool check_motion_estimation();

    /*! \brief pipeline of the VO
    *
    *  \param ite_num - number of iterations
    */
    void VOpipeline(int ite_num);

    /*! \brief move everything from current frame to last frame
    */
    void move_frame();

    /*! \brief write pose to csv file
    * 
    * three cols are x, y and z respectively
    */
    void write_pose();

    /*! \brief publish pose, pointcloud, image to rviz
    * 
    */
    void rviz_visualize();

    /*! \brief adaptive non-maximal supression algorithm to generate uniformly distributed feature points
    *  \param keypoints - keypoints list
    *  \param num - number of keypoints to keep
    */
    void adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints,
                                         const int num);

    
    /*! \brief single frame nonlinear optimization using g2o after PnP
    *  \param points_3d - 3D landmark points
    *  \param points_2d - 2D corresponding points in the image
    *  \param K - intrinsic matrix K
    *  \param pose - the pose to be optimized
    */
    void single_frame_optimization(const G2OVector3d &points_3d, const G2OVector2d &points_2d,
                             const cv::Mat &K, Sophus::SE3d &pose);
};

} // namespace vslam

#endif