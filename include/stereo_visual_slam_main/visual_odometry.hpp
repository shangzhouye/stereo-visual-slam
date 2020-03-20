#ifndef VO_INCLUDE_GUARD_HPP
#define VO_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for stereo visual odometry

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <stereo_visual_slam_main/visualization.hpp>
#include <stereo_visual_slam_main/optimization.hpp>
#include <stereo_visual_slam_main/map.hpp>

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
    Map &my_map_;

    std::string dataset_;
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_crosscheck_;

    int num_inliers_ = 0; // number of inliers after RANSAC

    SE3 T_c_l_ = SE3(); // T_current(camera)_last(camera)
    SE3 T_c_w_ = SE3(); // T_current(camera)_world

    int seq_ = 1; // sequence number

    // visualization module
    VslamVisual my_visual_;

    TrackState state_ = Init; // current tracking state
    int num_lost_ = 0;        // number of continuous lost frames

    int curr_keyframe_id_ = 0;
    int curr_landmark_id_ = 0;

    bool if_rviz_;

public:
    VO(ros::NodeHandle &nh, Map &map);

    /*! \brief initialize VO
    *
    *  \param dataset - the address of the dataset
    *  \param nh - the node handle
    *  \param map - the map owned by the ros node
    */
    VO(std::string dataset, ros::NodeHandle &nh, Map &map);

    /*! \brief read left and right images into a frame
    *
    *  \param id - id of the frame, starts from zero
    *  \param left_img - the postion left image is write to
    *  \param right_img - the postion right image is write to
    *  \return if the reading is successful
    */
    int read_img(int id, cv::Mat &left_img, cv::Mat &right_img);

    /*! \brief calculate the disparity map from two left-right images
    *
    *  \param frame - a frame includes both L/R images
    *  \param disparity - the postion the disparity map is write to
    *  \return if the calculation is successful
    */
    int disparity_map(const Frame &frame, cv::Mat &disparity);

    /*! \brief initialization pipline
    * 
    *  \return if successful
    */
    bool initialization();

    /*! \brief tracking pipline
    * 
    *  \param if_insert_keyframe - return whether this frame is added as keyframe
    *  \return whether the motion estimation is rejected or not
    */
    bool tracking(bool &if_insert_keyframe);

    /*! \brief feature detection
    *
    *  \param img - the image to detect features
    *  \param keypoints - the keypoints detected
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
    int feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2,
                         std::vector<cv::DMatch> &feature_matches);

    /*! \brief set reference 3d positions for the last frame
    *
    * filter the descriptors in the reference frame, calculate the 3d positions of those features
    * 
    *  \param pts_3d - output of 3d points
    *  \param keypoints - filtered keypoints
    *  \param descriptors - filted descriptors
    *  \param frame - the current frame
    *  \return - a list of reliable depth
    */
    std::vector<bool> set_ref_3d_position(std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints,
                                          cv::Mat &descriptors, Frame &frame);

    /*! \brief estimate the motion using PnP
    *
    *  \param frame - the current frame to etimate motion
    */
    void motion_estimation(Frame &frame);

    /*! \brief check if the estimated motion is valid
    * 
    *  \return if valid
    */
    bool check_motion_estimation();

    /*! \brief move everything from current frame to last frame
    */
    void move_frame();

    /*! \brief write pose to csv file
    * 
    * three cols are x, y and z respectively
    * 
    * \param frame - the frame to write pose
    */
    void write_pose(const Frame &frame);

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

    /*! \brief pipeline of the tracking thread
    *
    *  \param if_insert_keyframe - return whether this frame is added as keyframe
    *  \return return false if VO is lost
    */
    bool pipeline(bool &if_insert_keyframe);

    /*! \brief insert current frame as the keyframe
    *
    *  \param check - the frame is rejected or not
    *  \param pts_3d - output of 3d points
    *  \param keypoints - filtered keypoints
    *  \param descriptors - filted descriptors
    */
    bool insert_key_frame(bool check, std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptors);
};

} // namespace vslam

#endif