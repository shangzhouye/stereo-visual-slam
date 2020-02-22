/// \file
/// \brief Library for stereo visual odometry

#include <stereo_visual_slam_main/visual_odometry.hpp>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include "ros/ros.h"
#include <stereo_visual_slam_main/visualization.hpp>
#include <stereo_visual_slam_main/optimization.hpp>

namespace vslam
{

VO::VO(ros::NodeHandle &nh) : my_visual_(nh)
{
    detector_ = cv::ORB::create(3000);
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

VO::VO(std::string dataset, ros::NodeHandle &nh) : my_visual_(nh)
{
    dataset_ = dataset;
    detector_ = cv::ORB::create(3000);
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

int VO::read_img(int id, cv::Mat &left_img, cv::Mat &right_img)
{
    // auto create address string
    std::string left_address, right_address, image_name;

    image_name = std::to_string(id);
    image_name = std::string(6 - image_name.length(), '0') + image_name;

    left_address = this->dataset_ + "image_0/" + image_name + ".png";
    right_address = this->dataset_ + "image_1/" + image_name + ".png";

    // cout << left_address << " " << right_address << endl;

    left_img = cv::imread(left_address, CV_LOAD_IMAGE_GRAYSCALE);
    right_img = cv::imread(right_address, CV_LOAD_IMAGE_GRAYSCALE);

    if (!left_img.data)
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // cv::namedWindow("Display window Left", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display window Left", left_img);

    // cv::namedWindow("Display window Right", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display window Right", right_img);

    // cv::waitKey(0);

    return 0;
}

int VO::disparity_map(const Frame &frame, cv::Mat &disparity)
{
    // parameters tested for the kitti dataset
    // needs modification if use other dataset
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

    cv::Mat disparity_sgbm;
    sgbm->compute(frame.left_img_, frame.right_img_, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f); // confirm 1/16 here later

    // cv::imshow("disparity", disparity / 96.0);
    // cv::waitKey(1);

    return 0;
}

bool VO::initialization()
{
    // sequence starts from 1
    read_img(seq_ - 1, frame_last_.left_img_, frame_last_.right_img_);
    frame_last_.id_ = seq_ - 1; // write the sequence number to the frame

    read_img(seq_, frame_current_.left_img_, frame_current_.right_img_);
    frame_current_.id_ = seq_;

    disparity_map(frame_last_, frame_last_.disparity_);

    feature_detection(frame_last_.left_img_, keypoints_last_, descriptors_last_);

    set_ref_3d_position();

    feature_detection(frame_current_.left_img_, keypoints_curr_, descriptors_curr_);
    feature_matching(descriptors_last_, descriptors_curr_, feature_matches_);

    // feature_visualize();

    motion_estimation();

    bool check = check_motion_estimation();
    if (check)
    {
        T_c_w_ = T_c_l_ * T_c_w_;
        // write_pose();
        rviz_visualize();
        move_frame();
    }
    seq_++;

    return check;
}

bool VO::tracking()
{
    read_img(seq_, frame_current_.left_img_, frame_current_.right_img_);
    frame_current_.id_ = seq_;

    disparity_map(frame_last_, frame_last_.disparity_);

    set_ref_3d_position();

    feature_detection(frame_current_.left_img_, keypoints_curr_, descriptors_curr_);
    feature_matching(descriptors_last_, descriptors_curr_, feature_matches_);

    // feature_visualize();

    motion_estimation();

    bool check = check_motion_estimation();
    if (check)
    {
        T_c_w_ = T_c_l_ * T_c_w_;
        // write_pose();
        rviz_visualize();
        move_frame();
    }
    seq_++;

    // cout << "World origin in camera frame: " << T_c_w_.translation() << endl;

    return check;
}

int VO::feature_detection(const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    // ensure the image is read
    if (!img.data)
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // feature detection (Oriented FAST)
    detector_->detect(img, keypoints);

    adaptive_non_maximal_suppresion(keypoints, 500);

    // BRIEF describer
    descriptor_->compute(img, keypoints, descriptors);

    // show output image
    // Mat outimg1;
    // cv::drawKeypoints(img, keypoints, outimg1);
    // cv::imshow("ORB features", outimg1);
    // cv::waitKey(0);

    return 0;
}

void VO::adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints,
                                                      const int num)
{
    // if number of keypoints is already lower than the threshold, return
    if (keypoints.size() < num)
    {
        return;
    }

    // sort the keypoints according to its reponse (strength)
    std::sort(keypoints.begin(), keypoints.end(), [&](const cv::KeyPoint &lhs, const cv::KeyPoint &rhs) {
        return lhs.response > rhs.response;
    });

    // vector for store ANMS points
    std::vector<cv::KeyPoint> ANMSpt;

    std::vector<double> rad_i;
    rad_i.resize(keypoints.size());

    std::vector<double> rad_i_sorted;
    rad_i_sorted.resize(keypoints.size());

    // robust coefficient: 1/0.9 = 1.1
    const float c_robust = 1.11;

    // computing the suppression radius for each feature (strongest overall has radius of infinity)
    // the smallest distance to another point that is significantly stronger (based on a robustness parameter)
    for (int i = 0; i < keypoints.size(); ++i)
    {
        const float response = keypoints[i].response * c_robust;

        // maximum finit number of double
        double radius = std::numeric_limits<double>::max();

        for (int j = 0; j < i && keypoints[j].response > response; ++j)
        {
            radius = std::min(radius, cv::norm(keypoints[i].pt - keypoints[j].pt));
        }

        rad_i[i] = radius;
        rad_i_sorted[i] = radius;
    }

    // sort it
    std::sort(rad_i_sorted.begin(), rad_i_sorted.end(), [&](const double &lhs, const double &rhs) {
        return lhs > rhs;
    });

    // find the final radius
    const double final_radius = rad_i_sorted[num];
    for (int i = 0; i < rad_i.size(); ++i)
    {
        if (rad_i[i] >= final_radius)
        {
            ANMSpt.push_back(keypoints[i]);
        }
    }

    // swap address to keypoints, O(1) time
    keypoints.swap(ANMSpt);
}

int VO::feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, std::vector<cv::DMatch> &feature_matches)
{
    feature_matches_.clear();

    std::vector<cv::DMatch> matches_crosscheck;
    // use cross check for matching
    matcher_crosscheck_->match(descriptors_1, descriptors_2, matches_crosscheck);
    // cout << "Number of matches after cross check: " << matches_crosscheck.size() << endl;

    // calculate the min/max distance
    auto min_max = minmax_element(matches_crosscheck.begin(), matches_crosscheck.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.distance < rhs.distance;
    });

    auto min_element = min_max.first;
    auto max_element = min_max.second;
    // cout << "Min distance: " << min_element->distance << endl;
    // cout << "Max distance: " << max_element->distance << endl;

    // threshold: distance should be smaller than two times of min distance or a give threshold
    for (int i = 0; i < matches_crosscheck.size(); i++)
    {
        if (matches_crosscheck[i].distance <= std::max(2.0 * min_element->distance, 30.0))
        {
            feature_matches.push_back(matches_crosscheck[i]);
        }
    }

    // cout << "Number of matches after threshold: " << feature_matches.size() << endl;

    return 0;
}

void VO::feature_visualize()
{
    // show matched images
    cv::Mat img;
    cv::drawMatches(frame_last_.left_img_, keypoints_last_, frame_current_.left_img_, keypoints_curr_,
                    feature_matches_, img);
    cv::imshow("feature matches", img);
    cv::waitKey(1);
}

int VO::set_ref_3d_position()
{
    // clear existing 3D positions
    pts_3d_last_.clear();

    // create filetered descriptors for replacement
    cv::Mat descriptors_last_filtered;
    std::vector<cv::KeyPoint> keypoints_last_filtered;

    for (size_t i = 0; i < keypoints_last_.size(); i++)
    {
        Eigen::Vector3d pos_3d = frame_last_.find_3d(keypoints_last_.at(i));
        // filer out points with no depth information (disparity value = -1)
        if (pos_3d(2) > 10 && pos_3d(2) < 400)
        {
            pts_3d_last_.push_back(cv::Point3f(pos_3d(0), pos_3d(1), pos_3d(2)));
            descriptors_last_filtered.push_back(descriptors_last_.row(i));
            keypoints_last_filtered.push_back(keypoints_last_.at(i));
        }
    }

    // copy the filtered descriptors;
    descriptors_last_ = descriptors_last_filtered;
    keypoints_last_ = keypoints_last_filtered;
}

void VO::motion_estimation()
{
    // 3D positions from the last frame
    // 2D pixels in current frame
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;

    for (cv::DMatch m : feature_matches_)
    {
        pts3d.push_back(pts_3d_last_[m.queryIdx]);
        pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << frame_last_.fx_, 0, frame_last_.cx_,
             0, frame_last_.fy_, frame_last_.cy_,
             0, 0, 1);

    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

    num_inliers_ = inliers.rows;
    // cout << "Number of PnP inliers: " << num_inliers_ << endl;

    // transfer rvec to matrix
    cv::Mat SO3_R_cv;
    cv::Rodrigues(rvec, SO3_R_cv);
    Eigen::Matrix3d SO3_R;
    SO3_R << SO3_R_cv.at<double>(0, 0), SO3_R_cv.at<double>(0, 1), SO3_R_cv.at<double>(0, 2),
        SO3_R_cv.at<double>(1, 0), SO3_R_cv.at<double>(1, 1), SO3_R_cv.at<double>(1, 2),
        SO3_R_cv.at<double>(2, 0), SO3_R_cv.at<double>(2, 1), SO3_R_cv.at<double>(2, 2);

    T_c_l_ = SE3(
        SO3_R,
        Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

    // cout << "T_c_l Translation x: " << tvec.at<double>(0, 0) << "; y: " << tvec.at<double>(1, 0) << "; z: " << tvec.at<double>(2, 0) << endl;

    // convert cv point to eigen vector
    // only include inliers
    G2OVector2d g2o_pts2d;
    g2o_pts2d.resize(inliers.rows);
    G2OVector3d g2o_pts3d;
    g2o_pts3d.resize(inliers.rows);

    for (int idx = 0; idx < inliers.rows; idx++)
    {
        int index = inliers.at<int>(idx,0);
        g2o_pts2d[idx] << pts2d[index].x, pts2d[index].y;
        g2o_pts3d[idx] << pts3d[index].x, pts3d[index].y, pts3d[index].z;
    }

    // add nonlinear optimization
    single_frame_optimization(g2o_pts3d, g2o_pts2d, K, T_c_l_);
}

bool VO::check_motion_estimation()
{
    // check the number of inliers
    if (num_inliers_ < 10)
    {
        std::cout << "Frame id: " << frame_last_.id_ << " and " << frame_current_.id_ << std::endl;
        std::cout << "Rejected - inliers not enough: " << num_inliers_ << std::endl;
        return false;
    }

    // check if the motion is too large
    Sophus::Vector6d displacement = T_c_l_.log();
    double frame_gap = frame_current_.id_ - frame_last_.id_; // get the idx gap between last and current frame
    if (displacement.norm() > (5.0 * frame_gap))
    {
        std::cout << "Frame id: " << frame_last_.id_ << " and " << frame_current_.id_ << std::endl;
        std::cout << "Rejected - motion is too large: " << displacement.norm() << std::endl;
        return false;
    }

    // check if the motion is forward
    Eigen::Vector3d translation = T_c_l_.translation();
    if (translation(2) > 0.5)
    {
        std::cout << "Frame id: " << frame_last_.id_ << " and " << frame_current_.id_ << std::endl;
        std::cout << "Rejected - motion is backward: " << T_c_l_.transZ << std::endl;
        return false;
    }

    return true;
}

void VO::VOpipeline(int ite_num)
{

    for (size_t ite = 0; ite < ite_num; ite++)
    {
        // ros::Time time_start = ros::Time::now();

        switch (state_)
        {
        case Init:
        {

            if (initialization())
            {
                state_ = Track;
            }
            else
            {
                num_lost_++;
                if (num_lost_ > 10)
                {
                    state_ = Lost;
                }
            }
            break;
        }
        case Track:
        {
            if (tracking())
            {
                num_lost_ = 0;
            }
            else
            {
                num_lost_++;
                if (num_lost_ > 10)
                {
                    state_ = Lost;
                }
            }
            break;
        }
        case Lost:
        {
            std::cout << "VO IS LOST" << std::endl;
            return;
            break;
        }
        default:
            std::cout << "Invalid state" << std::endl;
            return;
            break;
        }

        // ros::Time time_end = ros::Time::now();
        // ROS_INFO("Time for this loop is: %f", (time_end - time_start).toSec());
        ros::spinOnce();
    }
}

void VO::move_frame()
{
    frame_last_ = frame_current_;
    keypoints_last_ = keypoints_curr_;
    descriptors_last_ = descriptors_curr_;
}

void VO::write_pose()
{
    SE3 T_w_c;
    T_w_c = T_c_w_.inverse();
    double r00, r01, r02, r10, r11, r12, r20, r21, r22, x, y, z;
    Eigen::Matrix3d rotation = T_w_c.rotationMatrix();
    Eigen::Vector3d translation = T_w_c.translation();
    r00 = rotation(0, 0);
    r01 = rotation(0, 1);
    r02 = rotation(0, 2);
    r10 = rotation(1, 0);
    r11 = rotation(1, 1);
    r12 = rotation(1, 2);
    r20 = rotation(2, 0);
    r21 = rotation(2, 1);
    r22 = rotation(2, 2);
    x = translation(0);
    y = translation(1);
    z = translation(2);

    std::ofstream file;
    file.open("estimated_traj.txt", std::ios_base::app);

    // alows dropping frame
    file << frame_current_.id_ << " " << r00 << " " << r01 << " " << r02 << " " << x << " "
         << r10 << " " << r11 << " " << r12 << " " << y << " "
         << r20 << " " << r21 << " " << r22 << " " << z << std::endl;
    file.close();
}

void VO::rviz_visualize()
{
    // currently using opencv to show the image
    cv::Mat outimg;
    cv::drawKeypoints(frame_last_.left_img_, keypoints_last_, outimg);
    cv::imshow("ORB features", outimg);
    cv::waitKey(1);

    // publish feature map
    my_visual_.publish_feature_map(pts_3d_last_);

    // publish transform
    my_visual_.publish_transform(T_c_w_);
    ros::spinOnce();
}

void VO::single_frame_optimization(const G2OVector3d &points_3d, const G2OVector2d &points_2d,
                                                const cv::Mat &K, Sophus::SE3d &pose)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Use GaussNewton Method
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // vertex
    VertexPose *vertex_pose = new VertexPose();
    // there is only one vertex: vertex 0
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    // K
    Eigen::Matrix3d K_eigen;
    K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // edges
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i)
    {
        auto p2d = points_2d[i];
        auto p3d = points_3d[i];
        EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
        edge->setId(index);
        // put the edges onto vertex zero
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    // optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // cout << "Pose optimized = " << vertex_pose->estimate().matrix() << endl;

    pose = vertex_pose->estimate();
}

} // namespace vslam
