/// \file
/// \brief Library for structureless stereo visual odometry

#include <stereo_visual_slam_main/structureless_vo.hpp>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>

using namespace std;
using namespace Eigen;

namespace vslam
{

StructurelessVO::StructurelessVO()
{
    detector_ = cv::ORB::create();
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

StructurelessVO::StructurelessVO(string dataset)
{
    dataset_ = dataset;
    detector_ = cv::ORB::create();
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

int StructurelessVO::read_img(int id, cv::Mat &left_img, cv::Mat &right_img)
{
    // auto create address string
    string left_address, right_address, image_name;

    image_name = to_string(id);
    image_name = string(6 - image_name.length(), '0') + image_name;

    left_address = this->dataset_ + "image_0/" + image_name + ".png";
    right_address = this->dataset_ + "image_1/" + image_name + ".png";

    // cout << left_address << " " << right_address << endl;

    left_img = cv::imread(left_address, CV_LOAD_IMAGE_GRAYSCALE);
    right_img = cv::imread(right_address, CV_LOAD_IMAGE_GRAYSCALE);

    if (!left_img.data)
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // cv::namedWindow("Display window Left", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display window Left", left_img);

    // cv::namedWindow("Display window Right", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display window Right", right_img);

    // cv::waitKey(0);

    return 0;
}

int StructurelessVO::disparity_map(const Frame &frame, cv::Mat &disparity)
{
    // parameters tested for the kitti dataset
    // needs modification if use other dataset
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

    cv::Mat disparity_sgbm;
    sgbm->compute(frame.left_img_, frame.right_img_, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f); // confirm 1/16 here later

    // cv::imshow("disparity", disparity / 96.0);
    // cv::waitKey(0);

    return 0;
}

bool StructurelessVO::initialization()
{
    // sequence starts from 1
    read_img(seq_ - 1, frame_last_.left_img_, frame_last_.right_img_);
    read_img(seq_, frame_current_.left_img_, frame_current_.right_img_);

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
        move_frame();
        T_c_w_ = T_c_l_ * T_c_w_;
        write_pose();
    }
    seq_++;

    return check;
}

bool StructurelessVO::tracking()
{
    read_img(seq_, frame_current_.left_img_, frame_current_.right_img_);

    disparity_map(frame_last_, frame_last_.disparity_);

    set_ref_3d_position();

    feature_detection(frame_current_.left_img_, keypoints_curr_, descriptors_curr_);
    feature_matching(descriptors_last_, descriptors_curr_, feature_matches_);

    // feature_visualize();

    motion_estimation();

    bool check = check_motion_estimation();
    if (check)
    {
        move_frame();
        T_c_w_ = T_c_l_ * T_c_w_;
        write_pose();
    }
    seq_++;

    cout << "World origin in camera frame: " << T_c_w_.translation() << endl;

    return check;
}

int StructurelessVO::feature_detection(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    // ensure the image is read
    if (!img.data)
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // feature detection (Oriented FAST)
    detector_->detect(img, keypoints);

    // BRIEF describer
    descriptor_->compute(img, keypoints, descriptors);

    // show output image
    // Mat outimg1;
    // cv::drawKeypoints(img, keypoints, outimg1);
    // cv::imshow("ORB features", outimg1);
    // cv::waitKey(0);

    return 0;
}

int StructurelessVO::feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, vector<cv::DMatch> &feature_matches)
{
    feature_matches_.clear();
    
    vector<cv::DMatch> matches_crosscheck;
    // use cross check for matching
    matcher_crosscheck_->match(descriptors_1, descriptors_2, matches_crosscheck);
    cout << "Number of matches after cross check: " << matches_crosscheck.size() << endl;

    // calculate the min/max distance
    auto min_max = minmax_element(matches_crosscheck.begin(), matches_crosscheck.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.distance < rhs.distance;
    });

    auto min_element = min_max.first;
    auto max_element = min_max.second;
    cout << "Min distance: " << min_element->distance << endl;
    cout << "Max distance: " << max_element->distance << endl;

    // threshold: distance should be smaller than two times of min distance or a give threshold
    for (int i = 0; i < matches_crosscheck.size(); i++)
    {
        if (matches_crosscheck[i].distance <= max(2.0 * min_element->distance, 20.0))
        {
            feature_matches.push_back(matches_crosscheck[i]);
        }
    }

    cout << "Number of matches after threshold: " << feature_matches.size() << endl;

    return 0;
}

void StructurelessVO::feature_visualize()
{
    // show matched images
    Mat img;
    cv::drawMatches(frame_last_.left_img_, keypoints_last_, frame_current_.left_img_, keypoints_curr_,
                    feature_matches_, img);
    cv::imshow("feature matches", img);
    cv::waitKey(0);
}

int StructurelessVO::set_ref_3d_position()
{
    // clear existing 3D positions
    pts_3d_last_.clear();

    // create filetered descriptors for replacement
    cv::Mat descriptors_last_filtered;
    vector<cv::KeyPoint> keypoints_last_filtered;

    for (size_t i = 0; i < keypoints_last_.size(); i++)
    {
        Vector3d pos_3d = frame_last_.find_3d(keypoints_last_.at(i));
        // filer out points with no depth information (disparity value = -1)
        if (pos_3d(2) > 0)
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

void StructurelessVO::motion_estimation()
{
    // 3D positions from the last frame
    // 2D pixels in current frame
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (cv::DMatch m : feature_matches_)
    {
        pts3d.push_back(pts_3d_last_[m.queryIdx]);
        pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
    }

    Mat K = (cv::Mat_<double>(3, 3) << frame_last_.fx_, 0, frame_last_.cx_,
             0, frame_last_.fy_, frame_last_.cy_,
             0, 0, 1);

    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

    num_inliers_ = inliers.rows;
    cout << "Number of PnP inliers: " << num_inliers_ << endl;

    // transfer rvec to matrix
    Mat SO3_R_cv;
    cv::Rodrigues(rvec, SO3_R_cv);
    Matrix3d SO3_R;
    SO3_R << SO3_R_cv.at<double>(0, 0), SO3_R_cv.at<double>(0, 1), SO3_R_cv.at<double>(0, 2),
        SO3_R_cv.at<double>(1, 0), SO3_R_cv.at<double>(1, 1), SO3_R_cv.at<double>(1, 2),
        SO3_R_cv.at<double>(2, 0), SO3_R_cv.at<double>(2, 1), SO3_R_cv.at<double>(2, 2);

    T_c_l_ = SE3(
        SO3_R,
        Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

    cout << "T_c_l Translation x: " << tvec.at<double>(0, 0) << "; y: " << tvec.at<double>(1, 0) << "; z: " << tvec.at<double>(2, 0) << endl;
}

bool StructurelessVO::check_motion_estimation()
{
    // check the number of inliers
    if (num_inliers_ < 10)
    {
        cout << "Rejected - inliers not enough: " << num_inliers_ << endl;
        return false;
    }

    // check if the motion is too large
    Sophus::Vector6d displacement = T_c_l_.log();
    if (displacement.norm() > 5.0)
    {
        cout << "Rejected - motion is too large: " << displacement.norm() << endl;
        return false;
    }

    // check if the motion is forward
    Vector3d translation = T_c_l_.translation();
    if (translation(2) > 0.5)
    {
        cout << "Rejected - motion is backward: " << T_c_l_.transZ << endl;
        return false;
    }

    return true;
}

void StructurelessVO::VOpipeline(int ite_num)
{

    for (size_t ite = 0; ite < ite_num; ite++)
    {

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
            cout << "VO IS LOST" << endl;
            break;
        }
        default:
            cout << "Invalid state" << endl;
            break;
        }
    }
}

void StructurelessVO::move_frame()
{
    frame_last_ = frame_current_;
    keypoints_last_ = keypoints_curr_;
    descriptors_last_ = descriptors_curr_;
}

void StructurelessVO::write_pose()
{
    SE3 T_w_c;
    T_w_c = T_c_w_.inverse();
    double x, y, z;
    x = T_w_c.translation()(0);
    y = T_w_c.translation()(1);
    z = T_w_c.translation()(2);

    ofstream file;
    file.open("estimated_traj.csv", ios_base::app);
    file << x << "," << y << "," << z << endl;
    file.close();
}

} // namespace vslam
