/// \file
/// \brief Library for stereo visual odometry

#include <stereo_visual_slam_main/visual_odometry.hpp>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stereo_visual_slam_main/visualization.hpp>
#include <stereo_visual_slam_main/optimization.hpp>
#include <stereo_visual_slam_main/map.hpp>

namespace vslam
{

VO::VO(ros::NodeHandle &nh, Map &map) : my_visual_(nh), my_map_(map)
{
    detector_ = cv::ORB::create(3000);
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

VO::VO(std::string dataset, ros::NodeHandle &nh, Map &map) : my_visual_(nh), my_map_(map)
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

    // std::cout << left_address << " " << right_address << std::endl;

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
    // cv::Mat outimg1;
    // cv::drawKeypoints(img, keypoints, outimg1);
    // cv::imshow("ORB features", outimg1);
    // cv::waitKey(1);

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

bool VO::initialization()
{
    // sequence starts from 1
    frame_last_ = Frame();

    read_img(0, frame_last_.left_img_, frame_last_.right_img_);
    frame_last_.frame_id_ = 0; // write the sequence number to the frame

    // detect features in the left image
    std::vector<cv::KeyPoint> keypoints_detected;
    cv::Mat descriptors_detected;
    feature_detection(frame_last_.left_img_, keypoints_detected, descriptors_detected);

    // put the features into the frame with feature_id, frame_id, keypoint, descriptor
    for (int i = 0; i < keypoints_detected.size(); ++i)
    {
        Feature feature_to_add(i, 0, keypoints_detected.at(i), descriptors_detected.row(i));
        frame_last_.features_.push_back(feature_to_add);
    }

    // debug printing
    std::cout << "Num of features in keyframe: " << frame_last_.features_.size() << std::endl;
    std::cout << "Feature 0 - id: " << frame_last_.features_.at(0).feature_id_ << std::endl;
    std::cout << "Feature 0 - frame: " << frame_last_.features_.at(0).frame_id_ << std::endl;
    std::cout << "Feature 0 - x position: " << frame_last_.features_.at(0).keypoint_.pt.x << std::endl;
    std::cout << "Feature 0 - descriptor: " << frame_last_.features_.at(0).descriptor_.size() << std::endl;


    // disparity_map(frame_last_, frame_last_.disparity_);

    // set_ref_3d_position();

    // // fill the extra information
    // frame_last_.fill_frame(SE3(), true, curr_keyframe_id_);
    // curr_keyframe_id_++;

    // // insert the keyframe
    // my_map_.insert_keyframe(frame_last_);

    // for (size_t i = 0; i < keypoints_last_.size(); i++)
    // {
    //     // push all the features onto the frame
    //     Feature feature_to_add(i, keypoints_last_.at(i), descriptors_last_.row(i));
    //     my_map_.keyframes_[frame_last_.keyframe_id_].add_feature(feature_to_add);
    //     // create and insert the landmarks
    //     Landmark landmark_to_add(curr_landmark_id_, pts_3d_last_.at(i), descriptors_last_.row(i));
    //     curr_landmark_id_++;
    //     my_map_.insert_landmark(landmark_to_add);
    //     // build the connection from feature to frame
    //     my_map_.keyframes_[frame_last_.keyframe_id_].features_.at(i).frame_ = &my_map_.keyframes_[frame_last_.keyframe_id_];
    //     // build the connection from feature to landmark
    //     my_map_.keyframes_[frame_last_.keyframe_id_].features_.at(i).landmark_ = &my_map_.landmarks_[landmark_to_add.id_];
    //     my_map_.landmarks_[landmark_to_add.id_].observed_times_++;
    //     // build the connection from landmark to feature
    //     my_map_.landmarks_[landmark_to_add.id_].observations_.push_back(&my_map_.keyframes_[frame_last_.keyframe_id_].features_.at(i));
    // }

    return true;
}

} // namespace vslam
