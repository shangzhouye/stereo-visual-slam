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

int StructurelessVO::initialization()
{
    read_img(0, frame_last_.left_img_, frame_last_.right_img_);
    read_img(1, frame_current_.left_img_, frame_current_.right_img_);

    disparity_map(frame_last_, frame_last_.disparity_);

    feature_detection(frame_last_.left_img_, descriptors_last_);
    feature_detection(frame_current_.left_img_, descriptors_curr_);

    feature_matching(descriptors_last_, descriptors_curr_, feature_matches_);

    return 0;
}

int StructurelessVO::feature_detection(const cv::Mat &img, cv::Mat &descriptors)
{
    // ensure the image is read
    if (!img.data)
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // feature detection (Oriented FAST)
    vector<cv::KeyPoint> keypoints;
    detector_->detect(img, keypoints);

    // BRIEF describer
    descriptor_->compute(img, keypoints, descriptors);

    // show output image
    Mat outimg1;
    cv::drawKeypoints(img, keypoints, outimg1);
    cv::imshow("ORB features", outimg1);

    return 0;
}

int StructurelessVO::feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, vector<cv::DMatch> &feature_matches)
{

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

} // namespace vslam

// // show matched images
// Mat img_match_crosscheck;
// cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches_crosscheck, img_match_crosscheck);
// cv::imwrite("matches_after_cross_check.jpg", img_match_crosscheck);
// cv::imshow("matches after cross check", img_match_crosscheck);