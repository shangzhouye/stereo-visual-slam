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

StructurelessVO::StructurelessVO() {}

StructurelessVO::StructurelessVO(string dataset)
{
    this->dataset_ = dataset;
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
    this->read_img(0, frame_last_.left_img_, frame_last_.right_img_);
    this->read_img(1, frame_current_.left_img_, frame_current_.right_img_);

    this->disparity_map(frame_last_, frame_last_.disparity_);
    this->disparity_map(frame_current_, frame_current_.disparity_);

    this->feature_matching(frame_last_.left_img_, frame_current_.left_img_);

    return 0;
}

int StructurelessVO::feature_matching(const cv::Mat &img_1, const cv::Mat &img_2)
{
    // ensure the image is read
    if (!img_1.data)
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // feature detection (Oriented FAST)
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector_->detect(img_1, keypoints_1);
    detector_->detect(img_2, keypoints_2);

    // BRIEF describer
    Mat descriptors_1, descriptors_2;
    descriptor_->compute(img_1, keypoints_1, descriptors_1);
    descriptor_->compute(img_2, keypoints_2, descriptors_2);

    // show output image
    Mat outimg1;
    cv::drawKeypoints(img_1, keypoints_1, outimg1);
    cv::imshow("ORB features", outimg1);

    vector<cv::DMatch> matches;
    matcher_->match(descriptors_1, descriptors_2, matches);

    // show matched images
    Mat img_match;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    cv::imshow("all matches", img_match);
    cout << "Number of all the matches: " << matches.size() << endl;

    // calculate the min/max distance
    auto min_max = minmax_element(matches.begin(), matches.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.distance < rhs.distance;
    });

    auto min_element = min_max.first;
    auto max_element = min_max.second;
    cout << "Min distance: " << min_element->distance << endl;
    cout << "Max distance: " << max_element->distance << endl;

    // threshold: distance should be smaller than two times of min distance or 30
    vector<cv::DMatch> matches_threshold;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2.0 * min_element->distance, 15.0))
        {
            matches_threshold.push_back(matches[i]);
        }
    }

    // show matched images with threshold
    Mat img_match_threshold;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches_threshold, img_match_threshold);
    cv::imshow("matches after threshold", img_match_threshold);
    cout << "Number of matches after threshold: " << matches_threshold.size() << endl;
    cv::waitKey(0);

    return 0;
}

} // namespace vslam