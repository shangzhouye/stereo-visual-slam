/// \file
/// \brief Library for structureless stereo visual odometry

#include <stereo_visual_slam_main/structureless_vo.hpp>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <iostream>
#include <string>

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

    cv::namedWindow("Display window Left", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window Left", left_img);

    cv::namedWindow("Display window Right", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window Right", right_img);

    cv::waitKey(0);

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

    cv::imshow("disparity", disparity / 96.0);
    cv::waitKey(0);

    return 0;
}

int StructurelessVO::initialization()
{
    this->read_img(0, this->frame_last_.left_img_, this->frame_last_.right_img_);
    this->read_img(1, this->frame_current_.left_img_, this->frame_current_.right_img_);

    this->disparity_map(this->frame_last_, this->frame_last_.disparity_);
    this->disparity_map(this->frame_current_, this->frame_current_.disparity_);

    return 0;
}

} // namespace vslam