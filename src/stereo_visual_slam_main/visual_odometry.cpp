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
    nh.getParam("/if_rviz", if_rviz_);
}

VO::VO(std::string dataset, ros::NodeHandle &nh, Map &map) : my_visual_(nh), my_map_(map)
{
    dataset_ = dataset;
    detector_ = cv::ORB::create(3000);
    descriptor_ = cv::ORB::create();
    matcher_crosscheck_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    nh.getParam("/if_rviz", if_rviz_);
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
    cv::Mat outimg1;
    cv::drawKeypoints(img, keypoints, outimg1);
    cv::imshow("ORB features", outimg1);
    cv::waitKey(1);

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
        const float response = keypoints.at(i).response * c_robust;

        // maximum finit number of double
        double radius = std::numeric_limits<double>::max();

        for (int j = 0; j < i && keypoints.at(j).response > response; ++j)
        {
            radius = std::min(radius, cv::norm(keypoints.at(i).pt - keypoints.at(j).pt));
        }

        rad_i.at(i) = radius;
        rad_i_sorted.at(i) = radius;
    }

    // sort it
    std::sort(rad_i_sorted.begin(), rad_i_sorted.end(), [&](const double &lhs, const double &rhs) {
        return lhs > rhs;
    });

    // find the final radius
    const double final_radius = rad_i_sorted.at(num - 1);
    for (int i = 0; i < rad_i.size(); ++i)
    {
        if (rad_i.at(i) >= final_radius)
        {
            ANMSpt.push_back(keypoints.at(i));
        }
    }

    // swap address to keypoints, O(1) time
    keypoints.swap(ANMSpt);
}

int VO::disparity_map(const Frame &frame, cv::Mat &disparity)
{
    // parameters tested for the kitti dataset
    // needs modification if use other dataset
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);

    cv::Mat disparity_sgbm;
    sgbm->compute(frame.left_img_, frame.right_img_, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // cv::imshow("disparity", disparity / 96.0);
    // cv::waitKey(1);

    return 0;
}

std::vector<bool> VO::set_ref_3d_position(std::vector<cv::Point3f> &pts_3d,
                                          std::vector<cv::KeyPoint> &keypoints,
                                          cv::Mat &descriptors,
                                          Frame &frame)
{
    // clear existing 3D positions
    pts_3d.clear();

    // create filetered descriptors for replacement
    cv::Mat descriptors_last_filtered;
    std::vector<cv::KeyPoint> keypoints_last_filtered;
    std::vector<bool> reliable_depth;

    for (size_t i = 0; i < keypoints.size(); i++)
    {
        Eigen::Vector3d relative_pos_3d;
        Eigen::Vector3d pos_3d = frame.find_3d(keypoints.at(i), relative_pos_3d);
        // filer out points with no depth information (disparity value = -1)
        if (relative_pos_3d(2) > 10 && relative_pos_3d(2) < 400)
        {
            pts_3d.push_back(cv::Point3f(pos_3d(0), pos_3d(1), pos_3d(2)));
            descriptors_last_filtered.push_back(descriptors.row(i));
            keypoints_last_filtered.push_back(keypoints.at(i));

            // mark reliable depth information
            if (relative_pos_3d(2) < 40)
            {
                reliable_depth.push_back(true);
            }
            else
            {
                reliable_depth.push_back(false);
            }
        }
    }

    // copy the filtered descriptors;
    descriptors = descriptors_last_filtered;
    keypoints = keypoints_last_filtered;

    return reliable_depth;
}

int VO::feature_matching(const cv::Mat &descriptors_1, const cv::Mat &descriptors_2, std::vector<cv::DMatch> &feature_matches)
{
    feature_matches.clear();

    std::vector<cv::DMatch> matches_crosscheck;
    // use cross check for matching
    matcher_crosscheck_->match(descriptors_1, descriptors_2, matches_crosscheck);
    // std::cout << "Number of matches after cross check: " << matches_crosscheck.size() << std::endl;

    // calculate the min/max distance
    auto min_max = minmax_element(matches_crosscheck.begin(), matches_crosscheck.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.distance < rhs.distance;
    });

    auto min_element = min_max.first;
    auto max_element = min_max.second;
    // std::cout << "Min distance: " << min_element->distance << std::endl;
    // std::cout << "Max distance: " << max_element->distance << std::endl;

    // threshold: distance should be smaller than two times of min distance or a give threshold
    double frame_gap = frame_current_.frame_id_ - frame_last_.frame_id_;
    for (int i = 0; i < matches_crosscheck.size(); i++)
    {
        if (matches_crosscheck.at(i).distance <= std::max(2.0 * min_element->distance, 30.0 * frame_gap))
        {
            feature_matches.push_back(matches_crosscheck.at(i));
        }
    }

    // std::cout << "Number of matches after threshold: " << feature_matches.size() << std::endl;

    return 0;
}

void VO::motion_estimation(Frame &frame)
{
    // 3D positions from the last frame
    // 2D pixels in current frame
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;

    for (int i = 0; i < frame.features_.size(); i++)
    {
        int landmark_id = frame.features_.at(i).landmark_id_;
        if (landmark_id == -1)
        {
            std::cout << "No landmark associated!" << std::endl;
        }

        pts3d.push_back(my_map_.landmarks_.at(landmark_id).pt_3d_);
        pts2d.push_back(frame.features_.at(i).keypoint_.pt);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << frame.fx_, 0, frame.cx_,
                 0, frame.fy_, frame.cy_,
                 0, 0, 1);

    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

    num_inliers_ = inliers.rows;
    // std::cout << "Number of PnP inliers: " << num_inliers_ << std::endl;

    // transfer rvec to matrix
    cv::Mat SO3_R_cv;
    cv::Rodrigues(rvec, SO3_R_cv);
    Eigen::Matrix3d SO3_R;
    SO3_R << SO3_R_cv.at<double>(0, 0), SO3_R_cv.at<double>(0, 1), SO3_R_cv.at<double>(0, 2),
        SO3_R_cv.at<double>(1, 0), SO3_R_cv.at<double>(1, 1), SO3_R_cv.at<double>(1, 2),
        SO3_R_cv.at<double>(2, 0), SO3_R_cv.at<double>(2, 1), SO3_R_cv.at<double>(2, 2);

    T_c_w_ = SE3(
        SO3_R,
        Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

    // mark inlieres
    for (int idx = 0; idx < inliers.rows; idx++)
    {
        int index = inliers.at<int>(idx, 0);
        frame.features_.at(index).is_inlier = true;

        // also mark the landmark
        // currently all landmarks are initialized to true
        // my_map_.landmarks_.at(frame.features_.at(index).landmark_id_).is_inlier = true;
    }

    // delete outliers
    frame.features_.erase(std::remove_if(
                              frame.features_.begin(), frame.features_.end(),
                              [](const Feature &x) {
                                  return x.is_inlier == false;
                              }),
                          frame.features_.end());

    // std::cout << "T_c_l Translation x: " << tvec.at<double>(0, 0) << "; y: " << tvec.at<double>(1, 0) << "; z: " << tvec.at<double>(2, 0) << std::endl;
}

bool VO::check_motion_estimation()
{
    // check the number of inliers
    if (num_inliers_ < 10)
    {
        std::cout << "Frame id: " << frame_last_.frame_id_ << " and " << frame_current_.frame_id_ << std::endl;
        std::cout << "Rejected - inliers not enough: " << num_inliers_ << std::endl;
        return false;
    }

    // check if the motion is too large
    Sophus::Vector6d displacement = T_c_l_.log();
    double frame_gap = frame_current_.frame_id_ - frame_last_.frame_id_; // get the idx gap between last and current frame
    if (displacement.norm() > (5.0 * frame_gap))
    {
        std::cout << "Frame id: " << frame_last_.frame_id_ << " and " << frame_current_.frame_id_ << std::endl;
        std::cout << "Rejected - motion is too large: " << displacement.norm() << std::endl;
        return false;
    }

    // check if the motion is forward
    // Eigen::Vector3d translation = T_c_l_.translation();
    // if (translation(2) > 1)
    // {
    //     std::cout << "Frame id: " << frame_last_.frame_id_ << " and " << frame_current_.frame_id_ << std::endl;
    //     std::cout << "Rejected - motion is backward: " << translation(2) << std::endl;
    //     return false;
    // }

    return true;
}

bool VO::insert_key_frame(bool check, std::vector<cv::Point3f> &pts_3d, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    // if the number of inliers is enough or the frame is rejected
    // parameter tunning
    // added more keyframes when turning
    if ((num_inliers_ >= 80 && T_c_l_.angleY() < 0.03) || check == false)
    {
        return false;
    }

    // fill the extra information
    frame_current_.is_keyframe_ = true;
    frame_current_.keyframe_id_ = curr_keyframe_id_;

    // add observations
    for (int i = 0; i < frame_current_.features_.size(); i++)
    {
        int landmark_id = frame_current_.features_.at(i).landmark_id_;
        my_map_.landmarks_.at(landmark_id).observed_times_++;
        Observation observation(frame_current_.keyframe_id_, frame_current_.features_.at(i).feature_id_);
        my_map_.landmarks_.at(landmark_id).observations_.push_back(observation);

        // std::cout << "Landmark " << landmark_id << " "
        //           << "has obsevation times: " << my_map_.landmarks_.at(landmark_id).observed_times_ << std::endl;
        // std::cout << "Landmark " << landmark_id << " "
        //           << "last observation keyframe: " << my_map_.landmarks_.at(landmark_id).observations_.back().keyframe_id_ << std::endl;
    }

    // add more features with triangulated points to the map
    disparity_map(frame_current_, frame_current_.disparity_);
    std::vector<bool> reliable_depth = set_ref_3d_position(pts_3d, keypoints, descriptors, frame_current_);

    // calculate the world coordinate
    // no relative motion any more

    int feature_id = frame_current_.features_.size();
    // if the feature does not exist in the frame already, add it
    for (int i = 0; i < keypoints.size(); i++)
    {
        bool exist = false;
        for (auto &feat : frame_current_.features_)
        {
            if (feat.keypoint_.pt.x == keypoints.at(i).pt.x && feat.keypoint_.pt.y == keypoints.at(i).pt.y)
            {
                exist = true;

                // try to update the landmark position if already exist
                if ((my_map_.landmarks_.at(feat.landmark_id_).reliable_depth_ == false) && (reliable_depth.at(i) == true))
                {
                    my_map_.landmarks_.at(feat.landmark_id_).pt_3d_ = pts_3d.at(i);
                    my_map_.landmarks_.at(feat.landmark_id_).reliable_depth_ = true;
                }
            }
        }
        if (exist == false)
        {
            // add this feature
            // put the features into the frame with feature_id, frame_id, keypoint, descriptor
            // build the connection from feature to frame
            Feature feature_to_add(feature_id, frame_current_.frame_id_,
                                   keypoints.at(i), descriptors.row(i));

            // build the connection from feature to landmark
            feature_to_add.landmark_id_ = curr_landmark_id_;
            frame_current_.features_.push_back(feature_to_add);
            // create a landmark
            // build the connection from landmark to feature
            Observation observation(frame_current_.keyframe_id_, feature_id);
            Landmark landmark_to_add(curr_landmark_id_, pts_3d.at(i), descriptors.row(i), reliable_depth.at(i), observation);
            curr_landmark_id_++;
            // insert the landmark
            my_map_.insert_landmark(landmark_to_add);
            feature_id++;
        }
    }
    curr_keyframe_id_++;

    // insert the keyframe
    my_map_.insert_keyframe(frame_current_);

    // std::cout << "Set frame: " << frame_current_.frame_id_ << " as keyframe "
    //           << frame_current_.keyframe_id_ << std::endl;

    return true;
}

void VO::move_frame()
{
    frame_last_ = frame_current_;
}

void VO::rviz_visualize()
{
    // currently using opencv to show the image
    // cv::Mat outimg;
    // cv::drawKeypoints(frame_last_.left_img_, keypoints_last_, outimg);
    // cv::imshow("ORB features", outimg);
    // cv::waitKey(1);

    // publish feature map
    std::vector<cv::Point3f> pts_3d;
    pts_3d.clear();
    for (auto &lm : my_map_.landmarks_)
    {
        pts_3d.push_back(lm.second.pt_3d_);
    }
    my_visual_.publish_feature_map(pts_3d);

    // publish transform
    my_visual_.publish_transform(T_c_w_);
    ros::spinOnce();
}

void VO::write_pose(const Frame &frame)
{
    SE3 T_w_c;
    T_w_c = frame.T_c_w_.inverse();
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
    file << frame.frame_id_ << " " << r00 << " " << r01 << " " << r02 << " " << x << " "
         << r10 << " " << r11 << " " << r12 << " " << y << " "
         << r20 << " " << r21 << " " << r22 << " " << z << std::endl;
    file.close();
}

bool VO::initialization()
{
    frame_last_ = Frame();

    read_img(0, frame_last_.left_img_, frame_last_.right_img_);
    frame_last_.frame_id_ = 0; // write the sequence number to the frame

    // detect features in the left image
    std::vector<cv::KeyPoint> keypoints_detected;
    cv::Mat descriptors_detected;
    std::vector<cv::Point3f> pts_3d;

    feature_detection(frame_last_.left_img_, keypoints_detected, descriptors_detected);

    disparity_map(frame_last_, frame_last_.disparity_);

    std::vector<bool> reliable_depth = set_ref_3d_position(pts_3d, keypoints_detected, descriptors_detected, frame_last_);

    for (int i = 0; i < keypoints_detected.size(); i++)
    {
        // put the features into the frame with feature_id, frame_id, keypoint, descriptor
        // build the connection from feature to frame
        Feature feature_to_add(i, 0, keypoints_detected.at(i), descriptors_detected.row(i));
        // build the connection from feature to landmark
        feature_to_add.landmark_id_ = curr_landmark_id_;
        frame_last_.features_.push_back(feature_to_add);
        // create a landmark
        // build the connection from landmark to feature
        // this 0 is also the keyframe id
        Observation observation(0, i);
        Landmark landmark_to_add(curr_landmark_id_, pts_3d.at(i), descriptors_detected.row(i), reliable_depth.at(i), observation);
        curr_landmark_id_++;
        // insert the landmark
        my_map_.insert_landmark(landmark_to_add);
    }

    // debug printing
    // std::cout << "  Num of features in keyframe: " << frame_last_.features_.size() << std::endl;
    // std::cout << "  Feature 0 - id: " << frame_last_.features_.at(0).feature_id_ << std::endl;
    // std::cout << "  Feature 0 - frame: " << frame_last_.features_.at(0).frame_id_ << std::endl;
    // std::cout << "  Feature 0 - x position: " << frame_last_.features_.at(0).keypoint_.pt.x << std::endl;
    // std::cout << "  Feature 0 - descriptor: " << frame_last_.features_.at(0).descriptor_.size() << std::endl;
    // std::cout << "  Feature 0 - landmark: " << frame_last_.features_.at(0).landmark_id_ << std::endl;

    // fill the extra information
    frame_last_.fill_frame(SE3(), true, curr_keyframe_id_);
    curr_keyframe_id_++;

    // insert the keyframe
    my_map_.insert_keyframe(frame_last_);

    // write_pose(frame_last_);

    return true;
}

bool VO::tracking(bool &if_insert_keyframe)
{
    // clear current frame
    frame_current_ = Frame();

    // get optimized last frame from the map
    if (frame_last_.is_keyframe_ == true)
    {
        frame_last_ = my_map_.keyframes_.at(frame_last_.keyframe_id_);
    }

    read_img(seq_, frame_current_.left_img_, frame_current_.right_img_);
    frame_current_.frame_id_ = seq_;

    // detect features in the current frame
    std::vector<cv::KeyPoint> keypoints_detected;
    cv::Mat descriptors_detected;
    feature_detection(frame_current_.left_img_, keypoints_detected, descriptors_detected);

    std::vector<cv::DMatch> feature_matches;

    cv::Mat descriptors_last;
    std::vector<cv::KeyPoint> keypoints_last;
    for (int i = 0; i < frame_last_.features_.size(); i++)
    {
        descriptors_last.push_back(frame_last_.features_.at(i).descriptor_);
        keypoints_last.push_back(frame_last_.features_.at(i).keypoint_);
    }
    feature_matching(descriptors_last, descriptors_detected, feature_matches);
    // std::cout << "****************** The size of features in two frames" << std::endl
    //           << descriptors_last.size() << std::endl
    //           << descriptors_detected.size() << std::endl;

    // show matched images
    // cv::Mat img;
    // cv::drawMatches(frame_last_.left_img_, keypoints_last, frame_current_.left_img_, keypoints_detected,
    //                 feature_matches, img);
    // cv::imshow("feature matches", img);
    // cv::waitKey(1);

    for (int i = 0; i < feature_matches.size(); i++)
    {
        // put the features into the frame with feature_id, frame_id, keypoint, descriptor
        // build the connection from feature to frame
        Feature feature_to_add(i, seq_,
                               keypoints_detected.at(feature_matches.at(i).trainIdx),
                               descriptors_detected.row(feature_matches.at(i).trainIdx));
        // build the connection from feature to landmark
        feature_to_add.landmark_id_ = frame_last_.features_.at(feature_matches.at(i).queryIdx).landmark_id_;
        frame_current_.features_.push_back(feature_to_add);
        // only after selected as keyframe: build the connection from landmark to feature
    }

    // debug printing
    // std::cout << "  Num of features in frame: " << frame_current_.features_.size() << std::endl;
    // std::cout << "  Feature 0 - id: " << frame_current_.features_.at(0).feature_id_ << std::endl;
    // std::cout << "  Feature 0 - frame: " << frame_current_.features_.at(0).frame_id_ << std::endl;
    // std::cout << "  Feature 0 - x position: " << frame_current_.features_.at(0).keypoint_.pt.x << std::endl;
    // std::cout << "  Feature 0 - descriptor: " << frame_current_.features_.at(0).descriptor_.size() << std::endl;
    // std::cout << "  Feature 0 - landmark: " << frame_current_.features_.at(0).landmark_id_ << std::endl;

    motion_estimation(frame_current_);

    // debug printing
    // std::cout << "  Num of features remaining in frame (RANSAC): " << frame_current_.features_.size() << std::endl;

    frame_current_.T_c_w_ = T_c_w_;

    T_c_l_ = frame_current_.T_c_w_ * frame_last_.T_c_w_.inverse();
    // std::cout << "  PnP estimated pose: " << frame_current_.T_c_w_.translation() << std::endl;
    // std::cout << "Relative rotation is: " << T_c_l_.angleY() << std::endl;

    bool check = check_motion_estimation();

    std::vector<cv::Point3f> pts_3d;
    if_insert_keyframe = insert_key_frame(check, pts_3d, keypoints_detected, descriptors_detected);
    // if (if_insert_keyframe)
    // {
    //     // only record pose for keyframes
    //     write_pose(frame_current_);
    // }
    // std::cout << "  Num of features in frame: " << frame_current_.features_.size() << std::endl;

    if (check)
    {
        if (if_rviz_)
        {
            rviz_visualize();
        }
        move_frame();
    }

    seq_++;

    // std::cout << "World origin in camera frame: " << T_c_w_.translation() << std::endl;

    // wait for user input for debugging
    // while (std::cin.get() != '\n');

    return check;
}

bool VO::pipeline(bool &if_insert_keyframe)
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
        if (tracking(if_insert_keyframe))
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
        return false;
        break;
    }
    default:
        std::cout << "Invalid state" << std::endl;
        return false;
        break;
    }

    // ros::Time time_end = ros::Time::now();
    // ROS_INFO("Time for this loop is: %f", (time_end - time_start).toSec());
    ros::spinOnce();

    return true;
}

} // namespace vslam
