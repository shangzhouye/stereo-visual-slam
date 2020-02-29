/// \file
/// \brief ROS Node to run the Vslam

#include <cmath>
#include <iostream>
#include "stereo_visual_slam_main/library_include.hpp"
#include "stereo_visual_slam_main/types_def.hpp"
#include <vector>
#include <string>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include "stereo_visual_slam_main/visual_odometry.hpp"
#include <stereo_visual_slam_main/map.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_vslam");
    ros::NodeHandle nh;
    std::string dataset;
    nh.getParam("/dataset", dataset);

    vslam::Map my_map;
    vslam::VO my_VO(dataset, nh, my_map);

    for (int ite = 0; ite < 1000; ite++)
    {
        my_VO.pipeline();
        // debug printing
        std::cout << "Num of landmarks: " << my_map.landmarks_.size() << std::endl;
        std::cout << "  Lanmark 0 - id: " << my_map.landmarks_[0].landmark_id_ << std::endl;
        std::cout << "  Lanmark 0 - observed_times: " << my_map.landmarks_[0].observed_times_ << std::endl;
        std::cout << "  Lanmark 0 - x position: " << my_map.landmarks_[0].pt_3d_.x << std::endl;
        std::cout << "  Lanmark 0 - first feature id: " << my_map.landmarks_[0].observations_.at(0).feature_id_ << std::endl;
        std::cout << "Num of keyframes: " << my_map.keyframes_.size() << std::endl;
        for (int num_keyframe = 0; num_keyframe < my_map.keyframes_.size(); num_keyframe++)
        {
            std::cout << "  Num of features in keyframe: " << num_keyframe << " - " << my_map.keyframes_[num_keyframe].features_.size() << std::endl;
        }
    }

    ros::spin();

    return 0;
}