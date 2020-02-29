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

    for (int ite = 0; ite < 4050; ite++)
    {
        bool not_lost = true;
        not_lost = my_VO.pipeline();
        // debug printing
        // learned: accessing non-existing [i] in unordered would create a new component
        // learned: use iterator to traverse unordered map
        // learned: at() throws out-of-range exception whereas operator[] shows undefined behavior.
        std::cout << "Num of landmarks: " << my_map.landmarks_.size() << std::endl;
        std::cout << "Num of keyframes: " << my_map.keyframes_.size() << std::endl;
        for (auto &kf : my_map.keyframes_)
        {
            std::cout << "  Num of features in keyframe: " << kf.second.keyframe_id_ << " - " << kf.second.features_.size() << std::endl;
        }

        if (not_lost == false)
        {
            break;
        }
        
    }

    ros::spin();

    return 0;
}