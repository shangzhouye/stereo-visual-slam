/// \file
/// \brief ROS Node to run the VO

#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include "stereo_visual_slam_main/library_include.hpp"
#include "stereo_visual_slam_main/frame.hpp"
#include <vector>
#include <string>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include "stereo_visual_slam_main/visual_odometry.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_vo");
    ros::NodeHandle nh;
    std::string dataset;
    nh.getParam("/dataset", dataset);

    vslam::VO myVO(dataset, nh);
    myVO.VOpipeline(1100);

    ros::spin();

    return 0;
}