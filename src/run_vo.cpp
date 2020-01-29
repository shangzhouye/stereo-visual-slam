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
#include "stereo_visual_slam_main/structureless_vo.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_vo");
    ros::NodeHandle nh;
    string dataset;
    nh.getParam("/dataset", dataset);

    vslam::StructurelessVO myVO(dataset);
    myVO.initialization();

    ros::spin();

    return 0;
}