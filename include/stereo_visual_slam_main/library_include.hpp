#ifndef LIBRARY_INCLUDE_H
#define LIBRARY_INCLUDE_H
/// \file
/// \brief Include libraries that are commonly used in the package

// ROS
#include "ros/ros.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// Sophus
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#endif