#ifndef LIBRARY_INCLUDE_H
#define LIBRARY_INCLUDE_H

// for eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// for Sophus
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using cv::Mat;

#endif