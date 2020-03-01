/// \file
/// \brief Library for nonlinear optimization using g2o

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>
#include <vector>
#include <string>
#include <unistd.h>
#include <stereo_visual_slam_main/visualization.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <stereo_visual_slam_main/optimization.hpp>

namespace vslam
{

void VertexPose::oplusImpl(const double *update)
{
    Eigen::Matrix<double, 6, 1> update_se3;
    // get the pertubation from the update vector
    update_se3 << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3d::exp(update_se3) * _estimate;
}

void VertexXYZ::oplusImpl(const double *update)
{
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
}

void EdgeProjection::computeError()
{
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Eigen::Vector3d pos_pixel = K_ * (T * v1->estimate());
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
}

void EdgeProjection::linearizeOplus()
{
    const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
    const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
    Sophus::SE3d T = v0->estimate();
    Eigen::Vector3d pw = v1->estimate();
    Eigen::Vector3d pos_cam = T * pw;
    double fx = K_(0, 0);
    double fy = K_(1, 1);
    double cx = K_(0, 2);
    double cy = K_(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Zinv = 1.0 / (Z + 1e-18);
    double Zinv2 = Zinv * Zinv;
    _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
        -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
        fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
        -fy * X * Zinv;
    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T.rotationMatrix();
}

} // namespace vslam