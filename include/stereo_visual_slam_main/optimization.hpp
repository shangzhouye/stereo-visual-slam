#ifndef OPTIMIZATION_INCLUDE_GUARD_HPP
#define OPTIMIZATION_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for nonlinear optimization using g2o

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
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

namespace vslam
{

// define aligned_allocator eigen type for g2o
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> G2OVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> G2OVector3d;

// define the vertex
// parameter: [dimention of parameters to optimize, data type]
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /*! \brief implement reset function: reset SE3 to zero
    */
    virtual void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

    /*! \brief implement update function: left multiplication of SE3
    */
    virtual void oplusImpl(const double *update) override;

    // leave read and write empty
    virtual bool read(std::istream &in) override {}
    virtual bool write(std::ostream &out) const override {}
};

// define the edge
// parameter: [dimension of measurement, data type, vertex type]
class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // constructor to read in the parameters needed
    EdgeProjection(const Eigen::Vector3d &point_3d, const Eigen::Matrix3d &K) : point_3d_(point_3d), K_(K) {}

    /*! \brief compute the error
    *
    * 1. get the current optimized pose in this iteration
    *       it is the _estimate in vertice class
    *       it is already in SE3 because exp has been done in update function
    * 2. calculate error by e = u - (1/s)KTP
    */
    virtual void computeError() override;

    /*! \brief give the analytic solution of jacobian
    */
    virtual void linearizeOplus() override;

    virtual bool read(std::istream &in) override {}

    virtual bool write(std::ostream &out) const override {}

public:
    Eigen::Vector3d point_3d_;
    Eigen::Matrix3d K_;
};

} // namespace vslam

#endif