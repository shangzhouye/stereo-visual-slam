/// \file
/// \brief Library for nonlinear optimization using g2o

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>
#include <stereo_visual_slam_main/map.hpp>
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

void optimize_map(std::unordered_map<unsigned long, Frame> &keyframes,
                  std::unordered_map<unsigned long, Landmark> &landmarks,
                  const cv::Mat &K)
{
    // g2o setup
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;

    // using LM method
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // add poses as vertices
    std::map<unsigned long, VertexPose *> vertices;
    unsigned long max_kf_id = 0;

    for (auto &keyframe : keyframes)
    {
        Frame kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(kf.keyframe_id_);
        vertex_pose->setEstimate(kf.T_c_w_);
        optimizer.addVertex(vertex_pose);
        if (kf.keyframe_id_ > max_kf_id)
        {
            max_kf_id = kf.keyframe_id_;
        }

        vertices.insert({kf.keyframe_id_, vertex_pose});
    }

    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    // K: convert c:mat to eigen matrix
    Eigen::Matrix3d K_eigen;
    K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // add edges
    int index = 1;

    // set robust kernel threshold
    double chi2_th = 5.991;

    std::map<EdgeProjection *, Feature> edges_and_features;

    for (auto &landmark : landmarks)
    {
        unsigned long landmark_id = landmark.second.landmark_id_;
        std::vector<Observation> observations = landmark.second.observations_;
        for (Observation &obs : observations)
        {

            Feature feat = keyframes.at(obs.keyframe_id_).features_.at(obs.feature_id_);

            // if it is not an inlier, do not add it to optimization
            if (feat.is_inlier == false)
                continue;

            // create a new edge
            EdgeProjection *edge = nullptr;
            edge = new EdgeProjection(K_eigen);

            // add the landmark as vertices
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end())
            {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second.to_vector_3d());
                // set the id following the maximum id of keyframes
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            edge->setId(index);

            edge->setVertex(0, vertices.at(obs.keyframe_id_));      // pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark

            // convert cv point to eigen
            Eigen::Matrix<double, 2, 1> measurement(feat.keypoint_.pt.x, feat.keypoint_.pt.y);
            edge->setMeasurement(measurement);
            edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());

            // set the robust kernel
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);

            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);

            index++;
        }
    }

    // do optimization
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // print the first pose optimized
    optimizer.setVerbose(true);
    std::cout << "Pose optimized (last one) = " << vertices.at(max_kf_id)->estimate().matrix() << std::endl;

    // modify the pose and landmark in the map
    for (auto &v : vertices)
    {
        keyframes.at(v.first).T_c_w_ = v.second->estimate();
    }
    for (auto &v : vertices_landmarks)
    {
        Eigen::Vector3d modified_pos_3d = v.second->estimate();
        landmarks.at(v.first).pt_3d_ = cv::Point3f(modified_pos_3d(0), modified_pos_3d(1), modified_pos_3d(2));
    }
}

} // namespace vslam