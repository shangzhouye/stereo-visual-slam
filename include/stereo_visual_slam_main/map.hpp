#ifndef MAP_INCLUDE_GUARD_HPP
#define MAP_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of map structure

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/frame.hpp>
#include <stereo_visual_slam_main/landmark.hpp>

namespace vslam
{

struct Map
{
public:
    std::vector<Frame> keyframes_;
    std::vector<Landmark> landmarks_;

public:
    Map() : keyframes_(std::vector<Frame>()), landmarks_(std::vector<Landmark>()) {}

    /*! \brief inset a keyframe into the map
    *
    *  \param frame_to_add - the frame to be added
    *  \return if successful
    */
    int insert_keyframe(Frame frame_to_add);

    /*! \brief inset a landmark into the map
    *
    *  \param landmark_to_add - the landmark to be added
    *  \return if successful
    */
    int insert_landmark(Landmark landmark_to_add);
};

} // namespace vslam

#endif