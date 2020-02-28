#ifndef MAP_INCLUDE_GUARD_HPP
#define MAP_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of map structure

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>

namespace vslam
{

struct Map
{
public:
    std::unordered_map<unsigned long, Frame> keyframes_;
    std::unordered_map<unsigned long, Landmark> landmarks_;

    // number of active keyframes
    const int num_keyframes_ = 7;

    // id of the current frame
    int current_frame_id_ = 0;

public:
    Map() {}

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