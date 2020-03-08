#ifndef MAP_INCLUDE_GUARD_HPP
#define MAP_INCLUDE_GUARD_HPP
/// \file
/// \brief Definition of map structure

#include <cmath>
#include <iostream>
#include <stereo_visual_slam_main/library_include.hpp>
#include <stereo_visual_slam_main/types_def.hpp>
#include <stereo_visual_slam_main/visualization.hpp>

namespace vslam
{

struct Map
{
public:
    std::unordered_map<unsigned long, Frame> keyframes_;
    std::unordered_map<unsigned long, Landmark> landmarks_;

    // number of active keyframes
    const int num_keyframes_ = 10;

    // id of the current keyframe
    int current_keyframe_id_ = 0;

    // visualization module
    VslamVisual my_visual_;

public:
    Map(ros::NodeHandle &nh) : my_visual_(nh) {}

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

    /*! \brief remove a keyframe from the map
    *
    *  \return if successful
    */
    int remove_keyframe();

    /*! \brief clean the map: remove landmarks with no observations
    *
    *  \return if successful
    */
    int clean_map();

    /*! \brief publish keyframes undergoing optimization
    *
    */
    void publish_keyframes();
};

} // namespace vslam

#endif