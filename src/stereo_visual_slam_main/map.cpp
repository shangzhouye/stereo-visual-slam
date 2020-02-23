/// \file
/// \brief Definition of map structure

#include <stereo_visual_slam_main/map.hpp>

namespace vslam
{
    int Map::insert_keyframe(Frame frame_to_add){
        keyframes_.push_back(frame_to_add);
    }

    int Map::insert_landmark(Landmark landmark_to_add){
        landmarks_.push_back(landmark_to_add);
    }

} // namespace vslam
