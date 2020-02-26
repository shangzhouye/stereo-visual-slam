/// \file
/// \brief Definition of map structure

#include <stereo_visual_slam_main/map.hpp>
#include <stereo_visual_slam_main/types_def.hpp>

namespace vslam
{

int Map::insert_keyframe(Frame frame_to_add)
{
    if (keyframes_.find(frame_to_add.keyframe_id_) == keyframes_.end())
    {
        keyframes_.insert(std::make_pair(frame_to_add.keyframe_id_, frame_to_add));
    }
    else
    {
        keyframes_[frame_to_add.keyframe_id_] = frame_to_add;
    }
}

int Map::insert_landmark(Landmark landmark_to_add)
{
    if (landmarks_.find(landmark_to_add.id_) == landmarks_.end())
    {
        landmarks_.insert(std::make_pair(landmark_to_add.id_, landmark_to_add));
    }
    else
    {
        landmarks_[landmark_to_add.id_] = landmark_to_add;
    }
}

} // namespace vslam
