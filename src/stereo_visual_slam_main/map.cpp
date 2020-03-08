/// \file
/// \brief Definition of map structure

#include <stereo_visual_slam_main/map.hpp>
#include <stereo_visual_slam_main/types_def.hpp>

namespace vslam
{

int Map::insert_keyframe(Frame frame_to_add)
{
    current_keyframe_id_ = frame_to_add.keyframe_id_;

    if (keyframes_.find(frame_to_add.keyframe_id_) == keyframes_.end())
    {
        keyframes_.insert(std::make_pair(frame_to_add.keyframe_id_, frame_to_add));
    }
    else
    {
        keyframes_.at(frame_to_add.keyframe_id_) = frame_to_add;
    }

    // if the number of keyframes is more than the threshold, remove one
    if (keyframes_.size() > num_keyframes_)
    {
        remove_keyframe();
    }
}

int Map::insert_landmark(Landmark landmark_to_add)
{
    if (landmarks_.find(landmark_to_add.landmark_id_) == landmarks_.end())
    {
        landmarks_.insert(std::make_pair(landmark_to_add.landmark_id_, landmark_to_add));
    }
    else
    {
        landmarks_.at(landmark_to_add.landmark_id_) = landmark_to_add;
    }
}

int Map::remove_keyframe()
{
    // find two keyframes: minimum distance and maximum distance
    double max_distance = 0, min_distance = 1000000;
    double max_keyframe_id = 0, min_keyframe_id = 0;
    SE3 T_w_c = keyframes_.at(current_keyframe_id_).T_c_w_.inverse();
    for (auto &kf : keyframes_)
    {
        if (kf.first == current_keyframe_id_)
        {
            continue;
        }
        double distance = (kf.second.T_c_w_ * T_w_c).log().norm();
        if (distance > max_distance)
        {
            max_distance = distance;
            max_keyframe_id = kf.first;
        }
        if (distance < min_distance)
        {
            min_distance = distance;
            min_keyframe_id = kf.first;
        }
    }

    // std::cout << "max_keyframe_id: " << max_keyframe_id << " min_keyframe_id: " << min_keyframe_id << std::endl;

    const double min_threshold = 0.2;
    int keyframe_to_remove_id = -1;
    if (min_distance < min_threshold)
    {
        // remove the closest if distance is lower than 0.2
        keyframe_to_remove_id = min_keyframe_id;
    }
    else
    {
        keyframe_to_remove_id = max_keyframe_id;
    }

    // std::cout << "Keyframe removed: " << keyframe_to_remove_id << std::endl;

    // remove observations
    for (auto feat : keyframes_.at(keyframe_to_remove_id).features_)
    {
        // mark the observations that need to delete
        for (int i = 0; i < landmarks_.at(feat.landmark_id_).observations_.size(); i++)
        {
            if (landmarks_.at(feat.landmark_id_).observations_.at(i).keyframe_id_ == keyframe_to_remove_id &&
                landmarks_.at(feat.landmark_id_).observations_.at(i).feature_id_ == feat.feature_id_)
            {
                landmarks_.at(feat.landmark_id_).observations_.at(i).to_delete = true;
            }
        }
        // delete them
        landmarks_.at(feat.landmark_id_).observations_.erase(std::remove_if(
                                                              landmarks_.at(feat.landmark_id_).observations_.begin(),
                                                              landmarks_.at(feat.landmark_id_).observations_.end(),
                                                              [](const Observation &x) {
                                                                  return x.to_delete == true;
                                                              }),
                                                          landmarks_.at(feat.landmark_id_).observations_.end());
        // observation times minus one
        landmarks_.at(feat.landmark_id_).observed_times_--;
    }

    // remove the keyframe
    // std::cout << "number of keyframes before: " << keyframes_.size() << std::endl;
    keyframes_.erase(keyframe_to_remove_id);
    // std::cout << "number of keyframes after: " << keyframes_.size() << std::endl;

    clean_map();

    return 0;
}

int Map::clean_map()
{
    int num_landmark_removed = 0;
    for (auto iter = landmarks_.begin();
         iter != landmarks_.end();)
    {
        if (iter->second.observed_times_ == 0)
        {
            iter = landmarks_.erase(iter);
            num_landmark_removed++;
        }
        else
        {
            ++iter;
        }
    }

    // std::cout << "Removed landmarks: " << num_landmark_removed << std::endl;

    return 0;
}

} // namespace vslam
