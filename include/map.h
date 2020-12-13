
//
// Created by sun on 2020/12/13.
//

#ifndef VO_MAP_H
#define VO_MAP_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam {
    /**
     * @brief 地图
     * 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等
     */
    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        // 增加一个关键帧
        void InsertKeyFrame(Frame::Ptr frame);

        // 增加一个地图顶点
        void InsertMappoint(MapPoint::Ptr mappoint);

        // 获取所有地图点
        LandmarksType GetAllMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmakrks_;
        }

        // 获取所有关键帧
        KeyframesType GetAllKeyFrames() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        //  获取激活地图点
        LandmarksType GetActivateMapPoints() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return activate_landmarks_;
        }

        // 获取激活关键帧
        KeyframesType GetAllKeyFrame() {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return activate_frames_;
        }

        // 清理map中观测数量为0的点
        void CleanMap();

    private:
        std::mutex data_mutex_;
        LandmarksType landmakrks_;  // all landmarks
        KeyframesType keyframes_;   // all key-frames
        LandmarksType activate_landmarks_; // active landmarks
        KeyframesType activate_frames_;    // activate keyframes

        Frame::Ptr current_frame_ = nullptr;

        int num_activate_keyframe_ = 7; // 激活关键帧的数量
    };
}

#endif //VO_MAP_H
