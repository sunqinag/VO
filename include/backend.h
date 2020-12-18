//
// Created by xtcsun on 2020/12/17.
//

#ifndef VO_BACKEND_H
#define VO_BACKEND_H

#include "common_include.h"
#include "frame.h"
#include "map.h"
#include "camera.h"

namespace myslam{

    class Map;

    /**
     * 后端有单独的优化线程,在map更新时启动优化
     * map更新由前端触发
     */
     class Backend{
     public:
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
         typedef std::shared_ptr<Backend> Ptr;

         // 构造函数中启动优化线程并挂起
         Backend();

         // 设置左右目相机,用于获得内外参
         void SetCameras(Camera::Ptr left, Camera::Ptr right){
             cam_left_ = left;
             cam_right_ = right;
         }

         // 设置地图
         void SetMap(std::shared_ptr<Map> map) {map_ = map;}

         // 触发地图更新,启动优化
         void UpdateMap();

         // 关闭后端线程
         void Stop();

     private:

         // 后端线程
         void BackendLoop();

         // 对关键帧和路标点进行优化
         void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

         std::shared_ptr<Map> map_;
         std::thread backend_thread_;
         std::mutex data_mutex_;

         std::condition_variable map_update_;
         // std::atomic<bool> backend_running_; //std::atomic对int, char, bool等数据结构进行原子性封装，
         // 在多线程环境中，对std::atomic对象的访问不会造成竞争-冒险。利用std::atomic可实现数据结构的无锁设计。
         std::atomic<bool> backend_running_;

         Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
     };
}


#endif //VO_BACKEND_H
