//
// Created by xtcsun on 2020/12/17.
//

#ifndef VO_VIEWER_H
#define VO_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "common_include.h"
#include "frame.h"
#include "map.h"

namespace myslam{
    /**
     * 可视化部分
     * */
     class Viewer{
     public:
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
         typedef std::shared_ptr<Viewer> Ptr;

         Viewer();

         void SetMap(Map::Ptr map){map_ = map;}

         void Close();


          // 增加一个当前帧
         void AddCurrentFrame(Frame::Ptr current_frame);

         // 更新地图
         void UpdateMap();

         /// plot the features in current frame into an image
         cv::Mat PlotFrameImage();

     private:

         void ThreadLoop();

         void DrawFrame(Frame::Ptr frame, const float* color);

         void DrawMapPoints();

         void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

         Frame::Ptr current_frame_ = nullptr;
         Map::Ptr map_ = nullptr;
         std::thread viewer_thread_;
         bool viewer_running_ = true;

         std::unordered_map<unsigned long,Frame::Ptr> activate_keyframes_;
         std::unordered_map<unsigned long, MapPoint::Ptr> activate_landmarkes_;
         bool map_updated_ = false;

         std::mutex viewer_data_mutex_;
     };
}


#endif //VO_VIEWER_H
