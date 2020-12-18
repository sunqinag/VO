//
// Created by xtcsun on 2020/12/17.
//

#include "../include/viewer.h"

namespace myslam{
    Viewer::Viewer(){
        std::cout<<"Viewer  构造函数还未实现"<<std::endl;
    }

    void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::UpdateMap() {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_!= nullptr);
        activate_keyframes_ = map_->GetActiveKeyFrames();
        activate_landmarkes_ = map_->GetActiveMapPoints();
        map_updated_ = true;
    }

    void Viewer::Close() {
        viewer_running_ = false;
        viewer_thread_.join();
    }
}