//
// Created by sun on 2020/12/13.
//

#include "../include/map.h"
#include "features.h"

namespace myslam
{
    void  Map::InsertKeyFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;
        // 函数会返回一个输入迭代器，当 find() 函数查找成功时，
        // 其指向的是在 [first, last) 区域内查找到的第一个目标元素；如果查找失败，则该迭代器的指向和 last 相同
        if (keyframes_.find(frame->keyfemae_id_) == keyframes_.end()){
            // 当此帧在关键帧列表的末尾时,从后面插入
            keyframes_.insert(std::make_pair(frame->keyfemae_id_,frame));
            activate_frames_.insert(std::make_pair(frame->keyfemae_id_,frame));
        }else{
            // 如果当前帧的id在keyframes_,activate_frames_列表中就替换掉
            keyframes_[frame->keyfemae_id_] = frame;
            activate_frames_[frame->keyfemae_id_] = frame;
        }
        
        // if (activate_frames_.size() > num_activate_keyframe_)
        // {
            
        // }  
    }

    void Map::RemoveOldKeyframe()
    {
        if (current_frame_ == nullptr) return;
        // 寻找与当前帧最近与最远的两个关键帧
        double max_dis = 0,min_dis=99999;
        double max_kf_dis =0,min_kf_dis = 0;
        auto Twc = current_frame_->Pose().inverse();
        for (auto& kf:activate_frames_) {
            // https://blog.csdn.net/tcx1992/article/details/80928790
            // second就是相当于value
            if (kf.second == current_frame_) continue; //将本帧跳过
            auto dis = (kf.second->Pose()*Twc).log().norm(); //
        }
        
        
    }
} // namespace myslam

