//
// Created by sun on 2020/12/12.
//

#include "../include/frame.h"

namespace myslam {


    Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
            : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

    Frame::Ptr Frame::CreateFrame()
    {
        static long factory_id = 0;
        Frame::Ptr new_frmae(new Frame);
        new_frmae->id_ = factory_id++;
        return new_frmae;
    }

    void Frame::SetKeyFrame()
    {
        static long keyframe_factory_id = 0;
        is_keyfeame_ = true;
        keyfemae_id_ = keyframe_factory_id++;
    }
}