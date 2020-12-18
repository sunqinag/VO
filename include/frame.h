//
// Created by sun on 2020/12/12.
//

#ifndef VO_FRAME_H
#define VO_FRAME_H

#include "common_include.h"

namespace myslam {
    // 前向声明：假设有两个类A和B，类A要将类B的对象(或者指正)作为自己的成员使用，并且类B将类A的对象(或者指针)作为自己可以访问的数据，那么这个时候要在a.h中include b.h,同时在b.h 中要include a.h，但是相互包含是不可以的，这个时候就要用到类的前向声明了。

    struct MapPoint;
    struct Feature;

    struct Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;
        unsigned long keyframe_id_ = 0;
        bool is_keyframe_ = false;
        double time_stamp_;
        SE3 pose_;
        std::mutex pose_mutex_;
        cv::Mat left_img_, right_img_;

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // extracted features in right image
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:
        Frame() {}

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

        //set and get pose, thread safe
        SE3 Pose() {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const SE3 &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        // 设置关键帧并分配关键帧id
        void SetKeyFrame();

        // 对象与对象之间的成员变量是相互独立的.要想共用数据,则需要使用静态成员或静态方法
        // 工厂构建模式，分配id
        static std::shared_ptr<Frame> CreateFrame();


    };

}


#endif //VO_FRAME_H
