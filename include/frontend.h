//
// Created by xtcsun on 2020/12/15.
//

#ifndef VO_FRONTEND_H
#define VO_FRONTEND_H

#include <opencv2/features2d.hpp>
#include "common_include.h"
#include "frame.h"
#include "map.h"
#include "camera.h"

namespace myslam
{

    class Viewer;
    class Backend;

    enum class FrontendStatus {INITING,TRACKING_GOOD,TRACKING_BAD,LOST};

    // 全短估计当前帧Pose,在班组关键帧条件时向地图加入关键帧并触发优化
    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        // 外部借口,添加一个帧并计算其定位结果
        bool AddFrame(Frame::Ptr frame);

        // set函数
        void SetMap(Map::Ptr map){map_=map;}

        FrontendStatus GetStatus() const { return status_;}

        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

        void SetCameras(Camera::Ptr left,Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

    private:

        /**
         * Track in normal mode
         * @return true if success
         */
        bool Track();

        /**
         尝试使用当前帧中保存的立体图像初始化前端_
         */
        bool StereoInit();

        /**
         * 在当前帧中检测左图像中的特征_
         */
        int DetectFeatures();

        /**
         * 找到对应在右图像中的特征 返回找到对应的特征数量
         */
         int FindFeaturesInRight();

        /**
        * 用单张图构建初始化特征
        */
         bool BuildInitMap();

        /**
         * 追踪上一帧 返回追踪到的点
         */
        int TrackLastFrame();

        /**
         * 估计当前帧的位姿
         */
        int EstimateCurrentPose();

        /**
         * 将当前帧设置为关键帧,并添加进后端
         */
         bool InsertKeyFrame();

        /**
         *将关键帧中的要素设置为地图点的新观察
         */
         void SetObservationsForKeyFrame();

        /**
         * 三角化当前帧的2D坐标
         */
         int TriangulateNewPoints();

        /**
        * Reset when lost
        */
        bool Reset();

        // data
        FrontendStatus status_ = FrontendStatus ::INITING;

        Frame::Ptr current_frame_ = nullptr;  //当前帧
        Frame::Ptr last_frame_ = nullptr;     //上一帧
        Camera::Ptr camera_left_ = nullptr;   // 左侧相机
        Camera::Ptr camera_right_ = nullptr;  // 右侧相机

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        SE3 relative_motion_;   //当前帧与上一帧的相对运动,用于估计当前帧pose初值
        int tracking_inliers_ = 0;  //用于测试新的关键帧

        // 参数
        int num_features_=200;
        int num_features_init_=100;
        int num_features_tracking_=50;
        int num_features_tracking_bad = 20;
        int num_features_needed_for_keyframe_=80;

        // poencv的特征点检测器
        cv::Ptr<cv::GFTTDetector> gftt_;
    };

}



#endif //VO_FRONTEND_H
