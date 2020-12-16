//
// Created by xtcsun on 2020/12/15.
//

#include <opencv2/opencv.hpp>
#include "../include/frontend.h"
#include "../include/config.h"
#include "../include/features.h"
#include "../include/algorithm.h"


namespace myslam {

    Frontend::Frontend() {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    bool Frontend::AddFrame(Frame::Ptr frame) {
        current_frame_ = frame;

        switch (status_) {//必须是一个整型或枚举类型，或者是一个 class 类型
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
//                Track();
                break;
            case FrontendStatus::LOST:
//                Reset();
                break;
        }

    }

    bool Frontend::StereoInit() {
        int num_features_left = DetectFeatures();  //监测当前帧左图像中的特征,返回数量keypoint保存在当前帧
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_) {
            return false;
        }

        bool build_map_seccess = BuildInitMap();
    }

    int Frontend::DetectFeatures() {
        // 检测当前帧左图像中的特征,返回数量keypoint保存在当前帧
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat:current_frame_->features_left_) {
            // 将现有的左图中的特征点都画一个小小的框,放到mask上
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }

        // 检测现有左图,将特征点和左图共建Feature,放到列表最后
        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp: keypoints) {
            current_frame_->features_left_.push_back(
                    Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }
        std::cout << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight() {
        // 在右图使用LK光流估算位资
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp:current_frame_->features_left_) {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->mappoint_.lock();
            if (mp) { // 遍历左图特征,如果这个特征有关联的地图点,就将地图点位姿从世界坐标系转为像素坐标系,并保存到kps_right
                // use projected points as initial guess
                auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            } else {
                // use same pixel in left image
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
                current_frame_->left_img_, current_frame_->right_img_, kps_left,
                kps_right, status, error, cv::Size(11, 11), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001),
                cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) { // 如果在右图找到相应的流就从右图特征中抽取出来,形成一个新的feature
                cv::KeyPoint kp(kps_right[i], 7);//关键点和关键点直径大小
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false; ///是不是反了?
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            } else {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        std::cout << "Find " << num_good_pts << " in the right image";
        return num_good_pts;
    }

    bool Frontend::BuildInitMap() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_right_[i] == nullptr) continue;
            // create map point from triangulation
            std::vector<Vec3> points{ // 不是应该输入深度嘛?
                    camera_left_->pixel2camera(
                            Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                 current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                                     current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses,points,pworld) && pworld[2]>0){
                auto new_map_point = MapPoint::CreateMewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->mappoint_ = new_map_point;
                current_frame_->features_right_[i]->mappoint_ = new_map_point;
                cnt_init_landmarks++;
//                map_->InsertMappoint(new_map_point);
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        //backend_->UpdateMap(); // 还未实现
        std::cout<<"Initial map created with "<<cnt_init_landmarks<<" map points"<<std::endl;
        return true;
    }
}