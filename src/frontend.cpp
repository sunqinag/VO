//
// Created by xtcsun on 2020/12/15.
//

#include <opencv2/opencv.hpp>
#include "../include/frontend.h"
#include "../include/config.h"
#include "../include/features.h"
#include "../include/algorithm.h"
#include "../include/viewer.h"
#include "../include/backend.h"
#include "../include/g2o_types.h"

/**
 * 前端功能:根据双目图像确定该帧的位姿: 实现起来则有多种方法--看书
 * 1.前端本身有初始化,正常追踪,追踪丢失三种状态
 * 2. 在前端初始化状态中,更具左右目之间的光流匹配,寻找可以三角化的地图点(mappoint)成功时可以建立初始地图
 * 3. 追踪阶段中,前端计算上一帧特征点到当前帧的光流,根据光流结果计算图像的位姿,该计算只使用左目图像,不使用右目
 * 4. 如果追踪到的点较少,就判断当前帧为关键帧,对于关键帧做一下几件事:
 *      * 提取新的特征点;
 *      * 找到这些点在右图的对应点,用三角化建立新的路标点
 *      * 将新的关键帧和路标点加入地图,并触发一次后端优化
 * 5. 如果追踪丢失,就重置前端系统,重新初始化.
 * */

namespace myslam {

    Frontend::Frontend() {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    bool Frontend::AddFrame(Frame::Ptr frame) {
        current_frame_ = frame;

        switch (status_) { //必须是一个整型或枚举类型，或者是一个 class 类型
            case FrontendStatus::INITING:
                StereoInit();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                Track();

            case FrontendStatus::LOST:
                Reset();

        }
    }

    bool Frontend::StereoInit() {
        int num_features_left = DetectFeatures(); //监测当前帧左图像中的特征,返回数量keypoint保存在当前帧
        int num_coor_features = FindFeaturesInRight();
        if (num_coor_features < num_features_init_) {
            return false;
        }

        bool build_map_seccess = BuildInitMap();
        if (build_map_seccess) {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_) {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    int Frontend::DetectFeatures() {
        // 检测当前帧左图像中的特征,返回数量keypoint保存在当前帧
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_) {
            // 将现有的左图中的特征点都画一个小小的框,放到mask上
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }

        // 检测现有左图,将特征点和左图共建Feature,放到列表最后
        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints) {
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
        for (auto &kp : current_frame_->features_left_) {
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
            if (status[i]) {                                     // 如果在右图找到相应的流就从右图特征中抽取出来,形成一个新的feature
                cv::KeyPoint kp(kps_right[i], 7); //关键点和关键点直径大小
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
            if (current_frame_->features_right_[i] == nullptr)
                continue;
            // create map point from triangulation
            std::vector<Vec3> points{// 不是应该输入深度嘛?
                    camera_left_->pixel2camera(
                            Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                 current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                                     current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateMewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->mappoint_ = new_map_point;
                current_frame_->features_right_[i]->mappoint_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMappoint(new_map_point);
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        //backend_->UpdateMap(); // 还未实现
        std::cout << "Initial map created with " << cnt_init_landmarks << " map points" << std::endl;
        return true;
    }

    bool Frontend::Track() {
        if (last_frame_) {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        int num_track_last = TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();

        if (tracking_inliers_ > num_features_tracking_) {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (tracking_inliers_ > num_features_tracking_bad) {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        } else {
            // lost
            status_ = FrontendStatus::LOST;
        }
        InsertKeyFrame();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if (viewer_) viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Frontend::InsertKeyFrame() {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
            // 仍然拥有足够多的特征点,不要放进关键帧中
            return false;
        }
        // 当前帧是一个新的关键帧
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        std::cout << "Set frame " << current_frame_->id_ << "as keyframe" << current_frame_->keyframe_id_;

        SetObservationsForKeyFrame();
        DetectFeatures(); //提取新的特征点

        // 在右图中进行追踪
        FindFeaturesInRight();
        // 三角化地图点
        TriangulateNewPoints();
        // 更新后端因为现在有一个新的关键帧
        backend_->UpdateMap();

        if (viewer_) viewer_->UpdateMap();

        return true;
    }

    int Frontend::TriangulateNewPoints() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_left_[i]->mappoint_.expired() &&
                current_frame_->features_right_[i] != nullptr) {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                        camera_left_->pixel2camera(
                                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                     current_frame_->features_left_[i]->position_.pt.y)),
                        camera_right_->pixel2camera(
                                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                     current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                    auto new_map_point = MapPoint::CreateMewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(current_frame_->features_left_[i]);
                    new_map_point->AddObservation(current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->mappoint_ = new_map_point;
                    current_frame_->features_right_[i]->mappoint_ = new_map_point;
                    map_->InsertMappoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        std::cout << "new  landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }


    void Frontend::SetObservationsForKeyFrame() {
        for (auto &feat: current_frame_->features_left_) {
            auto mp = feat->mappoint_.lock();
            if (mp) mp->AddObservation(feat);
        }
    }

    int Frontend::EstimateCurrentPose() {
        // setup g20
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverTye;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverTye>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        // K
        Mat33 K = camera_left_->K();

        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            auto mp = current_frame_->features_left_[i]->mappoint_.lock();
            if (mp) {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                        toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }
        // 估计姿势确定异常值
        const double chi2_h = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration) {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // 统计异常值
            for (size_t i = 0; i < edges.size(); ++i) {
                auto e = edges[i];
                if (features[i]->is_outlier_) {
                    e->computeError();
                }
                if (e->chi2() > chi2_h) {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                } else {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2) {
                    e->setRobustKernel(nullptr);
                }

            }
        }

        std::cout << "Outlier/Inlier in pose estimation: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier;
        //设置位姿和异常
        current_frame_->SetPose(vertex_pose->estimate());

        std::cout << "Current Pose=\n" << current_frame_->Pose().matrix();

        for (auto &feat: features) {
            if (feat->is_outlier_) {
                feat->mappoint_.reset();
                feat->is_outlier_ = false;//maybe we can still use it in feature
            }
        }
        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame() {
        // 用右图使用LK光流来估计位姿
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp: last_frame_->features_left_) {
            if (kp->mappoint_.lock()) {
                // use project point
                auto mp = kp->mappoint_.lock();
                auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_last.push_back(kp->position_.pt);// 压入当前关键帧的坐标
                kps_current.push_back(cv::Point2f(px[0], px[1])); // 压入当前mappoint经过位资调整后的坐标
            } else {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
                last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current, status, error,
                cv::Size(11, 11), 3, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                                      0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->mappoint_ = last_frame_->features_left_[i]->mappoint_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }
        std::cout << "Find " << num_good_pts << " in the last image";
        return num_good_pts;
    }

    bool Frontend::Reset() {
        std::cout << "Reset is not inplemented." << std::endl;
        return true;
    }
} // namespace myslam