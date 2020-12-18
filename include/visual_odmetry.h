//
// Created by xtcsun on 2020/12/18.
//

#ifndef VO_VISUAL_ODMETRY_H
#define VO_VISUAL_ODMETRY_H

#include "backend.h"
#include "common_include.h"
#include "dataset.h"
#include "frontend.h"
#include "viewer.h"

namespace myslam{

    class VisualOdmetry{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdmetry> Ptr;

        // constructor with config file
        VisualOdmetry(std::string &config_path);

        /**
         * do initialization things before run
         * @return true if success
         */
         bool Init();

        /**
         * start vo in the dataset
         */
         void Run();

        /**
         * Make a step forward in dataset
         */
         bool Step();

         /// 获取前端状态
         FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus();}

    private:
        bool inited_ = false;
         std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        Dataset::Ptr dataset_ = nullptr;
    };
}




#endif //VO_VISUAL_ODMETRY_H
