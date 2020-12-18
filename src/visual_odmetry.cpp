//
// Created by xtcsun on 2020/12/18.
//

#include "../include/visual_odmetry.h"
#include <chrono>
#include "../include/config.h"

namespace myslam {
    VisualOdmetry::VisualOdmetry(std::string &config_path)
            : config_file_path_(config_path) {}

    bool VisualOdmetry::Init() {
        // read config file
        if (Config::SetParameterFile(config_file_path_) == false) {
            return false;
        }

        dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        dataset_->Init();

        // create components and links;
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0),dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0),dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VisualOdmetry::Run() {
        while (1){
            std::cout<<"VO is running"<<std::endl;
            if (Step() == false){
                break;
            }
        }

        backend_->Stop();
        viewer_->Close();
        std::cout<<"VO exit"<<std::endl;
    }

    bool VisualOdmetry::Step() {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if (new_frame == nullptr) return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_out = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        std::cout<<"VO cost time: "<<time_out.count()<<" seconds";
        return success;
    }
}