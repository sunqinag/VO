//
// Created by xtcsun on 2020/12/15.
//

#include "../include/config.h"

namespace myslam{
    bool Config::SetParameterFile(const std::string &filename) {
        if (config_ == nullptr){
            config_ = std::shared_ptr<Config>(new Config);
        }
        config_->file_ = cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        if (config_->file_.isOpened() == false){
            std::cout<<"parameter file  "<<filename<<" dose not exit."<<std::endl;
            config_->file_.release();
            return false;
        }
        return true;
    }

    Config::~Config() {
        if (file_.isOpened())
            file_.release();
    }

    std::shared_ptr<Config> Config::config_= nullptr;
}