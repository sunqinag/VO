//
// Created by xtcsun on 2020/12/15.
//

#ifndef VO_CONFIG_H
#define VO_CONFIG_H

#include "common_include.h"
namespace myslam{
    class Config{
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;
        Config() {}
    public:
        ~Config();

        // 设置新的配置文件
        static bool SetParameterFile(const std::string &filename);

        template <typename T>
        static T Get(int key){
            return T(Config::config_->file_[key]);
        }
    };
}



#endif //VO_CONFIG_H
