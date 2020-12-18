//
// Created by xtcsun on 2020/12/18.
//

#ifndef VO_DATASET_H
#define VO_DATASET_H

#include "camera.h"
#include "common_include.h"
#include "frame.h"

namespace myslam {
    /**
     * 读取数据集,构造时传入配置文件和dataset_dir为数据及路径
     * init之后可以获得相机和下一帧图像
     */

    class Dataset {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;

        Dataset(const std::string &dataset_path);

        // 初始化,返回是否成功
        bool Init();

        /// create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();

        //get camera by id
        Camera::Ptr GetCamera(int camera_id) const {
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };
}

#endif //VO_DATASET_H
