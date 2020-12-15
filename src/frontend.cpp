//
// Created by xtcsun on 2020/12/15.
//

#include "../include/frontend.h"
#include "../include/config.h"


namespace myslam{

    Frontend::Frontend()
    {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>('num_features'),0.01,20);
    }
}