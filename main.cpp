#include <iostream>
#include "include/visual_odmetry.h"

using namespace std;

int main()
{
    string config_file = "/home/xtcsun/CLionProjects/VO/config/default.yaml";
    LOG(INFO)<<"进入VO";
    myslam::VisualOdmetry::Ptr vo(new myslam::VisualOdmetry(config_file));
    bool success = vo->Init();
//    assert(vo->Init() == true);
    vo->Run();

    return 0;
}