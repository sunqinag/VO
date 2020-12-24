#include <iostream>
#include "include/visual_odmetry.h"

using namespace std;

int main()
{
//    google::ParseCommandLineFlags(&argc, &argv, true);
    string FLAGS_config_file = "/home/xtcsun/Github/VO/config/default.yaml";
    myslam::VisualOdmetry::Ptr vo(
            new myslam::VisualOdmetry(FLAGS_config_file));
//    assert(vo->Init() == true);
    vo->Init();
    vo->Run();

    return 0;
}