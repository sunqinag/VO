#include <iostream>
#include "include/visual_odmetry.h"

using namespace std;

int main()
{
    string config_file = "/home/xtcsun/CLionProjects/VO/config/default.yaml";
    cout<<"进入VO"<<endl;
    myslam::VisualOdmetry::Ptr vo(new myslam::VisualOdmetry(config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}