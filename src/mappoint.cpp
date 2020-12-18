//
// Created by sun on 2020/12/13.
//

#include "../include/mappoint.h"
#include "../include/features.h"


namespace myslam{
    MapPoint::MapPoint(long id,Vec3 position):id_(id),pos_(position){}

    MapPoint::Ptr MapPoint::CreateMewMappoint() {
        static long factory_id =0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        for (auto iter = observations_.begin();iter != observations_.end();iter++) {
            if (iter->lock() == feat){
                observations_.erase(iter);
                feat->mappoint_.reset();
                observed_times_--;
                break;
            }
        }
    }
}
