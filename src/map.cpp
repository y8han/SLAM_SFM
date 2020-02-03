//
// Created by hanyunhai on 3/5/19.
//
#include "myslam/map.h"
namespace myslam
{
    void Map::insertKeyFrame(Frame::Ptr frame) {
        cout<<"Key frame size = "<<keyframes_.size()<<endl;
        if(keyframes_.find(frame->id_)==keyframes_.end())   //没找到的话返回尾后迭代器
            keyframes_.insert(make_pair(frame->id_,frame));  //增加新的一帧
        else
            keyframes_[frame->id_]=frame;
    }
    void Map::insertMapPoint(myslam::MapPoint::Ptr map_point) {
        if(map_points_.find(map_point->id_)==map_points_.end())
            map_points_.insert(make_pair(map_point->id_,map_point)); //增加新的地图点
        else
            map_points_[map_point->id_]=map_point;
    }
}