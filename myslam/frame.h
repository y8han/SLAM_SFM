//
// Created by hanyunhai on 3/5/19.
//

#ifndef SLAM_PROJECT_FRAME_H
#define SLAM_PROJECT_FRAME_H
#include "myslam/commom_include.h"
#include "myslam/camera.h"
namespace myslam {
    class Frame {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_;
        double time_stamp_;
        SE3 T_c_w_; //记录位姿，即机器人移动
        Camera::Ptr camera_;
        Mat color_,depth_;
    public:
        Frame();
        Frame(long id,double time_stamp=0,SE3 T_c_w=SE3(),Camera::Ptr camera=nullptr,Mat color=Mat(),
                Mat depth=Mat());
        ~Frame();
        static Frame::Ptr createFrame();
        double findDepth(const cv::KeyPoint& kp);
        Vector3d getCamCenter() const;  //const 成员函数不能对成员进行修改
        bool isInFrame(const Vector3d& pt_world);
    };
}
#endif //SLAM_PROJECT_FRAME_H
