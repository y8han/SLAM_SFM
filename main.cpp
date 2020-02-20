#include <iostream>
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/commom_include.h"
#include <Eigen/Core>
int main(int argc,char** argv) {
    if(argc!=2)
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    myslam::Config::setParameterFile(argv[1]);
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
    string dataset_dir=myslam::Config::get<string>("dataset_dir");
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin(dataset_dir+"/associate.txt");
    if(!fin)
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    vector<string> rgb_files,depth_files;
    vector<double> rgb_times,depth_times;
    while(!fin.eof())
    {
        string rgb_time,rgb_file,depth_time,depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(dataset_dir+"/"+rgb_file);
        depth_files.push_back(dataset_dir+"/"+depth_file);
        if(fin.good()==false)
            break;
    }
    myslam::Camera::Ptr camera(new myslam::Camera);

    cout<<"read total "<<rgb_files.size()<<endl;
    for(int i=0;i<rgb_files.size();i++)
    {
        Mat color=cv::imread(rgb_files[i]);
        Mat depth=cv::imread(depth_files[i],-1);
        if(color.data== nullptr||depth.data== nullptr)
            break;
        myslam::Frame::Ptr pFrame=myslam::Frame::createFrame(); //这是一个静态成员函数
        pFrame->camera_=camera; //每次都是一个已经存储相机内参的camera了
        pFrame->color_=color;
        pFrame->depth_=depth;
        pFrame->time_stamp_=rgb_times[i];
        boost::timer timer;
        vo->addFrame(pFrame); //addFrame里面计算了每次之间的移动，因此会比较耗时
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        //cout<<"ashdiahias"<<endl;
        if(vo->state_==myslam::VisualOdometry::LOST)
            break;
        SE3 Tcw=pFrame->T_c_w_.inverse();
        cv::Mat R=cv::Mat_<double>(3,3)<<(
                Tcw.rotation_matrix()(0,0),Tcw.rotation_matrix()(0,1),Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0),Tcw.rotation_matrix()(1,1),Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0),Tcw.rotation_matrix()(2,1),Tcw.rotation_matrix()(2,2)
                );
        cv::Mat t=cv::Mat_<double>(3,1)<<(Tcw.translation()(0,0),Tcw.translation()(1,0),Tcw.translation()(2,0));
        cout<<"results after optimization method: "<<endl;
        cout<<"R: "<<R<<endl;
        cout<<"t: "<<t<<endl;
        for(auto& pt:vo->map_->map_points_)
        {
            myslam::MapPoint::Ptr p=pt.second;
            Vector2d pixel=pFrame->camera_->world2pixel(p->pos_,pFrame->T_c_w_);
            cv::circle(color,cv::Point2f(pixel(0,0),pixel(1,0)),5,cv::Scalar(0,255,0),2);
        }
        cv::imshow("image",color);
        cv::waitKey(1);
    }
    return 0;
}
