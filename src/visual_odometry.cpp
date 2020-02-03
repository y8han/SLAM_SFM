#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include "myslam/g2o_types.h"
namespace myslam
{
    VisualOdometry::VisualOdometry()
    :state_(INITIALIZING),ref_(nullptr),curr_(nullptr),map_(new Map), num_lost_(0),num_inliers_(0),matcher_flann_(new cv::flann::LshIndexParams(5,10,2)){
        num_of_features_ = Config::get<int>("number_of_features");
        scale_factor_=Config::get<double>("scale_factor");
        level_pyramid_=Config::get<int> ("level_pyramid");
        match_ratio_ =Config::get<float>("match_ratio");
        min_inliers_=Config::get<int>("min_inliers");
        max_num_lost_=Config::get<float>("max_num_lost");
        key_frame_min_rot=Config::get<double>("keyframe_rotation");
        key_frame_min_trans=Config::get<double>("keyframe_translation");
        //orb_=cv::ORB::create(num_of_features_,scale_factor_,level_pyramid_,31,0,2,cv::ORB::HARRIS_SCORE,31,20);
        map_point_erase_ratio_=Config::get<double>("map_point_erase_ratio");
        orb_ = cv::ORB::create ( num_of_features_,scale_factor_, level_pyramid_ );
    }
    VisualOdometry::~VisualOdometry() {

    }
    bool VisualOdometry::addFrame(Frame::Ptr frame)
    {
        switch(state_)
        {
            case INITIALIZING:
            {
                state_=OK;
                curr_=ref_=frame;
               // map_->insertKeyFrame(frame);
                extractKeyPoints();  //第一帧的时候，提取关键角点
                computeDescriptors(); //计算每个角点的描述子
               // setRef3DPoints(); //把每个角点的三维坐标计算出来
                addKeyFrame();
                break;
            }
            case OK:
            {
                curr_=frame;
                curr_->T_c_w_=ref_->T_c_w_;  //认为前后两帧应该比较接近，用来初始化此帧的位姿，在判断地图中的点是否在此帧中需要使用到
                extractKeyPoints();
                computeDescriptors();
                featureMatching();
                poseEstimationPnP();
                if(checkEstimatedPose()==true)
                {
                    curr_->T_c_w_=T_c_w_estimated_; //得到当前帧相对于世界系
                    optimizeMap();
                    num_lost_=0;
                    if(checkKeyFrame()== true)
                        addKeyFrame();
                } else
                {
                    num_lost_++;//连续丢失达到一定的次数
                    if(num_lost_>max_num_lost_)
                        state_=LOST;
                    return false;
                }
                break;
            }
            case LOST:
            {
                cout<<"vo has lost."<<endl;
                break;
            }
        }
        return true;
    }
    void VisualOdometry::extractKeyPoints() {
        boost::timer timer;
        orb_->detect(curr_->color_,keypoints_curr_);
        cout<<"extract keypoints cost time: "<<timer.elapsed()<<endl;
    }
    void VisualOdometry::computeDescriptors() {
        boost::timer timer;
        orb_->compute(curr_->color_,keypoints_curr_,descriptors_curr_);
        cout<<"descriptor computation cost time: "<<timer.elapsed()<<endl;
    }
    void VisualOdometry::featureMatching() {
        boost::timer timer;
        vector<cv::DMatch> matches;
        //select the candidates in map
        Mat desp_map;
        vector<MapPoint::Ptr> candidate;
        for(auto& allpoints: map_->map_points_)
        {
            MapPoint::Ptr& p=allpoints.second;
            if(curr_->isInFrame(p->pos_))
            {
                p->observed_times_++;
                candidate.push_back(p);
                desp_map.push_back(p->descriptor_);
            }
        }
        matcher_flann_.match(desp_map,descriptors_curr_,matches); //用还在地图中的点和此帧提取的特征点进行匹配。
        float min_dis=std::min_element(matches.begin(),matches.end(),[](const cv::DMatch& m1,const cv::DMatch& m2){return m1.distance<m2.distance;})->distance;
        match_3dpts_.clear();
        match_2dkp_index_.clear();
        for(cv::DMatch& m:matches)
        {
            if(m.distance<max<float>(min_dis*match_ratio_,30.0))  //匹配一致的点，用来计算位姿用
             {
                match_3dpts_.push_back(candidate[m.queryIdx]);
                match_2dkp_index_.push_back(m.trainIdx); //记录的是keypoints_curr的序号
             }
        }
        cout<<"good matches: "<<match_3dpts_.size()<<endl;
        cout<<"match cost time: "<<timer.elapsed()<<endl;
    }
//    void VisualOdometry::setRef3DPoints() {//在最终版本中没使用这个函数
//        pts_3d_ref_.clear();
//        descriptors_ref_=Mat();
//        for(int i=0;i<keypoints_curr_.size();i++)
//        {
//            double d=ref_->findDepth(keypoints_curr_[i]); //这里的话，每一帧都是使用的现成的深度相机得到的数据。
//            if(d>0)
//            {
//                Vector3d p_cam=ref_->camera_->pixel2camera(
//                        Vector2d(keypoints_curr_[i].pt.x,keypoints_curr_[i].pt.y),d
//                        );
//                pts_3d_ref_.push_back(cv::Point3d(p_cam(0,0),p_cam(1,0),p_cam(2,0)));
//                descriptors_ref_.push_back(descriptors_curr_.row(i)); //
//            }
//        }
//    }
    void VisualOdometry::poseEstimationPnP() { //通过匹配点计算运动位姿
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;
        cout<<"size of features: "<<match_2dkp_index_.size()<<endl;
        for(int index:match_2dkp_index_)
        {
            pts2d.push_back(keypoints_curr_[index].pt);
        }
        for(MapPoint::Ptr pt:match_3dpts_)
            pts3d.push_back(pt->getPositionCV());
        Mat K=(cv::Mat_<double>(3,3)<<
                ref_->camera_->fx_,0,ref_->camera_->cx_,
                0,ref_->camera_->fy_,ref_->camera_->cy_,
                0,0,1);
        Mat rvec,tvec,inliers;
        cv::solvePnPRansac(pts3d,pts2d,K,Mat(),rvec,tvec,false,100,4.0,0.99,inliers); 
        //这里的异常数据应该是深度信息有误，导致计算出来的位姿矩阵比较奇怪
        //这里的100,4.0,0.99应该是RANSAC算法需要的迭代参数
        //因此求出来的位姿矩阵是 当前帧相对参考系(上一帧)的，即运动矩阵
        //这里已经不是上面所说的前三代版本了，这里记录的地图坐标就是世界系里的
        //即这里已经建了一个地图（局部），求得运动矩阵就是相对于世界系的
        num_inliers_=inliers.rows;
        cout<<"pnp inliers: "<<num_inliers_<<endl;
        
        T_c_w_estimated_=SE3(
                SO3(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)),
                Vector3d(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0))
                ); //当前帧相对于参考系（世界系）
        cout<<"results before optimization method: "<<endl;
        cout<<T_c_w_estimated_.inverse().rotation_matrix()<<endl;
        cv::Mat t=cv::Mat_<double>(3,1)<<(T_c_w_estimated_.inverse().translation()(0,0),T_c_w_estimated_.inverse().translation()(1,0),T_c_w_estimated_.inverse().translation()(2,0));
        cout<<t<<endl;
        boost::timer timer;
        //下面是使用图优化对RANSAC算法求得的位姿进行优化
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2> > Block;
        Block::LinearSolverType* linearSolver=new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr=new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
        g2o::OptimizationAlgorithmLevenberg* solver=new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        g2o::VertexSE3Expmap* pose=new g2o::VertexSE3Expmap();
        pose->setId(0);
        pose->setEstimate(g2o::SE3Quat(T_c_w_estimated_.rotation_matrix(),T_c_w_estimated_.translation()));
        optimizer.addVertex(pose);
        for(int i=0;i<inliers.rows;i++)
        {
            int index=inliers.at<int>(i,0); //RANSAC算法把异常值outlier都剔除了，因此这里的inliers是记录的原先的匹配点对中数据正确的点对
            //因为异常数据极大地影响优化算法，因此需要先剔除异常数据
            //(i,0)记录的是正确的点对在原先所有数据中的序列号
            EdgeProjectXYZ2UVPoseOnly* edge=new EdgeProjectXYZ2UVPoseOnly();
            edge->setId(i);
            edge->setVertex(0,pose);
            edge->camera_=curr_->camera_.get();
            edge->point_=Vector3d(pts3d[index].x,pts3d[index].y,pts3d[index].z);
            edge->setMeasurement(Vector2d(pts2d[index].x,pts2d[index].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
            match_3dpts_[index]->correct_times_++;
        }
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        T_c_w_estimated_=SE3(
                    pose->estimate().rotation(),
                    pose->estimate().translation());
        cout<<"solvePnP cost time: "<<timer.elapsed()<<endl;
    }
    bool VisualOdometry::checkKeyFrame() {  //这里是根据两帧(参考帧（上一个关键帧）和此帧)之间的运动变化是否足够大来区分此帧是否规划到关键帧里
        SE3 T_r_c=ref_->T_c_w_*T_c_w_estimated_.inverse();
        Sophus::Vector6d d=T_r_c.log();
        Vector3d trans=d.head<3>();
        Vector3d rot=d.tail<3>();
        if(rot.norm()>key_frame_min_rot||trans.norm()>key_frame_min_trans)
            //根据数据测得的大于设定的两帧之间的最小运动改变量
            return true;
        return false;
    }
    void VisualOdometry::addKeyFrame() {
        if(map_->keyframes_.empty()) //first key-frame, add all 3d points into map
        {
            for(int i=0;i<keypoints_curr_.size();i++)
            {
                double d=curr_->findDepth(keypoints_curr_[i]);
                if(d<0)
                    continue;
                Vector3d p_world=ref_->camera_->pixel2world(
                            Vector2d(keypoints_curr_[i].pt.x,keypoints_curr_[i].pt.y),curr_->T_c_w_,d);
                Vector3d n = p_world - ref_->getCamCenter();
                n.normalize();
                MapPoint::Ptr map_point=MapPoint::createMapPoint(
                        p_world,n,descriptors_curr_.row(i).clone(),curr_.get()
                    );//information of each 3d points
                map_->insertMapPoint(map_point);
            }
        }
        map_->insertKeyFrame(curr_);
        ref_=curr_;
    }
    bool VisualOdometry::checkEstimatedPose()
    {
        if(num_inliers_<min_inliers_)
        {
            cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
            return false;
        }
        SE3 T_r_c=ref_->T_c_w_*T_c_w_estimated_.inverse();//这里是上一帧的世界坐标系和这一帧的世界坐标系,这样操作以后得到的是相对运动
        Sophus::Vector6d d=T_r_c.log();
        if(d.norm()>5.0)
        {
            cout<<"reject because motion is too large: "<<d.norm()<<endl;
            return false;
        }
        return true;
    }
    void VisualOdometry::addMapPoints()
    {
        vector<bool> matched(keypoints_curr_.size(),false); //这里的vector的初始化是 生成size个数的false填入
        for(int index:match_2dkp_index_)
            matched[index]=true;  //将当前帧里的特征点全部加到地图里去，因为地图中的点少了，这里true的意思是这些特征点已经在地图里了
        for(int i=0;i<keypoints_curr_.size();i++)
        {
            if(matched[i]==true)
                continue;
            double d=ref_->findDepth(keypoints_curr_[i]);
            if(d<0)
                continue;
            Vector3d p_world=ref_->camera_->pixel2world(
                        Vector2d(keypoints_curr_[i].pt.x,keypoints_curr_[i].pt.y),
                        curr_->T_c_w_,d
                    );
            Vector3d n=p_world-ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point=MapPoint::createMapPoint(
                        p_world,n,descriptors_curr_.row(i).clone(),curr_.get()
                    );
            map_->insertMapPoint(map_point);
        }
    }
    void VisualOdometry::optimizeMap()
    {
        for(auto iter=map_->map_points_.begin();iter!=map_->map_points_.end();)
        {
            if(!curr_->isInFrame(iter->second->pos_))
            {
                iter=map_->map_points_.erase(iter);
                continue;
            }
            float match_ratio=float(iter->second->correct_times_)/iter->second->observed_times_;
            if(match_ratio<map_point_erase_ratio_)
            {
                iter=map_->map_points_.erase(iter);
                continue;
            }
            double angle=getViewAngle(curr_,iter->second);
            if(angle>M_PI/6.)
            {
                iter=map_->map_points_.erase(iter);
                continue;
            }
            if(iter->second->good_==false)
            {
            
            }
            iter++;
        }
        if(match_2dkp_index_.size()<100)
            addMapPoints(); //太少了,将此帧的点加入到地图中
        if(map_->map_points_.size()>1000)
        {
            map_point_erase_ratio_+=0.05;  //会更容易剔除点
        }
        else
            map_point_erase_ratio_=0.1;
        cout<<"map points: "<<map_->map_points_.size()<<endl;
    }
    double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point)
    {
        Vector3d n=point->pos_-frame->getCamCenter();
        n.normalize();
        return acos(n.transpose()*point->norm_);
    }
};
// Created by hanyunhai on 3/9/19.
//
