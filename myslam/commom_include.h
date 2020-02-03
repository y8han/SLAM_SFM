//
// Created by hanyunhai on 3/5/19.
//

#ifndef SLAM_PROJECT_COMMOM_INCLUDE_H
#define SLAM_PROJECT_COMMOM_INCLUDE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;
// for cv
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using cv::Mat;
using cv::KeyPoint;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;
#endif //SLAM_PROJECT_COMMOM_INCLUDE_H
