#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H
#include "myslam/commom_include.h"
#include "myslam/camera.h"
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/types/sba/types_six_dof_expmap.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<g2o/core/robust_kernel.h>
#include<g2o/core/robust_kernel_impl.h>
namespace myslam
{
//目标变量是3维的，待优化的变量有两个分别是点的坐标和位姿
class EdgeProjectXYZRGBD:public g2o::BaseBinaryEdge<3,Eigen::Vector3d,g2o::VertexSBAPointXYZ,g2o::VertexSE3Expmap> //对位姿和特征点三维坐标一起更新
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& out) const {}
};
class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3,Eigen::Vector3d,g2o::VertexSE3Expmap>  //只对位姿态更新
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& out) const {}

    Vector3d point_;
};
class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2,Eigen::Vector2d,g2o::VertexSE3Expmap>  //重投影的方法对位姿进行更新
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read(std::istream& in){}
    virtual bool write(std::ostream& os) const {}
    Vector3d point_;
    Camera* camera_;
};
}
#endif
