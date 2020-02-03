//
// Created by hanyunhai on 3/5/19.
//

#ifndef SLAM_PROJECT_CONFIG_H
#define SLAM_PROJECT_CONFIG_H
#include "myslam/commom_include.h"
namespace myslam {
    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {}

    public:
        ~Config();

        static void setParameterFile(const std::string &filename);  //静态成员函数可以使用静态成员变量

        template<typename T>
        static T get(const std::string &key) {
            return T(Config::config_->file_[key]); //返回模板的类型
        }
    };
}
#endif //SLAM_PROJECT_CONFIG_H
