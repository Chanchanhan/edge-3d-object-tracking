//
// Created by qqh on 18-8-7.
//

#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include "types.h"
#include "ModelWarpper.h"
namespace Utils{
    double He(const double &phi);
    double delta(const double &phi);
    mat4 GetTranformation(vec3 r, vec3 t);
    Eigen::Quaterniond toQuaterniond(const Eigen::Vector3d& v3d, double* angle = NULL);

    bool SampleVertices(std::vector<vec3>& support_points, cv::Mat &rendered, ModelWarpper& cube);
    Eigen::Matrix3d skew(const Eigen::Vector3d&v);
    vec3 world2camera(vec4 Xw, mat4 T);
    vec2 Project(vec3 pt, mat3 K);
    vec2 world2pixel(vec4 Xw, mat4 T, mat3 K);
    vec2 world2pixel(vec3 Xw, mat3 R, vec3 t, mat3 K);
    void BresehanCircle(const vec2 &vec,int radius,std::vector<vec2> &sampleVecs);
}


#endif //ROBOT_UTILS_H
