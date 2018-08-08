//
// Created by qqh on 18-8-7.
//

#ifndef ROBOT_SOVER_H
#define ROBOT_SOVER_H

#include <opencv2/opencv.hpp>
#include <random>
#include <numeric>
#include <ceres/ceres.h>
#include "types.h"
#include "Post.h"

class Solver{
public:
    Solver();
    ~Solver();
    void init(double* pose)
    {
        for (size_t i = 0; i < p_num_; i++)
        {
            std::copy(pose, pose + 6, particle[i]);
        }
    }
    void Normalize(double* weight, int num)
    {
        double sum = std::accumulate(weight, weight + num, 0.0);
        for (size_t i = 0; i < num; i++)
        {
            weight[i] /= sum;
        }
    }
    void Optimize(double * pose,mat3 &K,Post &post,ModelWarpper &render);
    void Optimize(double * pose_parameter, std::vector<vec3> &support_points,
             mat3 &K, cv::Mat &distance_map);
private:
    void Resample(std::vector<double*> particle, double* weight, const int p_num);
private:

    ceres::Solver::Options options;

    int optimal_index_ = 0;
    static const int p_num_ = 3;
    std::vector<double*> particle;
    double weight_[p_num_];
    std::random_device seed_;
    std::default_random_engine e;
    std::normal_distribution<double> gassuia_distrubtion[6]{
            std::normal_distribution<double>(0,0.2), std::normal_distribution<double>(0,0.1), std::normal_distribution<double>(0,0.1),
            std::normal_distribution<double>(0,0.01), std::normal_distribution<double>(0,0.01), std::normal_distribution<double>(0,0.01)
    };

};

#endif //ROBOT_SOVER_H
