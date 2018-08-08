//
// Created by qqh on 18-8-7.
//
#include "Parameterization.h"
#include "Solver.h"
#include "CostFunction.h"
Solver::Solver() {
    for (int i = 0; i < p_num_; i++)
    {
        particle.emplace_back(new double[6]);
        weight_[i] = 1.0 / p_num_;
    }
    e = std::default_random_engine(seed_());


//    options.minimizer_type = ceres::LINE_SEARCH;
//    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//    options.minimizer_progress_to_stdout = false;
//    options.max_num_iterations = 25;
//    options.gradient_tolerance = 1e-15;
//    options.function_tolerance = 1e-15;


    options.max_num_iterations = 50;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_solver_time_in_seconds = 0.2;
    options.min_trust_region_radius = 1e-8;
    //options.minimizer_progress_to_stdout = true;
    options.initial_trust_region_radius = 1e3;
    options.num_threads = 1;
    options.function_tolerance = 2e-3;
    options.gradient_tolerance = 2e-7;
}

Solver::~Solver()
{
    for (size_t i = 0; i < p_num_; i++)
    {
        delete[] particle[i];
    }
}

void Solver::Resample(std::vector<double*> particle, double* weight, const int p_num)
{
    std::vector<int> temp(p_num);
    double acc = 0;
    int k = 0;
    std::vector<double*> res;
    for (size_t i = 0; i < p_num&&k < p_num; )
    {
        if ((i + 1) / (double)p_num > acc)
        {
            acc += weight[k++];
        }
        else
        {
            double* temp = new double[6];
            std::copy(particle[k - 1], particle[k - 1] + 6, temp);
            res.push_back(temp);
            i++;
        }
    }
    while (res.size() < p_num)
    {
        double* temp = new double[6];
        std::copy(particle[k - 1], particle[k - 1] + 6, temp);
        res.push_back(temp);
    }
    if (res.size() > p_num)
        res.resize(p_num);
    for (size_t i = 0; i < p_num; i++)
    {
        std::copy(res[i], res[i] + 6, particle[i]);
        delete[] res[i];
    }

}

void Solver::Optimize(double* pose_parameter, std::vector<vec3> &support_points,
                      mat3 &K, cv::Mat &distance_map)
{
    cv::Mat dt_dx;
    cv::Mat dt_dy;
    Sobel(distance_map, dt_dx, CV_64F, 1, 0);
    Sobel(distance_map, dt_dy, CV_64F, 0, 1);
    Resample(particle, weight_, p_num_);
    for (int i = 0; i < p_num_; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            particle[i][j] += gassuia_distrubtion[j](e);
        }
    }
    double min_residual = HUGE_VAL;
    optimal_index_ = -1;

    for (size_t j = 0; j < p_num_; j++)
    {
        ceres::Problem ba;
        ba.AddParameterBlock(particle[j], 6, new PoseSE3Parameterization);
        for (size_t i = 0; i < support_points.size(); i++)
        {
            ba.AddResidualBlock(
                    new DistanceCostFunction(support_points[i], K, distance_map, dt_dx, dt_dy),
                    NULL,// new ceres::HuberLoss(10),
                    particle[j]);
        }
        //ba.SetParameterization(pose_parameter_tr,new  PoseSE3Parameterization);

        ceres::Solver ba_solver;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.max_num_iterations = 50;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_solver_time_in_seconds = 0.2;
        options.min_trust_region_radius = 1e-8;
        //options.minimizer_progress_to_stdout = true;
        options.initial_trust_region_radius = 1e3;
        options.num_threads = 1;
        options.function_tolerance = 2e-3;
        options.gradient_tolerance = 2e-7;
        //options.trust_region_strategy_type = ceres::DOGLEG;

        ba_solver.Solve(options, &ba, &summary);
        weight_[j] = exp(-summary.final_cost / support_points.size());
        if (summary.final_cost < min_residual)
        {
            min_residual = summary.final_cost;
            optimal_index_ = j;
        }
        std::cout << summary.BriefReport() << "\n";

    }
    Normalize(weight_, p_num_);
    if ((optimal_index_ < 0))
    {
        std::cout << "no optimal pose" << std::endl;
    }
    //std::cout << "optimal index is" << optimal_index_ << std::endl;
    std::copy(particle[optimal_index_], particle[optimal_index_] + 6, pose_parameter);
}
void Solver::Optimize(double  * pose ,mat3 &K, Post &post,
                      ModelWarpper &render) {

    cv::Mat dt_dx;
    cv::Mat dt_dy;
    Resample(particle, weight_, p_num_);
    for (int i = 0; i < p_num_; i++)
    {
        for (size_t j = 0; j < 6; j++)
        {
            particle[i][j] += gassuia_distrubtion[j](e);
        }
    }
    std::vector<vec3> samplePoints;
    auto rendered0 = render.render_.getRenderedImg();
    Utils::SampleVertices(samplePoints,rendered0,render);

    double min_residual = HUGE_VAL;
    optimal_index_ = -1;

    for (size_t j = 0; j < p_num_; j++)
    {
        ceres::Problem ba;
        ba.AddParameterBlock(particle[j], 6, new PoseSE3Parameterization);
        for (auto &Xi:samplePoints)
        {
            ba.AddResidualBlock(new RegionPostCostFunction(Xi,K,post.lv_set,post.forthMap,post.backMap),
                                NULL, particle[j]);
        }
//        ba.SetParameterization(particle[j],new  PoseSE3Parameterization);

        ceres::Solver ba_solver;
        ceres::Solver::Summary summary;

        //options.trust_region_strategy_type = ceres::DOGLEG;

        ba_solver.Solve(options, &ba, &summary);
        weight_[j] = exp(-summary.final_cost / samplePoints.size());
        if (summary.final_cost < min_residual)
        {
            min_residual = summary.final_cost;
            optimal_index_ = j;
        }
        std::cout << summary.BriefReport() << "\n";

    }
    Normalize(weight_, p_num_);
    if ((optimal_index_ < 0))
    {
        std::cout << "no optimal pose" << std::endl;
    }
    //std::cout << "optimal index is" << optimal_index_ << std::endl;
    else std::copy(particle[optimal_index_], particle[optimal_index_] + 6, pose);

//    ceres::Problem minEnergyProblem;
//    ceres::Solver::Summary summary;
//    double* pose_var = new double[6];
//    memcpy(pose_var,pose, sizeof(double)*6);
//    minEnergyProblem.AddParameterBlock(pose_var, 6,new PoseSE3Parameterization);
//
//    for(int i=0;i<4;i++){
//        std::vector<vec3> samplePoints;
//        auto rendered0 = render.render_.getRenderedImg();
//
//        Utils::SampleVertices(samplePoints,rendered0,render);
//        for (auto &Xi:samplePoints) {
//            minEnergyProblem.AddResidualBlock(new RegionPostCostFunction(Xi,K,post.lv_set,post.forthMap,post.backMap),
//                                              NULL, pose_var);
//        }
//
//        ceres::Solve(options, &minEnergyProblem, &summary);
//
//        vec3 ti(pose_var);
//        vec3 ri(pose_var + 3);
//        mat4 T = Utils::GetTranformation(ri, ti);
//        render.DisplayGL(T);
//        auto rendered = render.render_.getRenderedImg();
//        auto mask = rendered > 1;
//        std::cout<<summary.BriefReport()<<std::endl;
//        post.UpdateDT(mask);
//    }
//    memcpy(pose,pose_var , sizeof(double)*6);
//
//    LOG(INFO)<<summary.BriefReport();
}