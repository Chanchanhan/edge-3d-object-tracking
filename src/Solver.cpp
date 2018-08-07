//
// Created by qqh on 18-8-7.
//
#include "Parameterization.h"
#include "Solver.h"
#include "CostFunction.h"
Solver::Solver() {
    options.minimizer_type = ceres::LINE_SEARCH;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 25;
    options.gradient_tolerance = 1e-15;
    options.function_tolerance = 1e-15;
}


void Solver::Optimize(double  * pose ,mat3 &K, Post &post,
                      ModelWarpper &render) {
    ceres::Problem minEnergyProblem;
    ceres::Solver::Summary summary;
    double* pose_var = new double[6];
    memcpy(pose_var,pose, sizeof(double)*6);
    minEnergyProblem.AddParameterBlock(pose_var, 6,new PoseSE3Parameterization);

    for(int i=0;i<4;i++){
        std::vector<vec3> samplePoints;
        auto rendered0 = render.render_.getRenderedImg();

        Utils::SampleVertices(samplePoints,rendered0,render);
        for (auto &Xi:samplePoints) {
            minEnergyProblem.AddResidualBlock(new RegionPostCostFunction(Xi,K,post.lv_set,post.forthMap,post.backMap),
                                              NULL, pose_var);
        }

        ceres::Solve(options, &minEnergyProblem, &summary);

        vec3 ti(pose_var);
        vec3 ri(pose_var + 3);
        mat4 T = Utils::GetTranformation(ri, ti);
        render.DisplayGL(T);
        auto rendered = render.render_.getRenderedImg();
        auto mask = rendered > 1;
        std::cout<<summary.BriefReport()<<std::endl;
        post.UpdateDT(mask);
    }
    memcpy(pose,pose_var , sizeof(double)*6);

    LOG(INFO)<<summary.BriefReport();
}