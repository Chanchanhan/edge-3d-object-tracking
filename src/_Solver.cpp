////
//// Created by chuguanyi on 18-3-21.
////
//
//
//#include "Solver.h"
//#include "Parameterization.h"
//#include "CostFunction.h"
//
//
//void Solver::Resample(std::vector<double*> particle, double* weight, const int p_num)
//{
//	std::vector<int> temp(p_num);
//	double acc = 0;
//	int k = 0;
//	std::vector<double*> res;
//	for (size_t i = 0; i < p_num&&k < p_num; )
//	{
//		if ((i + 1) / (double)p_num > acc)
//		{
//			acc += weight[k++];
//		}
//		else
//		{
//			double* temp = new double[6];
//			std::copy(particle[k - 1], particle[k - 1] + 6, temp);
//			res.push_back(temp);
//			i++;
//		}
//	}
//	while (res.size() < p_num)
//	{
//		double* temp = new double[6];
//		std::copy(particle[k - 1], particle[k - 1] + 6, temp);
//		res.push_back(temp);
//	}
//	if (res.size() > p_num)
//		res.resize(p_num);
//	for (size_t i = 0; i < p_num; i++)
//	{
//		std::copy(res[i], res[i] + 6, particle[i]);
//		delete[] res[i];
//	}
//
//}
//
//
//void Solver::Optimize(double* pose_parameter, std::vector<vec3> support_points,
//	mat3 K, cv::Mat distance_map)
//{
//	cv::Mat dt_dx;
//	cv::Mat dt_dy;
//	Sobel(distance_map, dt_dx, CV_64F, 1, 0);
//	Sobel(distance_map, dt_dy, CV_64F, 0, 1);
//	Resample(particle, weight_, p_num_);
//	for (int i = 0; i < p_num_; i++)
//	{
//		for (size_t j = 0; j < 6; j++)
//		{
//			particle[i][j] += gassuia_distrubtion[j](e);
//		}
//	}
//	double min_residual = HUGE_VAL;
//	optimal_index_ = -1;
//
//	for (size_t j = 0; j < p_num_; j++)
//	{
//		ceres::Problem ba;
//		ba.AddParameterBlock(particle[j], 6, new PoseSE3Parameterization);
//		for (size_t i = 0; i < support_points.size(); i++)
//		{
//			ba.AddResidualBlock(
//				new DistanceCostFunction(support_points[i], K, distance_map, dt_dx, dt_dy),
//				NULL,// new ceres::HuberLoss(10),
//				particle[j]);
//		}
//		//ba.SetParameterization(pose_parameter_tr,new  PoseSE3Parameterization);
//
//		ceres::Solver ba_solver;
//		ceres::Solver::Options options;
//		ceres::Solver::Summary summary;
//
//		options.max_num_iterations = 50;
//		options.linear_solver_type = ceres::DENSE_QR;
//		options.max_solver_time_in_seconds = 0.2;
//		options.min_trust_region_radius = 1e-8;
//		//options.minimizer_progress_to_stdout = true;
//		options.initial_trust_region_radius = 1e3;
//		options.num_threads = 1;
//		options.function_tolerance = 2e-3;
//		options.gradient_tolerance = 2e-7;
//		//options.trust_region_strategy_type = ceres::DOGLEG;
//
//		ba_solver.Solve(options, &ba, &summary);
//		weight_[j] = exp(-summary.final_cost / support_points.size());
//		if (summary.final_cost < min_residual)
//		{
//			min_residual = summary.final_cost;
//			optimal_index_ = j;
//		}
//	}
//	Normalize(weight_, p_num_);
//	if ((optimal_index_ < 0))
//	{
//		std::cout << "no optimal pose" << std::endl;
//	}
//	//std::cout << "optimal index is" << optimal_index_ << std::endl;
//	std::copy(particle[optimal_index_], particle[optimal_index_] + 6, pose_parameter);
//	//std::cout << summary.BriefReport() << "\n";
//}
//
//
//
