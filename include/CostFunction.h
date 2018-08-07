//
// Created by qqh on 18-8-7.
//

#ifndef ROBOT_COSTFUNCTION_H
#define ROBOT_COSTFUNCTION_H
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <math.h>
#include <sophus/se3.hpp>

#include "types.h"
#include "Utils.h"
using namespace Utils;
const double MAX_ENERGY = INFINITY;
class RegionPostCostFunction
        : public ceres::SizedCostFunction<1 /* number of residuals */,
                6 /* size of first parameter */> {
public:
    RegionPostCostFunction(vec3 x, mat3 K,const cv::Mat& lv_set,cv::Mat &post_front,cv::Mat &post_back) :
            X_(x),K_(K),lv_set_(lv_set),post_front_(post_front),post_back_(post_back){};
    virtual ~RegionPostCostFunction() {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
            const double* t = &parameters[0][0];
            const double* r = &parameters[0][3];

            Eigen::Vector3d Xw;
            Xw << X_[0], X_[1], X_[2];
            Eigen::Map<const Eigen::Vector3d> angleaxis(r);
            Eigen::Map<const Eigen::Vector3d> translation(t);
            Eigen::Quaterniond quat = toQuaterniond(angleaxis);

            Eigen::Vector3d X_ce = quat * Xw + translation;
            vec3 X_c(X_ce[0], X_ce[1], X_ce[2]);
            vec2 p = Project(X_c, K_);
            cv::Point pt(round(p[0]),round(p[1]));
            if (p[0] <= 0 || p[1] <= 0 || p[0] >= lv_set_.cols - 1 || p[1] >= lv_set_.rows - 1)
            {
                    if (jacobians != NULL && jacobians[0] != NULL)
                    {
                            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor> > jac(jacobians[0]);
                            jac.setZero();
                    }
                    residuals[0] = MAX_ENERGY;
                    return true;
            }
            double phi_x = lv_set_.at<float>(pt);
            double f_x_xi = -log(He(phi_x)*post_front_.at<float>(pt) + (1 - He(phi_x))*post_back_.at<float>(pt));
            residuals[0] = f_x_xi*f_x_xi*0.5;


            if (jacobians != NULL && jacobians[0] != NULL) {
                    //get jac_lie
                    double f = K_(0, 0);
                    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> jac_lie;
                    double fbz = f / X_c[2];
                    double fbzz = f / (X_c[2] * X_c[2]);
                    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jac_p;
                    jac_p << fbz, 0, -fbzz * X_c[0],
                            0, fbz, -fbzz * X_c[1];
                    Eigen::Vector3d P(X_c[0], X_c[1], X_c[2]);
                    jac_lie.block<2, 3>(0, 3) = -jac_p * skew(P);
                    jac_lie.block<2, 3>(0, 0) = jac_p;
                    //get jac_theta
                    Eigen::Matrix<double, 1, 2, Eigen::RowMajor> jac_theta;
                    jac_theta(0, 0) = (lv_set_.at<float>(p[1], p[0]+1) -  lv_set_.at<float>(p[1], p[0]-1))/2;
                    jac_theta(0, 1) = (lv_set_.at<float>(p[1]+1, p[0]) -  lv_set_.at<float>(p[1]-1, p[0]))/ 2;
                    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor> > jac(jacobians[0]);
                    jac =(post_back_.at<float>(pt)- post_front_.at<float>(pt))*(1.0/f_x_xi) *phi_x*delta(phi_x)*jac_theta * jac_lie ;

            }

            return true;
    }
private:
    const vec3 X_;
    const mat3 K_;
    const cv::Mat& lv_set_;
    const cv::Mat& post_front_,post_back_;
};
#endif //ROBOT_COSTFUNCTION_H
