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

// A CostFunction implementing analytically derivatives for the
class DistanceCostFunction
        : public ceres::SizedCostFunction<1 /* number of residuals */,
                6 /* size of first parameter */> {
public:
    DistanceCostFunction(vec3 x, mat3 K,const cv::Mat& lv_set,cv::Mat& dt_dx,cv::Mat& dt_dy) :
            X_(x),K_(K),lv_set_(lv_set), dt_dx_(dt_dx),dt_dy_(dt_dy){};
    virtual ~DistanceCostFunction() {}

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
            residuals[0] = 10000000;
            //std::cout << "point out of range!" << std::endl;
            return true;
        }
        int x = floor(p[0]);
        int y = floor(p[1]);
        double subpix_x = p[0] - x;
        double subpix_y = p[1] - y;
        double wx0 = 1.0 - subpix_x;
        double wx1 = subpix_x;
        double wy0 = 1.0 - subpix_y;
        double wy1 = subpix_y;
        int bottom = ceil(p[1]);

        double val00 = lv_set_.at<float>(y, x);
        double val10 = lv_set_.at<float>(y, x + 1);
        double val01 = lv_set_.at<float>(y + 1, x);
        double val11 = lv_set_.at<float>(y + 1, x + 1);
        double w00 = (wx0*wy0);
        double w10 = (wx1*wy0);
        double w01 = (wx0*wy1);
        double w11 = (wx1*wy1);
        double Theta_x = w00 * val00 + w10 * val10 + w01 * val01 + w11 * val11;
        //std::cout << Theta_x << std::endl;
        double z = parameters[0][2]/50;//emmmm here should be adjustable
        residuals[0] = Theta_x;// +z;
        if (jacobians != NULL && jacobians[0] != NULL) {

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

            Eigen::Matrix<double, 1, 2, Eigen::RowMajor> jac_theta;
            val00 = dt_dx_.at<double>(y, x);
            val10 = dt_dx_.at<double>(y, x + 1);
            val01 = dt_dx_.at<double>(y + 1, x);
            val11 = dt_dx_.at<double>(y + 1, x + 1);
            jac_theta(0, 0) = w00 * val00 + w10 * val10 + w01 * val01 + w11 * val11;
            val00 = dt_dy_.at<double>(y, x);
            val10 = dt_dy_.at<double>(y, x + 1);
            val01 = dt_dy_.at<double>(y + 1, x);
            val11 = dt_dy_.at<double>(y + 1, x + 1);
            jac_theta(0, 1) = w00 * val00 + w10 * val10 + w01 * val01 + w11 * val11;
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor> > jac(jacobians[0]);
            Eigen::Matrix<double, 1, 6, Eigen::RowMajor> jac_z;
            jac_z.setZero();
            jac_z(2) = 0; 0.1;
            jac = jac_theta * jac_lie + jac_z;
        }

        return true;
    }
private:
    const vec3 X_;
    const mat3 K_;
    const cv::Mat& lv_set_;
    const cv::Mat& dt_dx_;
    const cv::Mat& dt_dy_;

};
#endif //ROBOT_COSTFUNCTION_H
