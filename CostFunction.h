//
// Created by chuguanyi on 18-3-21.
//

#pragma once
#define _USE_MATH_DEFINES
#include <ceres/ceres.h>
#include "types.h"
#include "CameraModel.h"
#include <opencv2/core/eigen.hpp>
#include <math.h>
#include <sophus/se3.hpp>
// A CostFunction implementing analytically derivatives for the
class ReprojectorCostFunction
	: public ceres::SizedCostFunction<2 /* number of residuals */,
	6 /* size of first parameter */> {
public:
	ReprojectorCostFunction(vec3 x, vec2 y, mat3 K) :X_(x), x_(y), K_(K) {};
	virtual ~ReprojectorCostFunction() {}

	virtual bool Evaluate(double const* const* parameters,
		double* residuals,
		double** jacobians) const {
		const double* t = &parameters[0][0];
		const double* r = &parameters[0][3];

		vec3 rotation(r);
		vec3 translation(t);
		mat3 R;
		Rodrigues(rotation, R);
		// f(x) = 10 - x.
		vec2 p = world2pixel(X_, R, translation, K_);
		residuals[0] = (p[0] - x_[0]);
		residuals[1] = (p[1] - x_[1]);
		double f = K_(0, 0);
		if (jacobians != NULL && jacobians[0] != NULL) {
			double fbz = f / X_[2];
			double fbzz = f / (X_[2] * X_[2]);
			Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > jac(jacobians[0]);
			jac(0, 0) = fbz;// f / Z
			jac(0, 1) = 0;
			jac(0, 2) = -fbzz * X_[0]; //-f * x /(Z^2)
			jac(0, 3) = -fbzz * X_[0] * X_[1]; // -f * X * Y / Z^2
			jac(0, 4) = f + fbzz * X_[0] * X_[0];//f + f * X^2/Z^2
			jac(0, 5) = -fbz * X_[1];// -f * Y / Z
			jac(1, 0) = 0;
			jac(1, 1) = fbz;// f / Z
			jac(1, 2) = -fbzz * X_[1]; //-f * Y / Z^2
			jac(1, 3) = -f - fbzz * X_[1] * X_[1];// -f - f*Y^2/Z^2
			jac(1, 4) = -fbzz * X_[0] * X_[1];// -f * X * Y / Z^2
			jac(1, 5) = -fbz * X_[0];//-f * X / Z
		}


		return true;
	}
private:
	const vec3 X_;
	const vec2 x_;
	const mat3 K_;
};
template<typename T>
inline T
interpolateMat(const cv::Mat& mat, double u, double v)
{
	double x = floor(u);
	double y = floor(v);
	double subpix_x = u - x;
	double subpix_y = v - y;
	T wx0 = 1.0 - subpix_x;
	T wx1 = subpix_x;
	T wy0 = 1.0 - subpix_y;
	T wy1 = subpix_y;
	T val00 = mat.at<T>(y, x);
	T val10 = mat.at<T>(y, x + 1);
	T val01 = mat.at<T>(y + 1, x);
	T val11 = mat.at<T>(y + 1, x + 1);
	return (wx0*wy0)*val00 + (wx1*wy0)*val10 + (wx0*wy1)*val01 + (wx1*wy1)*val11;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d&v)
{
	Eigen::Matrix3d m;
	m.fill(0.);
	m(0, 1) = -v(2);
	m(0, 2) = v(1);
	m(1, 2) = -v(0);
	m(1, 0) = v(2);
	m(2, 0) = -v(1);
	m(2, 1) = v(0);
	return m;
}

inline Eigen::Quaterniond toQuaterniond(const Eigen::Vector3d& v3d, double* angle = NULL)
{
	const double SMALL_EPS = 1e-10;
	double theta = v3d.norm();
	if (angle != NULL)
		*angle = theta;
	double half_theta = 0.5*theta;

	double imag_factor;
	double real_factor = cos(half_theta);
	if (theta<SMALL_EPS)
	{
		double theta_sq = theta * theta;
		double theta_po4 = theta_sq * theta_sq;
		imag_factor = 0.5 - 0.0208333*theta_sq + 0.000260417*theta_po4;
	}
	else
	{
		double sin_half_theta = sin(half_theta);
		imag_factor = sin_half_theta / theta;
	}

	return Eigen::Quaterniond(real_factor,
		imag_factor*v3d.x(),
		imag_factor*v3d.y(),
		imag_factor*v3d.z());
}

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


// A CostFunction implementing analytically derivatives for the
class DistanceXYZQCostFunction
	: public ceres::SizedCostFunction<1 /* number of residuals */,
	7 /* size of first parameter */> {
public:
	DistanceXYZQCostFunction(vec3 x, mat3 K, const cv::Mat& lv_set, cv::Mat& dt_dx, cv::Mat& dt_dy) :
		X_(x), K_(K), lv_set_(lv_set), dt_dx_(dt_dx), dt_dy_(dt_dy) {};
	virtual ~DistanceXYZQCostFunction() {}

	virtual bool Evaluate(double const* const* parameters,
		double* residuals,
		double** jacobians) const {
		const double* t = &parameters[0][0];
		const double* r = &parameters[0][3];

		vec3 translation(t);
		Eigen::Quaterniond rotation(r);
		mat3 R;
		cv::eigen2cv(rotation.matrix(), R);
		vec2 p = world2pixel(X_, R, translation, K_);
		vec3 X_c = R * X_ + translation;
		cv::Point pt(round(p[0]), round(p[1]));
		if (p[0] < 0 || p[1] < 0 || p[0] >= lv_set_.cols || p[1] >= lv_set_.rows)
			return false;
		int left = floor(p[0]);
		int right = ceil(p[0]);
		int top = floor(p[1]);
		int bottom = ceil(p[1]);
		double top_dt = lv_set_.at<float>(top, left) +
			(lv_set_.at<float>(top, right) - lv_set_.at<float>(top, left))*(p[0] - left);
		double btm_dt = lv_set_.at<float>(bottom, left) +
			(lv_set_.at<float>(bottom, right) - lv_set_.at<float>(bottom, left))*(p[0] - left);

		double Theta_x = top_dt + (btm_dt - top_dt) * (p[1] - top);
		//std::cout << Theta_x << std::endl;
		residuals[0] = Theta_x;
		Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jac_p;

		double f = K_(0, 0);
		Eigen::Matrix<double, 2, 7, Eigen::RowMajor> jac_lie;
		double fbz = f / X_c[2];
		double fbzz = f / (X_c[2] * X_c[2]);
		
		jac_p << fbz, 0, -fbzz * X_c[0],
				0, fbz, -fbzz * X_c[1];
		 
		Eigen::Matrix<double, 3, 7, Eigen::RowMajor> jac_p_q;
		double qx = rotation.x();
		double qy = rotation.y();
		double qz = rotation.z();
		double qr = rotation.w();
		double ax = X_c[0];
		double ay = X_c[1];
		double az = X_c[2];
		jac_p_q << 1, 0, 0, qy*ay + qz* az,          -2*qy*ax + qx*ay + qr*ax, -2*qz*ax - qr*ay + qx*az, -qz*ay + qy*az,
				0, 1, 0,    qy*ax - 2*qx*ay - qr*az, qx*ax + qz * az,          qr*ax - 2*qz*ay + qy*az,  qz*ax - qx*az,
				0, 0, 1,    qz*ax + qr*ay - 2*qx*az, -qr*ax + qz*ay - 2*qy*az, qx*ax + qy*ay,            -qy*ax + qx*ay;
		jac_lie = jac_p * 2 * jac_p_q;

		Eigen::Matrix<double, 1, 2, Eigen::RowMajor> jac_theta;
		jac_theta(0, 0) = interpolateMat<double>(dt_dx_, p[0], p[1]);
		jac_theta(0, 1) = interpolateMat<double>(dt_dy_, p[0], p[1]);
		if (jacobians != NULL && jacobians[0] != NULL) {
			Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jac(jacobians[0]);
			jac = jac_theta * jac_lie;
			//std::cout << X_ << Theta_x << std::endl;
			//std::cout << jac_theta << std::endl;
			//std::cout << jac << std::endl;
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