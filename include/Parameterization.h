//
// Created by chuguanyi on 18-3-25.
//
#pragma once
#include <ceres/ceres.h>
#include <sophus/se3.hpp>

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
	PoseSE3Parameterization() {}
	virtual ~PoseSE3Parameterization() {}
	virtual bool Plus(const double* x,
		const double* delta,
		double* x_plus_delta) const;
	virtual bool ComputeJacobian(const double* x,
		double* jacobian) const;
	virtual int GlobalSize() const { return 6; }
	virtual int LocalSize() const { return 6; }
};


bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
	using namespace Sophus;
	using namespace Eigen;
	Map<const Vector3d> origin(x + 3);

	SE3d se3_delta = SE3d::exp(Eigen::Map<const Vector6d>(delta));
	std::fill(x_plus_delta, x_plus_delta + 6, 0);
	Map<Vector3d> pose(x_plus_delta + 3);
	SO3d pose_se3 = SO3d::exp(pose);
	pose_se3 = se3_delta.so3() * SO3d::exp(origin);
	pose = pose_se3.log();
	Map<const Vector3d> translation(x);
	Map<Vector3d> translation_new(x_plus_delta);
	translation_new = se3_delta.so3() * translation + se3_delta.translation();
	return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
	Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J(jacobian);
	J.setIdentity();
	return true;
}
