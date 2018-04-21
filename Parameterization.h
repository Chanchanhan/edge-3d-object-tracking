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
	//Quaterniond quaterd_plus = se3_delta.rotation() * toQuaterniond(Eigen::Map<const Vector3d>(x));
	//Eigen::Map<Vector3d> angles_plus(x_plus_delta);
	//angles_plus = toAngleAxis(quaterd_plus);

	//Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 3);
	//trans_plus = se3_delta.rotation() * trans + se3_delta.translation();
	return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
	Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J(jacobian);
	J.setIdentity();
	return true;
}

class XYZQuatParameterization : public ceres::LocalParameterization {
public:
	XYZQuatParameterization() {}
	virtual ~XYZQuatParameterization() {}
	virtual bool Plus(const double* x,
		const double* delta,
		double* x_plus_delta) const;
	virtual bool ComputeJacobian(const double* x,
		double* jacobian) const;
	virtual int GlobalSize() const { return 7; }
	virtual int LocalSize() const { return 7; }
};

//here the real part varies from sophus and eigen and ceres
bool XYZQuatParameterization::Plus(const double* x,
	const double* delta,
	double* x_plus_delta) const
{
	Eigen::Map<const Eigen::Vector3d> translation(x);
	Eigen::Map<const Eigen::Quaterniond> rotation(x + 3);
	Eigen::Map<Eigen::Vector3d> updated_translation(x_plus_delta);
	Eigen::Map<Eigen::Quaterniond> updated_rotaion(x_plus_delta + 3);
	Eigen::Map<const Eigen::Vector3d> delta_translation(delta);
	Eigen::Map<const Eigen::Quaterniond> delta_rotation(delta + 3);

	updated_rotaion = delta_rotation * rotation;
	updated_rotaion.normalize();
	updated_translation = delta_rotation * translation + delta_translation;
	std::cout << Eigen::Quaterniond(x_plus_delta + 3).coeffs() << std::endl;
	return true;
}

bool XYZQuatParameterization::ComputeJacobian(const double* x,
	double* jacobian) const
{
	Eigen::Map<Eigen::Matrix<double, 7, 7, Eigen::RowMajor> > J(jacobian);
	J.setIdentity();
	return true;
}

//class PoseSE3AndDTParameterization : public ceres::LocalParameterization {
//public:
//	PoseSE3AndDTParameterization() {}
//	virtual ~PoseSE3AndDTParameterization() {}
//	virtual bool Plus(const double* x,
//		const double* delta,
//		double* x_plus_delta) const;
//	virtual bool ComputeJacobian(const double* x,
//		double* jacobian) const;
//	virtual int GlobalSize() const { return 6; }
//	virtual int LocalSize() const { return 6; }
//private:
//
//};
//
//bool PoseSE3AndDTParameterization::Plus(const double * x, const double * delta, double * x_plus_delta) const
//{
//
//	return false;
//}
//
//inline bool PoseSE3AndDTParameterization::ComputeJacobian(const double * x, double * jacobian) const
//{
//	Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J(jacobian);
//	J.setIdentity();
//	return true;
//}