//
// Created by chuguanyi on 18-3-25.
//

#include "ModelWarpper.h"
#include "Render.h"

using namespace cv;

ModelWarpper::ModelWarpper()
{
}


ModelWarpper::~ModelWarpper()
{
	if (model_)
		delete model_;
}

void ModelWarpper::LoadObj(const std::string & filename)
{
	model_ = glmReadOBJ(filename.c_str());
	for (size_t i = 1; i <= model_->numvertices; i++)
	{
		vec3 v;
		v[0] = model_->vertices_[(i) * 3];
		v[1] = model_->vertices_[(i) * 3 + 1];
		v[2] = model_->vertices_[(i) * 3 + 2];

		vertices_.push_back(v);
	}

	for (size_t i = 0; i < model_->numLines; i++)
	{
		std::pair<int, int> aline;
		aline.first = model_->lines[i].vindices[0] - 1;
		aline.second = model_->lines[i].vindices[1] - 1;
		lines_.push_back(aline);
	}
}

void ModelWarpper::Init(const mat3 & K)
{
	instrincs_ = K;
	const char* renderName = "render";
	Matx34f intrinsic;
	intrinsic(0, 0) = K(0,0);    intrinsic(0, 1) = 0;             intrinsic(0, 2) = K(0,2); intrinsic(0, 3) = 0;
	intrinsic(1, 0) = 0;         intrinsic(1, 1) = K(1,1);		  intrinsic(1, 2) = K(1,2); intrinsic(1, 3) = 0;
	intrinsic(2, 0) = 0;         intrinsic(2, 1) = 0;             intrinsic(2, 2) = 1;		intrinsic(2, 3) = 0;

	render_.init((Mat)intrinsic,
		640,  
		480,
		0, (char**)&renderName);
}

void ModelWarpper::DisplayGL(const mat4 & pose, const int iLevel)
{
	render_.m_shapePoseInfo.clear();
	ShapePoseInfo shapePoseInfo;
	shapePoseInfo.m_shape = model_;
	Matx44f posef = pose;
	render_.matrixFromCV2GL((Mat)posef, shapePoseInfo.mv_matrix);
	render_.m_shapePoseInfo.push_back(shapePoseInfo);
	render_.rendering();
	render_.getDepthImg();
}

cv::Mat ModelWarpper::DrawOn(cv::Mat img, vec3 r, vec3 t, Scalar color)
{
	cv::Mat out = img.clone();
	std::vector<Point2d> img_pts;
	projectPoints(vertices_, r, t, instrincs_, noArray(), img_pts);
	for (size_t i = 0; i < lines_.size(); i+=5)
	{
		line(out, img_pts[lines_[i].first], img_pts[lines_[i].second], color);
	}
	return out;
}
