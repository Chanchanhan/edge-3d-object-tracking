//
// Created by qqh on 18-8-7.
//

#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

#include "glm.h"
#include "types.h"
#include "Render.h"


class ModelWarpper
{
public:
	ModelWarpper();
	~ModelWarpper();
	void LoadObj(const std::string &filename);
	void Init(const mat3& K,const int &x,const int &y);
	void DisplayGL(const mat4 &pose, const int iLevel = 0);
	cv::Mat DrawOn(cv::Mat img, vec3 r, vec3 t, cv::Scalar color);
	GLMmodel * model_;
	std::vector<vec3> vertices_;
	std::vector<vec3> boxes;
	std::vector<std::pair<int, int>> lines_;
	mat3 instrincs_;
	Render render_;
};

