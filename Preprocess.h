//
// Created by chuguanyi on 18-3-25.
//

#pragma once
#include <opencv2/opencv.hpp>
#include "Histogram.h"

void Preprocess(const cv::Mat& frame, cv::Mat& distance_map)
{
	using namespace cv;
	using namespace std;
	cv::Mat countour_map = Mat::zeros(frame.size(),CV_8U);
	vector<vector<cv::Point>> countours;
	cv::findContours(frame, countours, 0, 1);
	drawContours(countour_map, countours, -1, Scalar(255, 255, 255));
	cv::distanceTransform(~countour_map, distance_map, cv::DIST_L2, 3);
}

inline mat4 GetTranformation(vec3 r, vec3 t)
{
	mat4 T = mat4::zeros();
	mat3 rotation;
	Rodrigues(r, rotation);
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			T(i, j) = rotation(i, j);
		}
	}
	T(0, 3) = t(0);
	T(1, 3) = t(1);
	T(2, 3) = t(2);
	T(3, 3) = 1;
	return T;
}

bool SampleVertices(std::vector<vec3>& support_points, cv::Mat rendered, ModelWarpper& cube)
{
	std::vector<std::vector<cv::Point> > rendered_countours;
	findContours(rendered, rendered_countours, 0, 1);
	if (rendered_countours.empty())
		return false;
	const int sample_num = std::min((int)rendered_countours[0].size(), 100);
	for (size_t i = 0; i < sample_num; i++)
	{
		auto sample = (rendered_countours[0][i * rendered_countours[0].size() / sample_num]);
		support_points.push_back((cv::Vec3f)cube.render_.get3DPos(sample.x, sample.y));
	}
	return true;
}


cv::Mat SegmentByHistogram(const cv::Mat& frame, const Histogram& f_post, const Histogram& b_post)
{
	using namespace cv;
	Mat foreth = Mat::zeros(frame.size(), CV_8U);
	auto p_foreth = foreth.ptr<uchar>(0);
	auto p = frame.ptr<Vec3b>(0);
	for (size_t i = 0; i < frame.total(); i++)
	{
		if (f_post(p[i][0], p[i][1], p[i][2]) > b_post[p[i][0], p[i][1], p[i][2]])
		{
			p_foreth[i] = 255;
		}
	}
	medianBlur(foreth, foreth, 3);
	morphologyEx(foreth, foreth, MORPH_DILATE, Mat(5, 5, CV_8U));
	morphologyEx(foreth, foreth, MORPH_DILATE, Mat(5, 5, CV_8U));
	//imshow("result1", edge);
	//waitKey(0);
	return foreth;
}

void UpdateHistogram(const cv::Mat& frame,const cv::Mat& mask,Histogram& f_post,Histogram& b_post,bool first = false)
{
	using namespace std;
	using namespace cv;
	Histogram hist_forth;
	Histogram hist_backgound;
	auto p = frame.ptr<Vec3b>(0);
	auto p_mask = mask.ptr<uchar>(0);
	for (size_t i = 0; i < frame.total(); i++)
	{
		if (p_mask[i])
		{
			hist_forth(p[i][0], p[i][1], p[i][2])++;
		}
		else
			hist_backgound(p[i][0], p[i][1], p[i][2])++;
	}
	for (size_t i = 0; i < hist_forth.size(); i++)
	{
		if (first)
		{
			f_post[i] = hist_forth[i] / (hist_forth[i] + hist_backgound[i]);
			b_post[i] = hist_backgound[i] / (hist_forth[i] + hist_backgound[i]);
		}
		else
		{
			f_post[i] = f_post[i] * 0.95 + 0.05*hist_forth[i] / (hist_forth[i] + hist_backgound[i]);
			b_post[i] = b_post[i] * 0.98 + 0.02*hist_backgound[i] / (hist_forth[i] + hist_backgound[i]);
		}
	}

	return ;
} 