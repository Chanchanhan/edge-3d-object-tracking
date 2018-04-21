//
// Created by chuguanyi on 18-4-16.
//

#pragma once
#include <numeric>
class Histogram
{
public:
//private:
	const int quantization = 64;
	const int fbit = 2;
	double* BGR;
	Histogram()
	{
		BGR = new double[quantization*quantization*quantization];
		std::fill(BGR, BGR + quantization * quantization*quantization, 1);
	};
	~Histogram()
	{
		delete[] BGR;
	};
	double size()
	{
		return quantization * quantization * quantization;
	}
	double& operator()(const int b, const int g, const int r)
	{
		return BGR[((b>> fbit)<<12) | ((g>> fbit)<<6) | (r>> fbit)];
	}
	double operator()(const int b, const int g, const int r) const
	{
		return BGR[((b>> fbit)<<12) | ((g>> fbit)<<6) | (r>>fbit)];
	}
	double& operator[](int i)
	{
		return BGR[i];
	}
	double operator[](int i) const
	{
		return BGR[i];
	}
};



