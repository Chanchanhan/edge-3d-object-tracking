//
// Created by qqh on 18-8-7.
//

#ifndef ROBOT_POST_H
#define ROBOT_POST_H


#include "Histogram.h"
#include "types.h"
#include "Utils.h"
class Post{

public:
    Post();
    ~Post(){};
    void Update(const cv::Mat &frame , const cv::Mat &mask,const int &radius,bool first = false);
    void UpdateHistogram(const cv::Mat& frame,const cv::Mat& mask,bool first = false);
    void UpdateSegment(const cv::Mat &frame,const int &radius);
    void UpdateDT(const cv::Mat& mask);
    void UpdateHistogramOptimize(const cv::Mat& mask);

    Histogram f_post, b_post;
    double nf,nb;
    cv::Mat segment;
    cv::Mat dtMap;
    cv::Mat mask;
    cv::Mat lv_set;
    cv::Mat forthMap,backMap;
    Histogram hist_forth;
    Histogram hist_backgound;
};



#endif //ROBOT_POST_H
