//
// Created by qqh on 18-8-6.
//

#include "Post.h"


struct cmp{
    bool operator()(const vec2 &a,const vec2 &b){
        if(a[1]!=b[1])
            return a[1]<b[1];
        return a[0]<b[0];
    }
};
void BresehanCircle(const vec2 &vec,int radius,std::vector<vec2> &sampleVecs){
    double c_x=0,c_y = radius;
    double d = 1-radius;
    static  std::set<vec2,cmp> point_set;
    static  bool first = true;
    if(first){
        while (c_x<c_y){
            point_set.insert({c_x,c_y});
            point_set.insert({c_y,c_x});
            point_set.insert({-c_x,c_y});
            point_set.insert({-c_y,c_x});

            point_set.insert({-c_x,-c_y});
            point_set.insert({-c_y,-c_x});
            point_set.insert({c_x,-c_y});
            point_set.insert({c_y,-c_x});

            if(d<0)
            {
                d = d+2*c_x+3;
            }
            else
            {
                d=d+2*(c_x-c_y)+5;
                c_y--;
            }
            c_x++;
        }
        first  = false;
    }

    for(auto& p:point_set)
    {
        sampleVecs.emplace_back(vec[0] + p[0],vec[1] + p[1]);
    }
}
Post::Post() {
    nf=0;
    nb=0;
}

void Post::Update(const cv::Mat &frame, const cv::Mat &mask, const int &radius, bool first) {
    UpdateDT(mask);
    UpdateHistogram(frame,mask,first);
    UpdateSegment(frame,radius);
//    cv::imshow("segment",segment*255);
//    cv::waitKey(0);
}

void Post::UpdateHistogram(const cv::Mat& frame,const cv::Mat& mask,bool first)
{
    using namespace std;
    using namespace cv;
    auto p = frame.ptr<Vec3b>(0);
    auto p_mask = mask.ptr<uchar>(0);
    auto p_lv = lv_set.ptr<float>(0);


    for (size_t i = 0; i < frame.total(); i++)
    {
        if (p_mask[i])
        {
            nf += Utils::He(p_lv[i]);
            hist_forth(p[i][0], p[i][1], p[i][2])++;
        }
        else{
            hist_backgound(p[i][0], p[i][1], p[i][2])++;
            nb += 1 - Utils::He(1 - p_lv[i]);
        }
    }
    for (size_t i = 0; i < hist_forth.size(); i++)
    {
        if (first)
        {

            f_post[i] = hist_forth[i]/(nf*hist_forth[i]+nb*hist_backgound[i]);
            b_post[i] = hist_backgound[i]/(nf*hist_forth[i]+nb*hist_backgound[i]);

        }
        else
        {
            f_post[i] = f_post[i] * 0.95 + 0.05*hist_forth[i] / (nf*hist_forth[i]+nb*hist_backgound[i]);
            b_post[i] = b_post[i] * 0.98 + 0.02*hist_backgound[i] / (nf*hist_forth[i]+nb*hist_backgound[i]);
        }
    }
    return ;
}
void Post::UpdateHistogramOptimize(const cv::Mat& mask)
{
    using namespace std;
    using namespace cv;
    auto p_mask = mask.ptr<uchar>(0);
    auto p_lv = lv_set.ptr<float>(0);


    for (size_t i = 0; i < mask.total(); i++)
    {
        if (p_mask[i])
        {
            nf += Utils::He(p_lv[i]);
        }
        else{
            nb += 1 - Utils::He(1 - p_lv[i]);
        }
    }
    for (size_t i = 0; i < hist_forth.size(); i++)
    {


        f_post[i] = f_post[i] * 0.95 + 0.05*hist_forth[i] / (nf*hist_forth[i]+nb*hist_backgound[i]);
        b_post[i] = b_post[i] * 0.98 + 0.02*hist_backgound[i] / (nf*hist_forth[i]+nb*hist_backgound[i]);

    }
}

void Post::UpdateSegment(const cv::Mat &frame,const int &radius) {
    using  namespace cv;
    forthMap = Mat::zeros(frame.rows,frame.cols,CV_32F);
    backMap = Mat::zeros(frame.rows,frame.cols,CV_32F);

    for(int i=0;i<frame.size().height;i++){
        for(int j =0;j<frame.size().width;j++){
            std::vector<vec2> vec2s;
            BresehanCircle(vec2(i,j),radius,vec2s);
            for(auto v:vec2s){
                Vec3b bgr =  frame.at<Vec3b>(i,j);
                forthMap.at<float>(i,j)+= f_post(bgr[0], bgr[1], bgr[2]);
                backMap.at<float>(i,j)+= b_post(bgr[0], bgr[1], bgr[2]);
            }
            forthMap.at<float>(i,j) *= (1.0/vec2s.size());
            backMap.at<float>(i,j) *= (1.0/vec2s.size());

        }
    }
    normalize(forthMap,forthMap);
    normalize(backMap,backMap);
    segment = forthMap - backMap;
}
void Post::UpdateDT(const cv::Mat &mask_) {
    mask = mask_.clone();
    using namespace cv;
    using namespace std;
    cv::Mat countour_map = Mat::zeros(mask.size(),CV_8U);
    vector<vector<cv::Point>> countours;
    cv::findContours(mask, countours, 0, 1);
    drawContours(countour_map, countours, -1, Scalar(255, 255, 255));
    cv::distanceTransform(~countour_map, dtMap, cv::DIST_L2, 3);
    lv_set = dtMap.clone();
    auto p_mask = mask.ptr<float>(0);
    auto p_lv = lv_set.ptr<float>(0);

    for (size_t i = 0; i < lv_set.total(); i++) {
        p_lv[i] = p_mask[i]? -p_lv[i] : p_lv[i];
    }
}