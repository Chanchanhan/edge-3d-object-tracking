//
// Created by qqh on 18-8-6.
//

#include "Post.h"


Post::Post() {
    nf=0;
    nb=0;
}

void Post::Update(const cv::Mat &frame, const cv::Mat &mask, const int &radius, bool first) {
    UpdateDT(mask);
    UpdateHistogram(frame,mask,first);
    UpdateSegment(frame,radius);
    UpdateFrameDT(frame);
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
            Utils::BresehanCircle(vec2(i,j),radius,vec2s);
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
    cv::Mat countour_map = Mat::zeros(mask.size(), CV_8U);
    vector<vector<cv::Point>> countours;
    cv::findContours(mask, countours, 0, 1);
    drawContours(countour_map, countours, -1, Scalar(255, 255, 255));
    cv::distanceTransform(~countour_map, dtMap, cv::DIST_L2, 3);
    lv_set = dtMap.clone();
    auto p_mask = mask.ptr<float>(0);
    auto p_lv = lv_set.ptr<float>(0);

    for (size_t i = 0; i < lv_set.total(); i++) {
        p_lv[i] = p_mask[i] ? -p_lv[i] : p_lv[i];
    }
}


cv::Mat Post::SegmentByHistogram(const cv::Mat& frame)
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

void Post::UpdateFrameDT(const cv::Mat &frame) {
    using namespace cv;

    int edgeThresh = 1;
    int lowThreshold = 50;
    int const max_lowThreshold = 100;
    int ratio = 3;
    int kernel_size = 3;

    Mat dst,edge,gray,dist,bw;
    cvtColor( frame, gray, COLOR_BGR2GRAY );
    blur( gray, edge, Size(3,3) );
    Canny( edge, edge, lowThreshold, lowThreshold*ratio, kernel_size );
//    imshow("edge",edge);
    threshold(edge, bw, 40, 255, THRESH_BINARY | THRESH_OTSU);
    cv::distanceTransform(~bw, frame_dtMap, cv::DIST_L2, 3);
//    normalize(frame_dtMap,frame_dtMap,255);
    imshow("Distance Transform Image", frame_dtMap);
//    waitKey(0);


}