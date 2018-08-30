//
// Created by qqh on 18-8-6.
//

#include "Post.h"


Post::Post() {
}


void Post::UpdateFrameDT(const cv::Mat &frame) {
    using namespace cv;

    int lowThreshold = 10;
    int ratio = 3;
    int kernel_size = 3;

    Mat dst,edge,gray,dist,bw;
    cvtColor( frame, gray, COLOR_BGR2GRAY );
    blur( gray, edge, Size(3,3) );
    Canny( edge, edge, lowThreshold, lowThreshold*ratio, kernel_size );
    imshow("edge",edge);
    threshold(edge, bw, 40, 255, THRESH_BINARY | THRESH_OTSU);
    cv::distanceTransform(~bw, frame_dtMap, cv::DIST_L2, 3);



}