//
// Created by qqh on 18-8-7.
//

#ifndef ROBOT_POST_H
#define ROBOT_POST_H


#include "types.h"
#include "Utils.h"
class Post{

public:
    Post();
    ~Post(){};

    void UpdateFrameDT(const cv::Mat &frame);

    cv::Mat dtMap,frame_dtMap;
};



#endif //ROBOT_POST_H
