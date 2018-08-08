//
// Created by chuguanyi on 18-3-21.
//

#include <vector>
#include <iostream>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "glm.h"
#include "ModelWarpper.h"
#include "Utils.h"
#include "Solver.h"
#include "Post.h"
using namespace std;
using namespace cv;

const int MAX_FILE_LEN = 65535;
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ModelWarpper render;
    render.LoadObj("/home/qqh/Developer/DataSets/6d-pose/LINEMOD/cat/cat.obj");
    Matx33d K;
    K << 1522.56, 0, 479.5,
    	0, 1522.56, 271.5,
    	0, 0, 1;
    int width = 960, height = 544;

    render.Init(K,width,height);


    char imgFile[MAX_FILE_LEN];
    sprintf(imgFile, "/home/qqh/Developer/DataSets/6d-pose/cat/JPEGImages/%06d.jpg", 0);

    double pose_parameter_tr[6] = { -1.360675971955060959e-02,		- 1.190256699919700623e-02,1.395618438720703125e+00,
                                    1.435960412025451660e+00,		1.953682899475097656e+00,- 1.178424954414367676e+00};


    Mat frame = imread(imgFile);
    cv::Mat last_frame;

    Histogram f, g;
    int frame_id = 0;
    Post post;
    Solver solver;
    solver.init(pose_parameter_tr);

    while (!frame.empty())
    {
        vector<vec3> support_points;
        vec3 ti(pose_parameter_tr);
        vec3 ri(pose_parameter_tr + 3);

        mat4 T = Utils::GetTranformation(ri, ti);
        render.DisplayGL(T);
        auto rendered = render.render_.getRenderedImg();
        auto mask = rendered > 1;
        int radius = 8;
        if (frame_id == 0)
            post.Update(frame,mask,radius, true);
        else
            post.Update(frame,mask,radius);

        vec3 tj0(pose_parameter_tr);
        vec3 rj0(pose_parameter_tr + 3);

        Utils::SampleVertices(support_points, rendered, render);

        solver.Optimize(pose_parameter_tr,K,post,render);

//        Mat segment = post.SegmentByHistogram(frame);
//        solver.Optimize(pose_parameter_tr, support_points, K, post.frame_dtMap);
//        solver.Optimize(pose_parameter_tr, support_points, K,segment);


        vec3 tj(pose_parameter_tr);
        vec3 rj(pose_parameter_tr + 3);

        Mat draw0 = render.DrawOn(frame, rj0, tj0, Scalar(0, 255, 0));
        Mat draw = render.DrawOn(frame, rj, tj, Scalar(0, 255, 0));
        imshow("input", draw0);
        imshow("result", draw);
        waitKey(0);
        frame_id++;
        last_frame = frame;
        sprintf(imgFile, "/home/qqh/Developer/DataSets/6d-pose/cat/JPEGImages/%06d.jpg", frame_id);
        frame = imread(imgFile);
    }

    return 0;
}
