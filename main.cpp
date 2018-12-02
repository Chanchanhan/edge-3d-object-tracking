
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


    char imgFile[MAX_FILE_LEN],rFile[MAX_FILE_LEN],tFile[MAX_FILE_LEN];
    int frame_id = 0;

    double pose_parameter_tr[6],pose_parameter_yolo2[6];
    ifstream r_stream ,t_stream,r2_stream ,t2_stream;

    while(!r_stream.is_open()){
        frame_id++;
        sprintf(imgFile, "/home/qqh/Developer/DataSets/6d-pose/cat/JPEGImages/%06d.jpg", frame_id);
        sprintf(rFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr/r_t%04d.txt", frame_id);
        sprintf(tFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr/t_%04d.txt", frame_id);
        r_stream=ifstream(rFile);
        t_stream=ifstream(tFile);

        sprintf(rFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr2/r_t%04d.txt", frame_id);
        sprintf(tFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr2/t_%04d.txt", frame_id);
        r2_stream=ifstream(rFile);
        t2_stream=ifstream(tFile);
    }
    char s[MAX_FILE_LEN];
    int cnt=0;
    while(t_stream.getline(s,MAX_FILE_LEN)){
        pose_parameter_tr[cnt++]=atof(s);
    }
    while(r_stream.getline(s,MAX_FILE_LEN)){
        pose_parameter_tr[cnt++]=atof(s);
    }
    cnt =0;
    while(t2_stream.getline(s,MAX_FILE_LEN)){
        pose_parameter_yolo2[cnt++]=atof(s);
    }
    while(r2_stream.getline(s,MAX_FILE_LEN)){
        pose_parameter_yolo2[cnt++]=atof(s);
    }
    t_stream.close(),r_stream.close();
    Mat frame = imread(imgFile);
    cv::Mat last_frame;
    bool first = true;
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

        post.UpdateFrameDT(frame);


        vec3 tj0(pose_parameter_tr);
        vec3 rj0(pose_parameter_tr + 3);

        Utils::SampleVertices(support_points, rendered, render);
        solver.Optimize(pose_parameter_tr, support_points, K, post.frame_dtMap);
//        Mat Segment = post.SegmentByHistogram(frame);


        vec3 tj(pose_parameter_tr);
        vec3 rj(pose_parameter_tr + 3);

        vec3 tj2(pose_parameter_yolo2);
        vec3 rj2(pose_parameter_yolo2 + 3);
        Mat draw0 = render.DrawOn(frame, rj0, tj0, Scalar(0, 255, 0));

        Mat draw = render.DrawOn(frame, rj, tj, Scalar(0, 255, 0));
        Mat draw2 = render.DrawOn(frame, rj2, tj2, Scalar(0, 255, 0));
        cv::resize(draw0,draw0,draw0.size()/2);
        cv::resize(draw,draw,draw.size()/2);
        cv::resize(draw2,draw2,draw2.size()/2);

        imshow("origin",draw2 );
        imshow("my before refine",draw0  );
        imshow("my after refine", draw);
        if(first){
            waitKey(0);
            first = false;
        }
        waitKey(1);
        last_frame = frame;


        while(!r_stream.is_open()){
            frame_id++;

            sprintf(imgFile, "/home/qqh/Developer/DataSets/6d-pose/cat/JPEGImages/%06d.jpg", frame_id);
            sprintf(rFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr/r_t%04d.txt", frame_id);
            sprintf(tFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr/t_%04d.txt", frame_id);
            r_stream=ifstream(rFile);
            t_stream=ifstream(tFile);

            sprintf(rFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr2/r_t%04d.txt", frame_id);
            sprintf(tFile, "/home/qqh/Developer/DataSets/6d-pose/backup/cat/test/pr2/t_%04d.txt", frame_id);
            r2_stream=ifstream(rFile);
            t2_stream=ifstream(tFile);
//
//
//            frame = imread(imgFile);
//            vector<vec3> support_points;
//            vec3 ti(pose_parameter_tr);
//            vec3 ri(pose_parameter_tr + 3);
//
//            mat4 T = Utils::GetTranformation(ri, ti);
//            render.DisplayGL(T);
//            auto rendered = render.render_.getRenderedImg();
//            auto mask = rendered > 1;
//            post.UpdateFrameDT(frame);
//            solver.Optimize(pose_parameter_tr, support_points, K, post.frame_dtMap);

//            waitKey(1);

        }
        if(r_stream.is_open()){
            char s[MAX_FILE_LEN];
            int cnt=0;
            while(t_stream.getline(s,MAX_FILE_LEN)){
                pose_parameter_tr[cnt++]=atof(s);
            }
            while(r_stream.getline(s,MAX_FILE_LEN)){
                pose_parameter_tr[cnt++]=atof(s);
            }
            t_stream.close(),r_stream.close();
            solver.init(pose_parameter_tr);
            cnt=0;
            while(t2_stream.getline(s,MAX_FILE_LEN)){
                pose_parameter_yolo2[cnt++]=atof(s);
            }
            while(r2_stream.getline(s,MAX_FILE_LEN)){
                pose_parameter_yolo2[cnt++]=atof(s);
            }
            solver.init(pose_parameter_tr);

        }

        frame = imread(imgFile);
    }

    return 0;
}
