//
// Created by qqh on 18-8-7.
//

#include "Utils.h"
const double s = 15;
double Utils::He(const double &phi){
    //with bigger s, the smaller recall and  higher ap
    return M_1_PI*(-atan(s*phi)+ M_PI_2);
}
double Utils::delta(const double &phi) {
    return s*M_1_PI/(phi*phi*s*s+1);
}
Eigen::Matrix3d Utils::skew(const Eigen::Vector3d&v)
{
    Eigen::Matrix3d m;
    m.fill(0.);
    m(0, 1) = -v(2);
    m(0, 2) = v(1);
    m(1, 2) = -v(0);
    m(1, 0) = v(2);
    m(2, 0) = -v(1);
    m(2, 1) = v(0);
    return m;
}

Eigen::Quaterniond Utils::toQuaterniond(const Eigen::Vector3d& v3d, double* angle )
{
    const double SMALL_EPS = 1e-10;
    double theta = v3d.norm();
    if (angle != NULL)
        *angle = theta;
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if (theta<SMALL_EPS)
    {
        double theta_sq = theta * theta;
        double theta_po4 = theta_sq * theta_sq;
        imag_factor = 0.5 - 0.0208333*theta_sq + 0.000260417*theta_po4;
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta / theta;
    }

    return Eigen::Quaterniond(real_factor,
                              imag_factor*v3d.x(),
                              imag_factor*v3d.y(),
                              imag_factor*v3d.z());
}

mat4 Utils::GetTranformation(vec3 r, vec3 t)
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

bool Utils::SampleVertices(std::vector<vec3>& support_points, cv::Mat &rendered, ModelWarpper& cube)
{
    std::vector<std::vector<cv::Point> > rendered_countours;
    findContours(rendered, rendered_countours, 0, 1);
    if (rendered_countours.empty())
        return false;
    const int sample_num = std::min((int)rendered_countours[0].size(), 100);
    int step = rendered_countours[0].size()/sample_num;
//    int  step =1;
    for (size_t i = 0; i < sample_num*step; i+=step)
    {
        auto sample = (rendered_countours[0][i]);
        support_points.push_back((cv::Vec3f)cube.render_.get3DPos(sample.x, sample.y));
    }
    return true;
}
vec3 Utils::world2camera(vec4 Xw, mat4 T)
{
    vec4 Xc = T * Xw;
    Xc = Xc / Xc[3];
    return { Xc[0], Xc[1], Xc[2] };
}

// $\pi(X) = x$
// from the camera coordinate to image coordinate
vec2 Utils::Project(vec3 pt, mat3 K)
{
    if (pt[2] <= 0)
        return { -1,-1 };
    pt = pt / pt[2];
    auto p = K * pt;
    p = p / pt[2];
    return { p[0],p[1] };
}

vec2 Utils::world2pixel(vec4 Xw, mat4 T, mat3 K)
{
    return Project(world2camera(Xw, T), K);
}

vec2 Utils::world2pixel(vec3 Xw, mat3 R, vec3 t, mat3 K)
{

    return Project(R * Xw + t, K);
}



struct cmp{
    bool operator()(const vec2 &a,const vec2 &b){
        if(a[1]!=b[1])
            return a[1]<b[1];
        return a[0]<b[0];
    }
};
void Utils::BresehanCircle(const vec2 &vec,int radius,std::vector<vec2> &sampleVecs){
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