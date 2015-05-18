#ifndef CSFEATURE_H
#define CSFEATURE_H

#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include "common.h"
#include <QVector>

using namespace pcl;
using namespace Eigen;
using namespace std;


class Gaus {
public:
    Gaus(): mean(Vector3d::Zero()), covariance(Matrix3d::Zero()){}
    ~Gaus() {}
    Vector3d mean;
    Matrix3d covariance;
    Matrix3d cov_inverse;
    std::vector<Eigen::Vector3f> points;
};


class Vcsh{
public:
    Vcsh() {hist.resize(48); std::fill(hist.begin(), hist.end(), 0);}
    ~Vcsh() {}
    std::vector<float> hist;
    void addrgb(Vector3i rgb);
};


class GausVcsh{
public:
    Gaus shape;
    Vcsh color;
    std::vector<PointT> points;
    void generate(float gridsize);
    float getDist(GausVcsh other);
};

class CsFeature
{
public:
    CsFeature();
    ~CsFeature();
//    std::vector<GausVcsh> feature;

    std::vector<float> hist;
    float getDist(CsFeature other);
    float getLongDist(CsFeature other);

};


class CsFeatureEstimation
{
public:
    CsFeatureEstimation();
    ~CsFeatureEstimation();
    void initBox(CloudPtr &cloud);  //initialize boxsize, amount of grids
    CsFeature compute(CloudPtr &cloud);
    float evalOnePoint(PointT pt);
    Vector4f m_boxMin, m_boxMax;
    CloudPtr m_keypoints;
    int m_keySize;

    CsFeature outF; //feature to be estimated



    float m_gridsize;
    int m_xNr, m_yNr, m_zNr;
    int m_gridNr;


};



#endif // CSFEATURE_H
