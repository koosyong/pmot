#ifndef PARTICLEPOSE_H
#define PARTICLEPOSE_H

#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>
#include <functional>

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <algorithm>
#include <numeric>
#include <QVector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "common.h"
#include "csfeature.h"


using namespace pcl;
using namespace Eigen;
using namespace std;




class Particlepose
{
public:
    Particlepose();
    ~Particlepose();
    void setdata(CloudPtr scene, CloudPtr model/*, pcl::gpu::Octree &octreegpu*/, int cnt);
    CloudPtr getResult();
    void initSample();
    void resample(float varDist, float varAng);
    void weight();
    CloudPtr toCloud();
    void updateModel(CloudPtr cloud);

    //estimate features for each particle and store into a long list
    std::vector<float> weightGPU(float gridsize, int xnr, int ynr, int znr, Vector4f min, Vector4f max, std::vector<float> refhist);


    float distfeature(VFHSignature308 f1, VFHSignature308 f2);
    inline void weightByDist();
    std::vector<float> getPatchScore(CloudPtr cloud);

    CsFeature  m_modelF;
    CsFeatureEstimation csest;

private:
    boost::mt19937 m_gen;
    tracking::ParticleXYZRPY sampleWithVar(tracking::ParticleXYZRPY part, float varDist, float varAng);
    tracking::ParticleXYZRPY m_finalParticle, m_lastFinalParticle, m_lastlastFinalParticle,m_lastlastlastFinalParticle
    , m_vel,m_acc;
    Eigen::Vector4f m_sceneCentroid;
    Eigen::Vector4f m_modelCentroid;
    pcl::gpu::Octree m_octreegpu;
    CloudPtr m_scene;
    CloudPtr m_model, m_modelOrig;
    CloudNPtr m_sceneN;
    CloudNPtr m_modelN;
    int m_particlenum;   //particle number
    std::vector<tracking::ParticleXYZRPY> m_particles, m_lastspeed, m_bestspeed;

    pcl::KdTreeFLANN<PointNT>::Ptr m_kdtreeSceneN;

    int m_cnt, m_internalCounter;

    float minDist, maxDist;

    bool usegpu;
};



#endif // PARTICLEPOSE_H
