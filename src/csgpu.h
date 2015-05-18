#ifndef CSGPU_H
#define CSGPU_H

#include <vector>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>



//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> Cloud;
//typedef Cloud::Ptr CloudPtr;
//typedef Cloud::ConstPtr CloudConstPtr;

//typedef pcl::PointXYZRGBNormal PointNT;
//typedef pcl::PointCloud<PointNT> CloudN;
//typedef CloudN::Ptr CloudNPtr;

class CSGPU
{
public:
    CSGPU() {}
    CSGPU(float gridsize_, int xnr_, int ynr_, int znr_, int partnr_, int cloudsize_){
        gridsize=gridsize_; xNr=xnr_; yNr=ynr_; zNr=znr_; partNr=partnr_; cloudSize=cloudsize_;
        cloudpos = (float3 *)malloc(cloudSize * sizeof(float3));
        cloudhsv = (float3 *)malloc(cloudSize * sizeof(float3));
//        refcloud = (float3 *)malloc(refcloudSize * sizeof(float3));
        partpos = (float3 *)malloc(partNr * sizeof(float3));
        partrot = (float3 *)malloc(partNr * sizeof(float3));
        histsize =(xNr+1) * (yNr+1) * (zNr+1) * 8;// 24*refcloudSize;//
        refhist = (float *)malloc(histsize * sizeof(float));
//        refhist = (float *)malloc(((xNr+yNr+zNr+3) * 8) * sizeof(float));

//        cloudpos = new float3[cloudSize]; cloudhsv = new float3[cloudSize];
//        partpos = new float3[partNr]; partrot = new float3[partNr];
//        refhist = new float((xNr+1) * (yNr+1) * (zNr+1) * 24);
    }
    ~CSGPU() {}
    int aa;
    int histsize;
    size_t partNr, cloudSize, refcloudSize;
    size_t xNr, yNr, zNr;
    float3 minPt, maxPt;
    float gridsize;
    float *refhist;
//    thrust::device_vector<float3> cloudpos;
    float3 *cloudpos;
    float3 *cloudhsv;
    float3 *refcloud;
    float3 *partpos;
    float3 *partrot;

    std::vector<float> compute();
};

#endif // CSGPU_H
