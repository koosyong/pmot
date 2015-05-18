#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include "csgpu.h"
#include <iostream>
#include "stdio.h"

__global__ void add(int a, int b, int *c)
{
    *c = a + b;
}
__device__  unsigned int  my_rand(unsigned int *seed) {

    // constants for random no gen.
    unsigned long a = 16807;
    unsigned long m = 2147483647;   	// 2^31 - 1
    unsigned long x = (unsigned long) *seed;

    x = (a * x)%m;

    *seed = (unsigned int) x;

    return x;
}



__global__ void run(int xNr, int yNr, int zNr,int partNr,
                    float3 minPt, float3 maxPt,float gridsize,
                    float3 *cloudpos,  float3 *cloudhsv, int cloudSize,
                    float3 *partpos,  float3 *partrot, int histsize, float *refhist, float *d_weights)
{

//    int id = blockIdx.x* blockDim.x + threadIdx.x;
//    printf("ind of %i is started\n", id);

    int id =  blockIdx.x;
    int pId = threadIdx.x;

//    int id =  blockIdx.x*blockDim.x + threadIdx.x;
//    int pId = threadIdx.x;

    if (id+1>partNr)  {return;}
    float px,py,pz,roll,pitch,yaw;

    px = partpos[id].x; py = partpos[id].y; pz = partpos[id].z;
    roll = partrot[id].x; pitch = partrot[id].y; yaw = partrot[id].z;

    float t[4][4];

    float A = cosf (yaw), B = sinf (yaw), C = cosf (pitch), D = sinf (pitch),
    E = cosf (roll), F = sinf (roll), DE = D*E, DF = D*F;
    t[0][0] = A*C; t[0][1] = A*DF - B*E; t[0][2] = B*F + A*DE; t[0][3] = px;
    t[1][0] = B*C; t[1][1] = A*E + B*DF; t[1][2] = B*DE - A*F; t[1][3] = py;
    t[2][0] = -D;  t[2][1] = C*F;        t[2][2] = C*E;        t[2][3] = pz;
    t[3][0] = 0.f; t[3][1] = 0.f;        t[3][2] = 0.f;        t[3][3] = 1.f;


    //printf("time of ind %i is %i\n", id, clock());
//    int indOffset = histsize * id; int nrPerThr;
//     __shared__ float hists[10000]; indOffset=0;
//    nrPerThr = (int)ceil((float)histsize/blockDim.x);
//    for(int i=pId*nrPerThr; i<min(nrPerThr*(pId+1),histsize); i++){
//        hists[i]=0.f;
//    }
//    __syncthreads();


    int nrPerThr = (int)ceil( ((float)cloudSize) / ((float)blockDim.x) );
//    printf("number per thread is %i\n", nrPerThr);

    //    {printf("threadidx is %i start is  %i end is %i\n",pId, nrPerThr*pId, min(nrPerThr*(pId+1),cloudSize));}




    float thisCorr = 0.f;

    int thres = (int)ceil(512.f/(float)partNr * 100.f);
    unsigned int seed = blockIdx.x+threadIdx.x;
    for(int i=nrPerThr*pId; i<min(nrPerThr*(pId+1),cloudSize); i++){
//    for(int i=0; i<cloudSize; i++){

//        unsigned int rand_ = my_rand(&seed)%100;
//        if (rand_>thres){continue;}
//        if ((i+id+pId)% 5 !=0){continue;}

        float _x = cloudpos[i].x, _y = cloudpos[i].y, _z = cloudpos[i].z;

        float x = t[0][0]*_x + t[0][1]*_y + t[0][2]*_z + t[0][3];
        float y = t[1][0]*_x + t[1][1]*_y + t[1][2]*_z + t[1][3];
        float z = t[2][0]*_x + t[2][1]*_y + t[2][2]*_z + t[2][3];
        float h=cloudhsv[i].x, s=cloudhsv[i].y, v=cloudhsv[i].z;


        int grayindex;
        float weightgray, weightcolor;
        int hindex1,hindex2;
        float hweight1, hweight2;
        h = h*360.0f;
//        if(h>=0   && h <=60)  {hindex1=0; hindex2=1; hweight2= h/60.0f;          hweight1=1.0f -hweight2;}
//        if(h>60  && h <=120)  {hindex1=1; hindex2=2; hweight2= (h-60.0f)/60.0f;  hweight1=1.0f -hweight2;}
//        if(h>120 && h <=180)  {hindex1=2; hindex2=3; hweight2= (h-120.0f)/60.0f; hweight1=1.0f -hweight2;}
//        if(h>180 && h <=240)  {hindex1=3; hindex2=4; hweight2= (h-180.0f)/60.0f; hweight1=1.0f -hweight2;}
//        if(h>240 && h <=300)  {hindex1=4; hindex2=5; hweight2= (h-240.0f)/60.0f; hweight1=1.0f -hweight2;}
//        if(h>300 && h <=360)  {hindex1=5; hindex2=0; hweight2= (h-300.0f)/60.0f; hweight1=1.0f -hweight2;}

        hindex1 = (int)floor(h/60.f);
        if (hindex1==5){
            hindex2=0;
        }else{
            hindex2=hindex1+1;
        }

        hweight2 = (h- 60.f*(float)hindex1)/60.f; hweight1 = 1.f - hweight2;

        if(v<0.5) {grayindex = 6;}
        if(v>=0.5){grayindex = 7;}
        weightcolor = pow(s,(0.14f * pow(1.0f/v,0.9f)));
        weightgray  = 1.0f - weightcolor;
        if( v<0.1f){weightcolor=0.0f; weightgray=1.0f;}

//        weightcolor=0.f; weightgray=1.f; grayindex=7;

        hweight1 = weightcolor * hweight1;
        hweight2 = weightcolor * hweight2;

        int xInd, yInd, zInd, theInd;
        xInd = (int)floorf( (x - minPt.x)/gridsize );
        if((xInd >= xNr) || (xInd<0)) {continue;}
        yInd = (int)floorf( (y - minPt.y)/gridsize );
        if((yInd >= yNr) || (yInd<0)) {continue;}
        zInd = (int)floorf( (z - minPt.z)/gridsize );
        if((zInd >= zNr) || (zInd<0)) {continue;}
        theInd = zInd*xNr*yNr + xInd*yNr + yInd;

//        printf("indexs are %i %i %i\n", xInd, yInd, zInd);
//        x = x - gridsize*(float)xInd - minPt.x;
//        y = y - gridsize*(float)yInd - minPt.y;
//        z = z - gridsize*(float)zInd - minPt.z;
//        int xPlusInd = zInd*xNr*yNr + (xInd+1)*yNr + yInd;
//        int yPlusInd = zInd*xNr*yNr + xInd*yNr + (yInd+1);
//        int zPlusInd = (zInd+1)*xNr*yNr + xInd*yNr + yInd;
//        float weight1, weight2;
//        float incrValue =1.f;

//        weight2 = x/gridsize; weight1 = 1.f-weight2;  //weight2 *= weight2; weight1*=weight1;
//        thisCorr += (refhist[hindex1 + 24*theInd + 0] *  hweight1*weight1*incrValue);
//        thisCorr += (refhist[hindex1 + 24*xPlusInd + 0] *   hweight1*weight2*incrValue);
//        thisCorr += (refhist[hindex2 + 24*theInd + 0]   *   hweight2*weight1*incrValue);
//        thisCorr += (refhist[hindex2 + 24*xPlusInd + 0]  *   hweight2*weight2*incrValue);
//        thisCorr += (refhist[grayindex + 24*theInd + 0]  *   weightgray*weight1*incrValue);
//        thisCorr += (refhist[grayindex + 24*xPlusInd + 0] *   weightgray*weight2*incrValue);

//        weight2 = y/gridsize; weight1 = 1.f-weight2;  //weight2 *= weight2; weight1*=weight1;
//        thisCorr += (refhist[hindex1 + 24*theInd + 8] *   hweight1*weight1*incrValue);
//        thisCorr += (refhist[hindex1 + 24*yPlusInd + 8] *   hweight1*weight2*incrValue);
//        thisCorr += (refhist[hindex2 + 24*theInd + 8]   *   hweight2*weight1*incrValue);
//        thisCorr += (refhist[hindex2 + 24*yPlusInd + 8]  *   hweight2*weight2*incrValue);
//        thisCorr += (refhist[grayindex + 24*theInd + 8]  *   weightgray*weight1*incrValue);
//        thisCorr += (refhist[grayindex + 24*yPlusInd + 8] *   weightgray*weight2*incrValue);

//        weight2 = z/gridsize; weight1 = 1.f-weight2; // weight2 *= weight2; weight1*=weight1;
//        thisCorr += (refhist[hindex1 + 24*theInd + 16 ] *  hweight1*weight1*incrValue);
//        thisCorr += (refhist[hindex1 + 24*zPlusInd + 16 ] *   hweight1*weight2*incrValue);
//        thisCorr += (refhist[hindex2 + 24*theInd + 16 ]  *   hweight2*weight1*incrValue);
//        thisCorr += (refhist[hindex2 + 24*zPlusInd + 16 ]  *   hweight2*weight2*incrValue);
//        thisCorr += (refhist[grayindex + 24*theInd + 16 ]  *   weightgray*weight1*incrValue);
//        thisCorr += (refhist[grayindex + 24*zPlusInd + 16 ] *   weightgray*weight2*incrValue);


        x = x - gridsize*(float)xInd - minPt.x;
        y = y - gridsize*(float)yInd - minPt.y;
        z = z - gridsize*(float)zInd - minPt.z;

//        float numsafe=1000.f;
//        x *= numsafe; y*=numsafe; z*=numsafe; gridsize*=numsafe ;

        float x_=gridsize-x,  y_=gridsize-y,  z_=gridsize-z;
        float dist0 = sqrt(x*x + y*y +z*z);
        int xPInd = zInd*xNr*yNr + (xInd+1)*yNr + yInd;  float distxP = sqrt(x_*x_ + y*y +z*z) ;
        int xyPInd = zInd*xNr*yNr + (xInd+1)*yNr + yInd+1; float distxyP = sqrt(x_*x_ + y_*y_ +z*z);
        int xzPInd = (zInd+1)*xNr*yNr + (xInd+1)*yNr + yInd; float distxzP = sqrt(x_*x_ + y*y +z_*z_);
        int xyzPInd = (zInd+1)*xNr*yNr + (xInd+1)*yNr + yInd+1; float distxyzP = sqrt(x_*x_ + y_*y_ +z_*z_);
        int yPInd = zInd*xNr*yNr + xInd*yNr + (yInd+1); float distyP = sqrt(x*x + y_*y_ +z*z);
        int yzPInd = (zInd+1)*xNr*yNr + xInd*yNr + (yInd+1); float distyzP = sqrt(x*x + y_*y_ +z_*z_);
        int zPInd = (zInd+1)*xNr*yNr + xInd*yNr + yInd;  float distzP = sqrt(x*x + y*y +z_*z_);

        float incrValue =1.f; float weight; int offset;
        float bandwidth = gridsize;
        float gridsize3= 0.000001f;//gridsize*gridsize*gridsize;
//        weight = max(0.f, (bandwidth-dist0))/gridsize;
        weight = max(0.f, ((x_*y_*z_)/gridsize3));
        offset = 8*theInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxP))/gridsize;
        weight = max(0.f, ((x*y_*z_)/gridsize3));
        offset = 8*xPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxyP))/gridsize;
        weight = max(0.f, ((x*y*z_)/gridsize3));
        offset = 8*xyPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxzP))/gridsize;
        weight = max(0.f, ((x*y_*z)/gridsize3));
        offset = 8*xzPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxyzP))/gridsize;
        weight = max(0.f, ((x*y*z)/gridsize3));
        offset = 8*xyzPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distyP))/gridsize;
        weight = max(0.f, ((x_*y*z_)/gridsize3));
        offset = 8*yPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distyzP))/gridsize;
        weight = max(0.f, ((x_*y*z)/gridsize3));
        offset = 8*yzPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distzP))/gridsize;
        weight = max(0.f, ((x_*y_*z)/gridsize3));
        offset = 8*zPInd;
        thisCorr += (refhist[hindex1 + offset] * hweight1*weight);
        thisCorr += (refhist[hindex2 + offset] * hweight2*weight);
        thisCorr += (refhist[grayindex + offset] * weightgray*weight);
    }

    atomicAdd(&d_weights[id], thisCorr);


//    nrPerThr = (int)ceil((float)histsize/blockDim.x);
//    float thisdiff=0.f;
//    for(int i=pId*nrPerThr; i<min(nrPerThr*(pId+1),histsize); i++){
////            d_weights[id]+= pow( (fabsf(hists[i + indOffset] - refhist[i])),2);
//        thisdiff += powf( (fabsf(hists[i + indOffset] - refhist[i])),1);
//    }
//    atomicAdd(&d_weights[id], thisdiff);

//    __syncthreads();

}

std::vector<float> CSGPU::compute()
{
    float *d_weights;
    float *weights;
    weights = (float *)malloc(partNr * sizeof(float));


//    size_t histSize = (xNr+1) * (yNr+1) * (zNr+1) * 24;
    size_t histSize = histsize;

    size_t histSize_ = partNr*histsize;
    float* d_hists;

    cudaMalloc((void **) &d_weights, partNr*sizeof(float));
    cudaMemset(d_weights, 0.f, partNr*sizeof(float));

//    cudaMalloc((void **) &d_hists, histSize_*sizeof(float));
//    cudaMemset(d_hists, 0.f, histSize_*sizeof(float));

//    cudaMemcpy(d_hists, hists, histSize_*sizeof(float), cudaMemcpyHostToDevice);
//    cudaMemset(d_weights, 0, partNr*sizeof(float));

    float3 *d_cloudpos, *d_cloudhsv, *d_partpos, *d_partrot, *d_refcloud;
    float *d_refhist;
    cudaMalloc((void **) &d_cloudpos, cloudSize*sizeof(float3));
    cudaMalloc((void **) &d_cloudhsv, cloudSize*sizeof(float3));
    cudaMalloc((void **) &d_partpos, partNr*sizeof(float3));
    cudaMalloc((void **) &d_partrot, partNr*sizeof(float3));
    cudaMalloc((void **) &d_refhist, histSize*sizeof(float));



    cudaMemcpy(d_cloudpos, cloudpos, cloudSize*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_cloudhsv, cloudhsv, cloudSize*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_partpos, partpos, partNr*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_partrot, partrot, partNr*sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_refhist, refhist, histSize*sizeof(float), cudaMemcpyHostToDevice);



    cudaEvent_t start, stop;
    float time;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    cudaEventRecord(start, 0);
    // Do calculation on device:

    int threadDim = std::min( (int)floor(65535.f/(float)partNr),  512);
    if (threadDim>32){
      threadDim = threadDim - threadDim%32;
    }

     run<<<partNr,512>>>(xNr, yNr, zNr, partNr,
                     minPt, maxPt, gridsize,
                     d_cloudpos,  d_cloudhsv, cloudSize,
                     d_partpos, d_partrot, histSize, d_refhist, d_weights);

//    run<<<100,100>>>(xNr, yNr, zNr, partNr,
//                    minPt, maxPt, gridsize,
//                    d_cloudpos,  d_cloudhsv, cloudSize,
//                    d_partpos, d_partrot, histSize, d_refhist, d_weights);
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time, start, stop);
    printf ("Time for the kernel: %f ms\n", time);

    cudaMemcpy(weights, d_weights, partNr*sizeof(float), cudaMemcpyDeviceToHost);

    float *hists;
//    hists = (float *)malloc(histSize_ * sizeof(float));
//    cudaMemcpy(hists, d_hists, histSize_*sizeof(float), cudaMemcpyDeviceToHost);


//    for(int i=0; i<100; i++){
//        std::cout<<weights[i]<<std::endl;
//    }
    cudaFree(d_weights); cudaFree(d_cloudpos);   cudaFree(d_cloudhsv);
    cudaFree(d_partpos); cudaFree(d_partrot);    cudaFree(d_refhist); cudaFree(d_hists);
////    for(int i=0; i<partNr; i++){
////        std::cout<<i<<"th weight is: "<<weights[i]<<std::endl;
//////        printf ("%i th Decimals: %d \n", i, weights[i]);
////    }
    std::vector<float> out; out.resize(partNr);
    for (size_t i=0; i<partNr; i++){
        out[i] = weights[i];
    }

    free(weights); free(cloudpos); free(cloudhsv);
    free(partpos); free(partrot); free(refhist); free(hists);

    return out;

//    int *dev_c;
//    cudaMalloc((void**)&dev_c, sizeof(int));
//    add<<<1,1>>>(a,b,dev_c);
//    cudaMemcpy(&c, dev_c, sizeof(int), cudaMemcpyDeviceToHost);
////    std::cout<<a<<"+"<<b" = "<< c<<std::endl;
//    cudaFree(dev_c);
}

