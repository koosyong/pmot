#include "csfeature.h"


CsFeature::CsFeature()
{
}

CsFeature::~CsFeature()
{
}

float CsFeature::getDist(CsFeature other)
{
//    float dist = 0.f;
//    double lastT = pcl::getTime();

//    for (size_t i=0; i<feature.size(); i++){
//        dist += feature[i].getDist(other.feature[i]);
////        cout<< dist<<endl;
//    }
////    cout<<"getdist time : "<<pcl::getTime()-lastT<<endl<<endl;

//    return dist;
}

float GausVcsh::getDist(GausVcsh other)
{
//    if (points.size()<2 && other.points.size()<2){
//        return 0.f;
//    }
    // distatnce calculation===========================
    double dist = 0;
    // color part
    for(size_t i=0; i<color.hist.size(); i++){
        dist += fabs(pow( (color.hist.at(i)-other.color.hist.at(i)), 1));
    }
//    dist = sqrt(dist);
//    dist *= 3.0;
//    float size1 = (float)this->points.size();
//    float size2 = (float)other.points.size();
//    float shapedist = fabs(size1-size2)/35.f;
//    dist += shapedist;
//    dist *= 3.f;
    // gaussian(shape) part
//    if (points.size()<2 || other.points.size()<2){
//        dist += 2.f;
//    }else{
//        Vector3d mean_;
//        Matrix3d cov_;
//        mean_ = shape.mean - other.shape.mean;
//        cov_ = shape.covariance + other.shape.covariance;
//        Matrix3d cov_inv = cov_.inverse();
//        double a = mean_.transpose()*cov_inv*mean_;
//        double gauss = (1./sqrt(pow(2*M_PI, 3)*cov_.determinant()))*exp(-0.5*a);
////        gauss = max(0.0, gauss);
////        gauss = min(1.0, gauss);
//        double shapedist = 2.0 - 2*gauss;
//        cout<<" shape dist: "<<shapedist<<endl;
//        cout<<" det: "<<cov_.determinant() <<endl;
//        cout<<" mean: "<<mean_ <<endl;
//        cout<<" m*covinv*m: "<<a<<endl;
//        cout<<" gauss "<<gauss<<endl<<endl;
////        float size1 = (float)this->points.size();
////        float size2 = (float)other.points.size();
////        float shapedist = fabs(size1-size2)/25.f;
////                cout<<" shape dist: "<<fabs(size1-size2)<<"  " <<shapedist<<endl;
//        if (shapedist ==shapedist){
//            dist += shapedist;
//        }else{
//            dist += 2.0;
//        }
//    }

    return dist;
}

void GausVcsh::generate(float gridsize)
{
    //shape part==========================================
//    for(size_t i=0; i<this->points.size(); i++){
//        Vector3d pos = this->points.at(i).getVector3fMap().cast <double> ();
//        pos = 1000.0 * pos;   //for computing, otherwise cov gets too small
//        this->shape.mean += pos;
////        cout<<"update mean";
//    }
//    if (points.size()>0){
//        this->shape.mean /= (double)this->points.size();
//    }
//    for(int i=0;i<this->points.size();i++){
//        float demean_xx, demean_yy, demean_zz, demean_xy, demean_xz, demean_yz;
//        Vector3d demean;
//        demean[0] = this->points.at(i).x - this->shape.mean[0];
//        demean[1] = this->points.at(i).y - this->shape.mean[1];
//        demean[2] = this->points.at(i).z - this->shape.mean[2];
//        demean_xx = demean[0] * demean[0];
//        demean_yy = demean[1] * demean[1];
//        demean_zz = demean[2] * demean[2];
//        demean_xy = demean[0] * demean[1];
//        demean_xz = demean[0] * demean[2];
//        demean_yz = demean[1] * demean[2];

//        this->shape.covariance(0,0) += demean_xx;
//        this->shape.covariance(0,1) += demean_xy;
//        this->shape.covariance(0,2) += demean_xz;

//        this->shape.covariance(1,0) += demean_xy;
//        this->shape.covariance(1,1) += demean_yy;
//        this->shape.covariance(1,2) += demean_yz;

//        this->shape.covariance(2,0) += demean_xz;
//        this->shape.covariance(2,1) += demean_yz;
//        this->shape.covariance(2,2) += demean_zz;
//    }
//    if (points.size()>1){
//        this->shape.covariance /= (double)this->points.size();
//    }else{
//        this->shape.covariance(0,0) += 1e-5;
//        this->shape.covariance(1,1) += 1e-5;
//        this->shape.covariance(2,2) += 1e-5;
//    }

    double incrValue = 1.0/double(points.size());

//    //shape part new=======================================
//    for (size_t i=0; i<this->points.size(); i++){
//        PointT pt = this->points[i];
//        float weight1, weight2;
//        int ind1,ind2;
//        //x part
//        float halfsize=gridsize/2.0;
//        if(pt.x < halfsize){
//            ind1=0; ind2=1; weight2=pt.x/halfsize; weight1=1.f-weight2;
//        }else{
//            ind1=1; ind2=2; weight2=(pt.x-halfsize)/halfsize; weight1=1.f-weight2;
//        }
//        this->color.hist[16+ind1] += weight1*incrValue;
//        this->color.hist[16+ind2] += weight2*incrValue;
//        //y part
//        if(pt.y < halfsize){
//            ind1=0; ind2=1; weight2=pt.y/halfsize; weight1=1.f-weight2;
//        }else{
//            ind1=1; ind2=2; weight2=(pt.y-halfsize)/halfsize; weight1=1.f-weight2;
//        }
//        this->color.hist[19+ind1] += weight1*incrValue;
//        this->color.hist[19+ind2] += weight2*incrValue;
//        //z part
//        if(pt.z < halfsize){
//            ind1=0; ind2=1; weight2=pt.z/halfsize; weight1=1.f-weight2;
//        }else{
//            ind1=1; ind2=2; weight2=(pt.z-halfsize)/halfsize; weight1=1.f-weight2;
//        }
//        this->color.hist[22+ind1] += weight1*incrValue;
//        this->color.hist[22+ind2] += weight2*incrValue;
//    }

//    for (size_t i=0; i<this->points.size(); i++){
//        PointT pt = this->points[i];
//        float weight1, weight2;
//        float sqlength = 3.f *pow(gridsize,2.0);
//        int ind1,ind2;
//        float x=pt.x, y=pt.y, z=pt.z, x_=gridsize-pt.x, y_= gridsize-pt.y, z_=gridsize-pt.z;
//        //1111111111
//        weight1 = (x*x+y*y+z*z)/sqlength; weight2 = 1.f-weight1;
//        this->color.hist[8] += weight1*incrValue;
//        this->color.hist[9] += weight2*incrValue;
//        //2222222222
//        weight1 = (x_*x_+y*y+z*z)/sqlength; weight2 = 1.f-weight1;
//        this->color.hist[10] += weight1*incrValue;
//        this->color.hist[11] += weight2*incrValue;
//        //3333333333
//        weight1 = (x*x+y_*y_+z*z)/sqlength; weight2 = 1.f-weight1;
//        this->color.hist[12] += weight1*incrValue;
//        this->color.hist[13] += weight2*incrValue;
//        //4444444444
//        weight1 = (x_*x_+y_*y_+z*z)/sqlength; weight2 = 1.f-weight1;
//        this->color.hist[14] += weight1*incrValue;
//        this->color.hist[15] += weight2*incrValue;
//    }

    incrValue *= 7.0;
    //color part========================================
    for (size_t i=0; i<this->points.size(); i++){
        PointT pt = this->points[i];
        float h,s,v;
        rgbTohsv(this->points.at(i).r, this->points.at(i).g, this->points.at(i).b, h, s, v);
        int grayindex;
        float weightgray, weightcolor;
        int hindex1,hindex2;
        float hweight1, hweight2;
        h = h*360.0f;
        if(h>=0   && h <=60)  {hindex1=0; hindex2=1; hweight2= h/60.0f;          hweight1=1.0f -hweight2;}
        if(h>60  && h <=120)  {hindex1=1; hindex2=2; hweight2= (h-60.0f)/60.0f;  hweight1=1.0f -hweight2;}
        if(h>120 && h <=180)  {hindex1=2; hindex2=3; hweight2= (h-120.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>180 && h <=240)  {hindex1=3; hindex2=4; hweight2= (h-180.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>240 && h <=300)  {hindex1=4; hindex2=5; hweight2= (h-240.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>300 && h <=360)  {hindex1=5; hindex2=0; hweight2= (h-300.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(v<0.5) {grayindex = 6;}
        if(v>=0.5){grayindex = 7;}
        weightcolor = std::pow(s,(0.14f * std::pow(1.0f/v,0.9f)));
        weightgray  = 1.0f - weightcolor;
        if( v<0.1f){weightcolor=0.0f; weightgray=1.0f;}

        float weight1, weight2;
        float sqlength = 3.f *pow(gridsize,2.0);
        int ind1,ind2;
        float x=pt.x, y=pt.y, z=pt.z, x_=gridsize-pt.x, y_= gridsize-pt.y, z_=gridsize-pt.z;
        //1111111111
//        weight1 = 1.f-(x*x+y*y+z*z)/sqlength; weight2 = 1.f-(x_*x_+y_*y_+z_*z_)/sqlength;
        weight1 = x/gridsize; weight2 = 1.f-weight1;
        this->color.hist[hindex1*6 + 0] += hweight1*weight1*incrValue;
        this->color.hist[hindex1*6 + 1] += hweight1*weight2*incrValue;
        this->color.hist[hindex2*6 + 0] += hweight2*weight1*incrValue;
        this->color.hist[hindex2*6 + 1] += hweight2*weight2*incrValue;
        this->color.hist[grayindex*6 + 0] += weightgray*weight1*incrValue;
        this->color.hist[grayindex*6 + 1] += weightgray*weight2*incrValue;
        //2222222222
//        weight1 = 1.f-(x_*x_+y*y+z*z)/sqlength; weight2 = 1.f-(x*x+y_*y_+z_*z_)/sqlength;
        weight1 = y/gridsize; weight2 = 1.f-weight1;
        this->color.hist[hindex1*6 + 2] += hweight1*weight1*incrValue;
        this->color.hist[hindex1*6 + 3] += hweight1*weight2*incrValue;
        this->color.hist[hindex2*6 + 2] += hweight2*weight1*incrValue;
        this->color.hist[hindex2*6 + 3] += hweight2*weight2*incrValue;
        this->color.hist[grayindex*6 + 2] += weightgray*weight1*incrValue;
        this->color.hist[grayindex*6 + 3] += weightgray*weight2*incrValue;
        //3333333333
//        weight1 = 1.f-(x*x+y_*y_+z*z)/sqlength; weight2 = 1.f-(x_*x_+y*y+z_*z_)/sqlength;
        weight1 = z/gridsize; weight2 = 1.f-weight1;
        this->color.hist[hindex1*6 + 4] += hweight1*weight1*incrValue;
        this->color.hist[hindex1*6 + 5] += hweight1*weight2*incrValue;
        this->color.hist[hindex2*6 + 4] += hweight2*weight1*incrValue;
        this->color.hist[hindex2*6 + 5] += hweight2*weight2*incrValue;
        this->color.hist[grayindex*6 + 4] += weightgray*weight1*incrValue;
        this->color.hist[grayindex*6 + 5] += weightgray*weight2*incrValue;
//        //4444444444
//        weight1 = 1.f-(x_*x_+y_*y_+z*z)/sqlength; weight2 = 1.f-(x*x+y*y+z_*z_)/sqlength;
//        this->color.hist[hindex1*8 + 6] += hweight1*weight1*incrValue;
//        this->color.hist[hindex1*8 + 7] += hweight1*weight2*incrValue;
//        this->color.hist[hindex2*8 + 6] += hweight2*weight1*incrValue;
//        this->color.hist[hindex2*8 + 7] += hweight2*weight2*incrValue;
//        this->color.hist[grayindex*8 + 6] += weightgray*weight1*incrValue;
//        this->color.hist[grayindex*8 + 7] += weightgray*weight2*incrValue;

//        this->color.hist[hindex1] += weightcolor * hweight1 * incrValue;
//        this->color.hist[hindex2] += weightcolor * hweight2 * incrValue;
//        this->color.hist[grayindex] += weightgray * incrValue;
    }
}

CsFeatureEstimation::CsFeatureEstimation()
{
    ros::param::get("/dhri/multipleObjectTracking/gpu/gridsize", m_gridsize);
}

CsFeatureEstimation::~CsFeatureEstimation()
{
}


void CsFeatureEstimation::initBox(CloudPtr &cloud)
{
    pcl::getMinMax3D(*cloud, m_boxMin, m_boxMax);
    float extra = 0.0f;
    m_boxMin[0]-=extra; m_boxMin[1]-=extra; m_boxMin[2]-=extra;
    m_boxMax[0]+=extra; m_boxMax[1]+=extra; m_boxMax[2]+=extra;
    m_xNr = static_cast<int>( ceil((m_boxMax[0] - m_boxMin[0])/m_gridsize) );
    m_yNr = static_cast<int>( ceil((m_boxMax[1] - m_boxMin[1])/m_gridsize) );
    m_zNr = static_cast<int>( ceil((m_boxMax[2] - m_boxMin[2])/m_gridsize) );
    m_gridNr = m_xNr * m_yNr * m_zNr;

//    m_keySize = 20;
//    m_keypoints.reset(new Cloud);
//    for (size_t i=0; i<m_keySize; ++i){
//        int rand_ = rand() % cloud->points.size();
//        m_keypoints->points.push_back(cloud->points[rand_]);
//    }
}

CsFeature CsFeatureEstimation::compute(CloudPtr &cloud)
{
    outF.hist.clear();
//    outF.feature.resize(m_gridNr);
    double lastT = pcl::getTime ();

//    //add points to correspoinding grid
//    for (size_t i=0; i<cloud->points.size(); ++i){
//        PointT pt = cloud->points[i];
//        int xInd, yInd, zInd, theInd;
//        xInd = (int)floor( (pt.x - m_boxMin[0])/m_gridsize );
//        if((xInd >= m_xNr) || (xInd<0)) {continue;}
//        yInd = (int)floor( (pt.y - m_boxMin[1])/m_gridsize );
//        if((yInd >= m_yNr) || (yInd<0)) {continue;}
//        zInd = (int)floor( (pt.z - m_boxMin[2])/m_gridsize );
//        if((zInd >= m_zNr) || (zInd<0)) {continue;}
////        theInd = xInd*m_yNr*m_zNr + yInd*m_zNr + zInd;   //correct???

//        theInd = zInd*m_xNr*m_yNr + xInd*m_yNr + yInd;   //correct???

//        pt.x = pt.x - m_gridsize*(float)xInd - m_boxMin[0];
//        pt.y = pt.y - m_gridsize*(float)yInd - m_boxMin[1];
//        pt.z = pt.z - m_gridsize*(float)zInd - m_boxMin[2];

//        outF.feature[theInd].points.push_back(pt);
//    }
////    cout<<"assign grid time : "<<pcl::getTime()-lastT<<endl;
//    lastT = pcl::getTime();
//    //generate feature
//    for (int i=0; i<m_gridNr; i++){
////        cerr<<"gridnr "<<m_gridNr<<" generate "<<i<<"th "<<endl;
//        outF.feature[i].generate(m_gridsize);
//    }
////    cout<<"generate feature time : "<<pcl::getTime()-lastT<<endl;

    outF.hist.resize((m_xNr+1) * (m_yNr+1) * (m_zNr+1) * 8);
//    outF.hist.resize((m_xNr + m_yNr + m_zNr + 3) * 8);

    std::fill(outF.hist.begin(), outF.hist.end(), 0);
    for (size_t i=0; i<cloud->points.size(); ++i){
        PointT pt = cloud->points[i];
        int xInd, yInd, zInd, theInd;
        xInd = (int)floor( (pt.x - m_boxMin[0])/m_gridsize );
        if((xInd >= m_xNr) || (xInd<0)) {continue;}
        yInd = (int)floor( (pt.y - m_boxMin[1])/m_gridsize );
        if((yInd >= m_yNr) || (yInd<0)) {continue;}
        zInd = (int)floor( (pt.z - m_boxMin[2])/m_gridsize );
        if((zInd >= m_zNr) || (zInd<0)) {continue;}
        theInd = zInd*m_xNr*m_yNr + xInd*m_yNr + yInd;
        pt.x = pt.x - m_gridsize*(float)xInd - m_boxMin[0];
        pt.y = pt.y - m_gridsize*(float)yInd - m_boxMin[1];
        pt.z = pt.z - m_gridsize*(float)zInd - m_boxMin[2];

        float h,s,v;
        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
        int grayindex;
        float weightgray, weightcolor;
        int hindex1,hindex2;
        float hweight1, hweight2;
        h = h*360.0f;
        if(h>=0   && h <=60)  {hindex1=0; hindex2=1; hweight2= h/60.0f;          hweight1=1.0f -hweight2;}
        if(h>60  && h <=120)  {hindex1=1; hindex2=2; hweight2= (h-60.0f)/60.0f;  hweight1=1.0f -hweight2;}
        if(h>120 && h <=180)  {hindex1=2; hindex2=3; hweight2= (h-120.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>180 && h <=240)  {hindex1=3; hindex2=4; hweight2= (h-180.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>240 && h <=300)  {hindex1=4; hindex2=5; hweight2= (h-240.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(h>300 && h <=360)  {hindex1=5; hindex2=0; hweight2= (h-300.0f)/60.0f; hweight1=1.0f -hweight2;}
        if(v<0.5) {grayindex = 6;}
        if(v>=0.5){grayindex = 7;}
        weightcolor = std::pow(s,(0.14f * std::pow(1.0f/v,0.9f)));
        weightgray  = 1.0f - weightcolor;
        if( v<0.1f){weightcolor=0.0f; weightgray=1.0f;}

//        weightcolor=0.f; weightgray=1.f; grayindex=7;
        hweight1 = weightcolor * hweight1;
        hweight2 = weightcolor * hweight2;


//        int xPlusInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd;
//        int yPlusInd = zInd*m_xNr*m_yNr + xInd*m_yNr + (yInd+1);
//        int zPlusInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + yInd;

//        float weight1, weight2;
//        float incrValue =1.f;
//        weight2 = pt.x/m_gridsize; weight1 = 1.f-weight2; //weight2 *= weight2; weight1*=weight1;
//        outF.hist[hindex1 + 24*theInd + 0]     += hweight1*weight1*incrValue;
//        outF.hist[hindex1 + 24*xPlusInd + 0]   += hweight1*weight2*incrValue;
//        outF.hist[hindex2 + 24*theInd + 0]     += hweight2*weight1*incrValue;
//        outF.hist[hindex2 + 24*xPlusInd + 0]   += hweight2*weight2*incrValue;
//        outF.hist[grayindex + 24*theInd + 0]   += weightgray*weight1*incrValue;
//        outF.hist[grayindex + 24*xPlusInd + 0] += weightgray*weight2*incrValue;
//        //2222222222
////        weight1 = 1.f-(x_*x_+y*y+z*z)/sqlength; weight2 = 1.f-(x*x+y_*y_+z_*z_)/sqlength;
//        weight2 = pt.y/m_gridsize; weight1 = 1.f-weight2;    //      weight2 *= weight2; weight1*=weight1;
//        outF.hist[hindex1 + 24*theInd + 8]     += hweight1*weight1*incrValue;
//        outF.hist[hindex1 + 24*yPlusInd + 8]   += hweight1*weight2*incrValue;
//        outF.hist[hindex2 + 24*theInd + 8]     += hweight2*weight1*incrValue;
//        outF.hist[hindex2 + 24*yPlusInd + 8]   += hweight2*weight2*incrValue;
//        outF.hist[grayindex + 24*theInd + 8]   += weightgray*weight1*incrValue;
//        outF.hist[grayindex + 24*yPlusInd + 8] += weightgray*weight2*incrValue;
//        //3333333333
////        weight1 = 1.f-(x*x+y_*y_+z*z)/sqlength; weight2 = 1.f-(x_*x_+y*y+z_*z_)/sqlength;
//        weight2 = pt.z/m_gridsize; weight1 = 1.f-weight2;   //     weight2 *= weight2; weight1*=weight1;
//        outF.hist[hindex1 + 24*theInd + 16]     += hweight1*weight1*incrValue;
//        outF.hist[hindex1 + 24*zPlusInd + 16]   += hweight1*weight2*incrValue;
//        outF.hist[hindex2 + 24*theInd + 16]     += hweight2*weight1*incrValue;
//        outF.hist[hindex2 + 24*zPlusInd + 16]   += hweight2*weight2*incrValue;
//        outF.hist[grayindex + 24*theInd + 16]   += weightgray*weight1*incrValue;
//        outF.hist[grayindex + 24*zPlusInd + 16] += weightgray*weight2*incrValue;


        float x=pt.x, x_=m_gridsize-pt.x, y=pt.y, y_=m_gridsize-pt.y, z= pt.z, z_=m_gridsize-pt.z;
        float dist0 = sqrt(x*x + y*y +z*z);
        int xPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd;  float distxP = sqrt(x_*x_ + y*y +z*z) ;
        int xyPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1; float distxyP = sqrt(x_*x_ + y_*y_ +z*z);
        int xzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd; float distxzP = sqrt(x_*x_ + y*y +z_*z_);
        int xyzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1; float distxyzP = sqrt(x_*x_ + y_*y_ +z_*z_);
        int yPInd = zInd*m_xNr*m_yNr + xInd*m_yNr + (yInd+1); float distyP = sqrt(x*x + y_*y_ +z*z);
        int yzPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + (yInd+1); float distyzP = sqrt(x*x + y_*y_ +z_*z_);
        int zPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + yInd;  float distzP = sqrt(x*x + y*y +z_*z_);

        float incrValue =1.f; float weight; int offset; float bandwidth=m_gridsize;
        weight = max(0.f, (bandwidth-dist0))/m_gridsize;  offset = 8*theInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxP))/m_gridsize;  offset = 8*xPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxyP))/m_gridsize;  offset = 8*xyPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxzP))/m_gridsize;  offset = 8*xzPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distxyzP))/m_gridsize;  offset = 8*xyzPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distyP))/m_gridsize;  offset = 8*yPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distyzP))/m_gridsize;  offset = 8*yzPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;

        weight = max(0.f, (bandwidth-distzP))/m_gridsize;  offset = 8*zPInd;
        outF.hist[hindex1 + offset]     += hweight1*weight*incrValue;
        outF.hist[hindex2 + offset]     += hweight2*weight*incrValue;
        outF.hist[grayindex + offset]   += weightgray*weight*incrValue;
    }

    return outF;
}


float CsFeature::getLongDist(CsFeature other)
{
    float out=0.f;
    for(size_t i=0; i<other.hist.size(); i++){
        out += pow(fabs(this->hist[i] - other.hist[i]),2);
    }
    return out;
}

float CsFeatureEstimation::evalOnePoint(PointT pt)
{
    float h,s,v;
    rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
    int grayindex;
    float weightgray, weightcolor;
    int hindex1,hindex2;
    float hweight1, hweight2;


    h = h*360.0f;

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

    float x=pt.x, y=pt.y, z=pt.z;
    int xInd, yInd, zInd, theInd;
    xInd = (int)floorf( (x - m_boxMin[0])/m_gridsize );
    if(xInd >= m_xNr) {xInd=m_xNr;} if(xInd<0){xInd=0;}
    yInd = (int)floorf( (y - m_boxMin[1])/m_gridsize );
    if(yInd >= m_yNr) {yInd=m_yNr;} if(yInd<0){yInd=0;}
    zInd = (int)floorf( (z - m_boxMin[2])/m_gridsize );
    if(zInd >= m_zNr) {zInd=m_zNr;} if(zInd<0){zInd=0;}
    theInd = zInd*m_xNr*m_yNr + xInd*m_yNr + yInd;

    x = x - m_gridsize*(float)xInd - m_boxMin[0];
    y = y - m_gridsize*(float)yInd - m_boxMin[1];
    z = z - m_gridsize*(float)zInd - m_boxMin[2];

//        float numsafe=1000.f;
//        x *= numsafe; y*=numsafe; z*=numsafe; gridsize*=numsafe ;



    float x_=m_gridsize-pt.x, y_=m_gridsize-pt.y, z_=m_gridsize-pt.z;
    float dist0 = sqrt(x*x + y*y +z*z);
    int xPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd;  float distxP = sqrt(x_*x_ + y*y +z*z) ;
    int xyPInd = zInd*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1; float distxyP = sqrt(x_*x_ + y_*y_ +z*z);
    int xzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd; float distxzP = sqrt(x_*x_ + y*y +z_*z_);
    int xyzPInd = (zInd+1)*m_xNr*m_yNr + (xInd+1)*m_yNr + yInd+1; float distxyzP = sqrt(x_*x_ + y_*y_ +z_*z_);
    int yPInd = zInd*m_xNr*m_yNr + xInd*m_yNr + (yInd+1); float distyP = sqrt(x*x + y_*y_ +z*z);
    int yzPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + (yInd+1); float distyzP = sqrt(x*x + y_*y_ +z_*z_);
    int zPInd = (zInd+1)*m_xNr*m_yNr + xInd*m_yNr + yInd;  float distzP = sqrt(x*x + y*y +z_*z_);

    float incrValue =1.f; float weight; int offset;
    float bandwidth = m_gridsize;
    float gridsize3= 0.000001f;//gridsize*gridsize*gridsize;
//        weight = max(0.f, (bandwidth-dist0))/gridsize;
    weight = max(0.f, ((x_*y_*z_)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*theInd;
    float thisCorr = 0.f;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxP))/gridsize;
    weight = max(0.f, ((x*y_*z_)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*xPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxyP))/gridsize;
    weight = max(0.f, ((x*y*z_)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*xyPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxzP))/gridsize;
    weight = max(0.f, ((x*y_*z)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*xzPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distxyzP))/gridsize;
    weight = max(0.f, ((x*y*z)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*xyzPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distyP))/gridsize;
    weight = max(0.f, ((x_*y*z_)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*yPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distyzP))/gridsize;
    weight = max(0.f, ((x_*y*z)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*yzPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

//        weight = max(0.f, (bandwidth-distzP))/gridsize;
    weight = max(0.f, ((x_*y_*z)/gridsize3)); weight =pow (weight,3.f);
    offset = 8*zPInd;
    thisCorr += (outF.hist[hindex1 + offset] * hweight1*weight);
    thisCorr += (outF.hist[hindex2 + offset] * hweight2*weight);
    thisCorr += (outF.hist[grayindex + offset] * weightgray*weight);

    return thisCorr;
}

