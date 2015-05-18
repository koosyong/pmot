#include "particlepose.h"
#include "csgpu.h"
#include <Eigen/Geometry>
using namespace pcl;


Particlepose::Particlepose()
{
    ros::param::get("/dhri/multipleObjectTracking/gpu/usegpu", usegpu);
    ros::param::get("/dhri/multipleObjectTracking/gpu/particlenumber", m_particlenum);
    m_lastFinalParticle.zero();
    m_lastlastFinalParticle.zero();
    m_lastlastlastFinalParticle.zero();
//    m_particlenum =  256;
    m_cnt = -1;
    m_internalCounter=0;
    m_vel.zero();
    m_particles.resize(m_particlenum);
    initSample();
}


Particlepose::~Particlepose()
{

}

void Particlepose::setdata(CloudPtr scene, CloudPtr model, int cnt)
{
    m_lastlastlastFinalParticle = m_lastlastFinalParticle;
    m_lastlastFinalParticle = m_lastFinalParticle;
    m_lastFinalParticle = m_finalParticle;
    m_vel = m_lastFinalParticle - m_lastlastFinalParticle;
    m_acc = m_vel - (m_lastlastFinalParticle - m_lastlastlastFinalParticle);
//    m_scene.reset(new Cloud);
//    m_model.reset(new Cloud);
//    m_modelOrig.reset(new Cloud);
//    m_sceneN.reset(new CloudN);
//    m_modelN.reset(new CloudN);
//    m_kdtreeSceneN.reset(new pcl::KdTreeFLANN<PointNT>);

//    if (cnt != m_cnt+1){
//        pcl::compute3DCentroid(*model, m_modelCentroid);
//    }
//    m_cnt = cnt;

//    vector<int> indices;
//    pcl::removeNaNFromPointCloud(*scene, *scene, indices);
//    pcl::removeNaNFromPointCloud(*model, *model, indices);

//    //modify scene//////////////////////////////////////////////////////////////////
//    std::vector<int> indexes(1);
//    std::vector<float> sqDists(1);
//    pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
//    kdtree->setInputCloud(model);
//    for (size_t j = 0; j < scene->points.size(); j++)
//    {
//        kdtree->nearestKSearchT(scene->points[j], 1, indexes,sqDists);
//        if (sqrt(sqDists[0]) < 0.035f){
//            PointT pttemp = scene->points[j];
//            pttemp.x = pttemp.x - m_modelCentroid[0];
//            pttemp.y = pttemp.y - m_modelCentroid[1];
//            pttemp.z = pttemp.z - m_modelCentroid[2];
//            m_scene->points.push_back(pttemp);
//        }

//    }
//    cout<<"m_scene size "<<m_scene->points.size()<<endl;
//    //modify model/////////////////////////////////////////////////////////////////////
////    kdtree.reset(new pcl::KdTreeFLANN<PointT>);
////    kdtree->setInputCloud(scene);
//    for (size_t i=0; i<model->points.size(); i++){
////        kdtree->nearestKSearchT(model->points[i], 1, indexes,sqDists);
//        PointT pttemp = model->points[i];
//        pttemp.x = pttemp.x - m_modelCentroid[0];
//        pttemp.y = pttemp.y - m_modelCentroid[1];
//        pttemp.z = pttemp.z - m_modelCentroid[2];

////        int rand_ = rand() % 100;
////        if (rand_< 4){
//            m_model->points.push_back(pttemp);

//        m_modelOrig->points.push_back(pttemp);


//    }
//    cout<<"m_model size "<<m_model->points.size()<<endl;





    m_scene.reset(new Cloud);
    m_model.reset(new Cloud);
    if (cnt > m_cnt+1){
        m_internalCounter=0;

        initSample();
        m_modelOrig.reset(new Cloud);
        *m_model=*model;
        pcl::compute3DCentroid(*model, m_modelCentroid);
        for (size_t i=0; i<model->points.size(); i++){
    //        kdtree->nearestKSearchT(model->points[i], 1, indexes,sqDists);
            PointT pttemp = model->points[i];
            pttemp.x = pttemp.x - m_modelCentroid[0];
            pttemp.y = pttemp.y - m_modelCentroid[1];
            pttemp.z = pttemp.z - m_modelCentroid[2];
            m_modelOrig->points.push_back(pttemp);
        }
        for (size_t i=0; i<m_particlenum; i++){
            m_particles[i].x = m_particles[i].x+ m_modelCentroid[0];
            m_particles[i].y = m_modelCentroid[1];
            m_particles[i].z = m_modelCentroid[2];

        }
    }
    if (m_internalCounter  < 4){
        m_vel.zero();
        m_acc.zero();
    }
    m_internalCounter++;
    m_cnt = cnt;

    //modify scene//////////////////////////////////////////////////////////////////
    std::vector<int> indexes(1);
    std::vector<float> sqDists(1);
    pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
    kdtree->setInputCloud(model);
    for (size_t j = 0; j < scene->points.size(); j++)
    {
        kdtree->nearestKSearchT(scene->points[j], 1, indexes,sqDists);
        if (sqrt(sqDists[0]) < 0.035f){
            PointT pttemp = scene->points[j];
            m_scene->points.push_back(pttemp);
        }

    }
    cout<<"m_scene size "<<m_scene->points.size()<<endl;
    cout<<"m_model size "<<m_modelOrig->points.size()<<endl;



    if (!usegpu){
        m_sceneN.reset(new CloudN);
        m_modelN.reset(new CloudN);
        m_kdtreeSceneN.reset(new pcl::KdTreeFLANN<PointNT>);

        NormalEstimationOMP<PointT, PointNT> ne;
        ne.setNumberOfThreads (8);
    //    ne.setKSearch(10);
        ne.setRadiusSearch(0.02f);
        ne.setViewPoint (1.f, 0.f, 1.f);
        ne.setInputCloud (m_modelOrig);
        ne.compute (*m_modelN);
        ne.setInputCloud (m_scene);
        ne.compute (*m_sceneN);
        for (size_t i=0; i<m_modelN->points.size(); i++){
            m_modelN->points[i].x =  m_modelOrig->points[i].x;
            m_modelN->points[i].y =  m_modelOrig->points[i].y;
            m_modelN->points[i].z =  m_modelOrig->points[i].z;
        }
        for (size_t i=0; i<m_sceneN->points.size(); i++){
            m_sceneN->points[i].x =  m_scene->points[i].x;
            m_sceneN->points[i].y =  m_scene->points[i].y;
            m_sceneN->points[i].z =  m_scene->points[i].z;
        }
        m_kdtreeSceneN->setInputCloud(m_sceneN);
    }
}



CloudPtr Particlepose::getResult()
{
    float vardist=0.005f, varang=0.5f;
    for(int i=0; i<1; i++){
        double lastT = pcl::getTime();
        resample(vardist, varang);
        vardist /=1.5f; varang /=1.5f;
        cout<<"resample time : "<<pcl::getTime()-lastT<<endl;
        lastT = pcl::getTime();
        weight();
        cout<<"weight time : "<<pcl::getTime()-lastT<<endl;
    }


    //======particle swarm optimiazation===========
//    float vardist=0.005f, varang=0.1f;
//    resample(vardist, varang);
//    std::vector<tracking::ParticleXYZRPY> lastSpeed;lastSpeed.resize(m_particlenum);
//    std::vector<tracking::ParticleXYZRPY> bestSpeed;bestSpeed.resize(m_particlenum);
//    std::vector<float> bestweight;bestweight.resize(m_particlenum);

//    for (size_t i =0; i<m_particlenum; i++){
//        lastSpeed[i].zero();bestSpeed[i].zero();bestweight[i]=0.f;
//    }
//    float c1=2.8; float c2=1.3; float K=2.f/ fabs(2-(c1+c2)-sqrt((c1+c2)*(c1+c2)-4.f*(c1+c2)));
//    for(int i=0; i<10; i++){
//        weight();
//        for (size_t j=0; j<m_particlenum; j++){
//            if (m_particles[j].weight>bestweight[j]) {
//                bestweight[j] = m_particles[j].weight;  bestSpeed[j] = m_particles[j];
//            }
//            float rand_1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
//            float rand_2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
//            lastSpeed[j] = (lastSpeed[j] + (bestSpeed[j] - m_particles[j])*c1*rand_1 +
//                            (m_lastFinalParticle-m_particles[j])*c2*rand_2 )*K;
//            m_particles[j] = m_particles[j] +  lastSpeed[j];
//        }
//    }


//    Eigen::Affine3f finaltrans = m_finalParticle.toEigenMatrix();
//    cout<<"finaltrans is : " <<m_finalParticle<<endl;
//    transformPointCloud(*m_modelOrig, *m_modelOrig, finaltrans);
//    for (size_t i=0; i<m_modelOrig->points.size(); i++){
//        m_modelOrig->points[i].x += m_modelCentroid[0];
//        m_modelOrig->points[i].y += m_modelCentroid[1];
//        m_modelOrig->points[i].z += m_modelCentroid[2];
//    }

//    m_modelCentroid = finaltrans*m_modelCentroid;
//    return m_modelOrig;

    if (!(m_finalParticle.x==m_finalParticle.x && m_finalParticle.y==m_finalParticle.y && m_finalParticle.z == m_finalParticle.z &&
          m_finalParticle.roll == m_finalParticle.roll && m_finalParticle.pitch == m_finalParticle.pitch && m_finalParticle.yaw == m_finalParticle.yaw))
        m_finalParticle = m_lastFinalParticle;
    Eigen::Affine3f finaltrans = m_finalParticle.toEigenMatrix();
    cout<<"finaltrans is : " <<m_finalParticle<<endl;
    CloudPtr out (new Cloud);
    transformPointCloud(*m_modelOrig, *out, finaltrans);
    return out;



}


void Particlepose::initSample()
{
    for (size_t i=0; i<m_particlenum; i++){
        tracking::ParticleXYZRPY temp;
        temp.zero();
        m_lastspeed.push_back(temp);
        m_bestspeed.push_back(temp);
        temp.weight = 1.f/m_particlenum;
        temp = sampleWithVar(temp, 0.008f, 0.8f); // dist in m, ang in degrees
        m_particles[i]=temp;
    }
}

void Particlepose::resample(float varDist, float varAng )
{
    double probabilities[m_particlenum];
    for (size_t i=0; i<m_particlenum; i++){
        probabilities[i] = m_particles[i].weight;
    }
    std::vector<double> cumulative;
    std::partial_sum(&probabilities[0], &probabilities[0] + m_particlenum,
                        std::back_inserter(cumulative));
    boost::uniform_real<> dist(0, cumulative.back());
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(m_gen, dist);

    std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();

    std::vector<tracking::ParticleXYZRPY> templist;
    templist = m_particles;

    float motionratio;
    ros::param::get("/dhri/multipleObjectTracking/gpu/motionratio", motionratio);
    int motionnum = (int)floor(m_particlenum * motionratio);
    cout<<"motionnum: "<<motionnum<<endl;
    tracking::ParticleXYZRPY acc = m_lastFinalParticle-m_lastlastFinalParticle;
    for (size_t i=0; i<motionnum; i++){
        int index = std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();
        float rand_1 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
//        float rand_2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
//        templist[i] = m_particles[index]*rand_1*2.f+acc*rand_2*2.f;
//        templist[i] = m_lastFinalParticle* (0.5f+rand_1)+acc*rand_2*0.1f;
        templist[i] = sampleWithVar(m_particles[index] + m_vel, varDist, varAng); // dist in m, ang in degrees
//        templist[i] = sampleWithVar(m_lastFinalParticle +acc*0.3f, varDist, varAng);
    }
    for (size_t i=motionnum; i<m_particlenum; i++){
//        templist[i].zero();
//        templist[i]=sampleWithVar( templist[i], 0.01f, 1.f); // dist in m, ang in degrees
        int index = std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin();
        templist[i] = sampleWithVar(m_particles[index], varDist, varAng); // dist in m, ang in degrees
    }
    m_particles = templist;


//    for (size_t i=0; i<m_particlenum; i++){
//        float rand_ = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
////        m_lastspeed[i] = (m_lastspeed[i] + (m_lastFinalParticle - m_lastlastFinalParticle)*5.f*rand_)*0.4f;
//        m_lastspeed[i] = (m_lastFinalParticle - m_lastlastFinalParticle)*0.2f +
//                (m_lastFinalParticle - m_particles[i])*0.1f;
//        m_particles[i] = m_particles[i] + m_lastspeed[i];
//    }

}

void Particlepose::weight()
{
    double lastT = pcl::getTime();

    if(!usegpu){
        minDist = 1e12; maxDist=-1e12;
        for(size_t i=0; i<m_particlenum; i++){
            lastT = pcl::getTime();
            CloudNPtr thisModel(new CloudN);
            float ratio = 1.f;
            int ratiosize = (int) floor(m_modelN->points.size()*ratio);
            for(size_t j=0; j<ratiosize; j++){
                int rand_ = rand() % m_modelN->points.size();
                    thisModel->points.push_back(m_modelN->points.at(j));
            }
            transformPointCloudWithNormals(*thisModel, *thisModel, m_particles[i].toEigenMatrix());
//            cout<<"transformed thismodel"<<endl;
            float thisDist = 0.f;
            int datasize = thisModel->points.size();
//            std::vector<int> indexes(1);
//            std::vector<float> sqDists(1);

            for (size_t j = 0; j < thisModel->points.size(); j++)
            {
                PointNT pt = thisModel->points.at(j);
                std::vector<int> indexes(1);
                std::vector<float> sqDists(1);
                m_kdtreeSceneN->nearestKSearchT(pt, 1, indexes,sqDists);
//                if ((m_kdtreeSceneN->radiusSearch(pt,0.02f,indexes,sqDists))>0){
//                    float distemp = 0.f;
//                    for (size_t k=0; k<indexes.size(); k++){
//                             distemp = distemp + 1.f-sqrt(sqDists[k])/0.02f;
//                         }
//                    thisDist +=distemp;
//                }


                PointNT nearestPt = m_sceneN->points.at(indexes[0]);
                float dist = (pt.getVector3fMap() - nearestPt.getVector3fMap()).norm();
    //            if (dist<0.02f){
                    dist = 1.f* min(dist/ 0.02f,1.f);
                    thisDist += 1.f-dist;
                    float normaldist = 1.f-acos(fabs(pt.getNormalVector3fMap().dot(nearestPt.getNormalVector3fMap())))
                            /1.5708f;
                    if (normaldist == normaldist){  //avoid nan
                         thisDist += 1.f-normaldist;
                    }else{
                        thisDist += 1.f-1.f;
                    }
                    float cdist = colordist(pt, nearestPt);
                    thisDist += 1.f-cdist;
    //            }else{
    //                thisDist += 5.5f;
    ////                datasize -= 1;
    //            }

            }



//            if(datasize>0)
//               thisDist /= datasize;
//            else
//                thisDist = 5.f;
            m_particles[i].weight = thisDist;
            if (m_particles[i].weight > maxDist)
                maxDist = m_particles[i].weight;
            if (m_particles[i].weight < minDist)
                minDist = m_particles[i].weight;

            cout<<"compute feature one particle time : "<<pcl::getTime()-lastT<<endl;

       }  //for(size_t i=0; i<m_particlenum; i++)


    } else{


//        CsFeature sceneF;
//        csest.initBox(m_scene);
//        sceneF = csest.compute(m_scene);
        CsFeature  m_modelF;
        csest.initBox(m_modelOrig);
        double lastT = pcl::getTime();
        m_modelF = csest.compute(m_modelOrig);
        cout<<"initialize weighting time : "<<pcl::getTime()-lastT<<endl;
        minDist = 1e10; maxDist=-1e10;
        std::vector<float> partweights = weightGPU (csest.m_gridsize, csest.m_xNr, csest.m_yNr,  csest.m_zNr,
                                      csest.m_boxMin, csest.m_boxMax, m_modelF.hist);
        minDist = 1e10; maxDist=-1e10;

        for(size_t i=0; i<m_particlenum; i++){
    //        cout<<i<<"th weight is: "<<partweights[i]<<endl;
            m_particles[i].weight = partweights[i];

            if (m_particles[i].weight > maxDist)
                maxDist = m_particles[i].weight;
            if (m_particles[i].weight < minDist)
                minDist = m_particles[i].weight;
        }

    }







//    CsFeatureEstimation csest;
//    CsFeature sceneF;
//    csest.initBox(m_scene);
//    sceneF = csest.compute(m_scene);
////    cout<<"initialize weighting time : "<<pcl::getTime()-lastT<<endl;
//    minDist = 1e10; maxDist=-1e10;
//    for(size_t i=0; i<m_particlenum; i++){
//        CloudPtr temp (new Cloud);
////        tracking::ParticleXYZRPY inversepart; inversepart.zero();
////        inversepart = inversepart - m_particles[i];
//        transformPointCloud(*m_model,*temp, m_particles[i].toEigenMatrix());
////        cout<<"transform time : "<<pcl::getTime()-lastT<<endl;
//        lastT = pcl::getTime();
//        CsFeature  thisF = csest.compute(temp);
////        cout<<"compute feature time : "<<pcl::getTime()-lastT<<endl;

//        m_particles[i].weight = sceneF.getLongDist(thisF);
////        cout<<"dist of "<<i<<"th particle: "<< m_particles[i].weight <<endl;

//        if (m_particles[i].weight > maxDist)
//            maxDist = m_particles[i].weight;
//        if (m_particles[i].weight < minDist)
//            minDist = m_particles[i].weight;
//    }



//    CsFeatureEstimation csest;
//    CsFeature modelF;
//    csest.initBox(m_model);
//    modelF = csest.compute(m_model);
//    cout<<"initialize weighting time : "<<pcl::getTime()-lastT<<endl;

//    minDist = 1e10; maxDist=-1e10;
//    for(size_t i=0; i<m_particlenum; i++){
//        CloudPtr temp (new Cloud);
//        tracking::ParticleXYZRPY inversepart; inversepart.zero();
//        inversepart = inversepart - m_particles[i];
//        Eigen::Affine3f transl=
//                getTransformation(0.f,0.f,0.f,inversepart.roll,inversepart.pitch,inversepart.yaw);
//        Eigen::Affine3f rot=
//                getTransformation(inversepart.x,inversepart.y,inversepart.z,0.f,0.f,0.f);
//        Eigen::Affine3f invtrans = rot *transl;
//        transformPointCloud(*m_scene,*temp, invtrans);
//        lastT = pcl::getTime();
//        CsFeature  thisF = csest.compute(temp);
//        m_particles[i].weight = modelF.getLongDist(thisF);
//        cout<<i<<"th weight "<<m_particles[i].weight<<endl;

//        if (m_particles[i].weight > maxDist)
//            maxDist = m_particles[i].weight;
//        if (m_particles[i].weight < minDist)
//            minDist = m_particles[i].weight;
//    }

//    Vector4f min, max;
//    int* test= festGPU(0.2, 4, 10 ,10,
//                                  min, max,  m_model);
//    for (int i=0; i<50; i++){
//        cout<<i<<": "<<test[i]<<";   ";
//    }


    ///////////////////////////////normarlize////////////////////////////////////
    float weightSum = 0.f;
    for (size_t i=0; i<m_particlenum; i++){
        float value;
        if(usegpu){
             value= pow((1.f-(m_particles[i].weight - minDist)/(maxDist-minDist)), 1)+0.000001f;
        }else{
            value = pow((1.f-(m_particles[i].weight - minDist)/(maxDist-minDist)), 1)+0.000001f;
//            value= pow((1.f-(m_particles[i].weight - minDist)/(maxDist-minDist)), 1)+0.000001f;
        }
//        m_particles[i].weight =1.f-value;
        m_particles[i].weight = min(exp(- 25.f * value),1.f);
//        if (m_particles[i].weight<0.8f) {m_particles[i].weight=0.f;}
        if (minDist == 1e10){m_particles[i].weight=1.f;}
        weightSum += m_particles[i].weight;
    }
//    cout<<"particle weights: [";
    for (size_t i=0; i<m_particlenum; i++){
//        cout<<m_particles[i].weight<<", ";
        m_particles[i].weight = m_particles[i].weight/weightSum*10000.f;
    }
//    cout<<" ] "<<endl;
    m_finalParticle.zero();

     Quaternion<float> finalq; finalq=  Quaternion<float>::Identity();
    for (size_t i=0; i<m_particlenum; i++){
        Quaternion<float> q;
        Eigen::AngleAxis<float> aaX(m_particles[i].roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxis<float> aaY(m_particles[i].pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxis<float> aaZ(m_particles[i].yaw, Eigen::Vector3f::UnitZ());

        q = aaZ * aaY * aaX;
        finalq = Quaternion<float>(finalq.w()+q.w()*m_particles[i].weight, finalq.x()+q.x()*m_particles[i].weight,
                                   finalq.y()+q.y()*m_particles[i].weight, finalq.z()+q.z()*m_particles[i].weight);
        m_finalParticle = m_finalParticle + m_particles[i] * m_particles[i].weight;
    }
    m_finalParticle = m_finalParticle*(1.f/10000.f);
    finalq = Quaternion<float>(finalq.w()*(1.f/10000.f), finalq.x()*(1.f/10000.f),
                               finalq.y()*(1.f/10000.f), finalq.z()*(1.f/10000.f));
    finalq.normalize();
    Affine3f rot(finalq);

    pcl::getEulerAngles(rot, m_finalParticle.roll, m_finalParticle.pitch, m_finalParticle.yaw);
    if (!(m_finalParticle.x==m_finalParticle.x && m_finalParticle.y==m_finalParticle.y && m_finalParticle.z == m_finalParticle.z &&
          m_finalParticle.roll == m_finalParticle.roll && m_finalParticle.pitch == m_finalParticle.pitch && m_finalParticle.yaw == m_finalParticle.yaw))
        m_finalParticle = m_lastFinalParticle;
 }

tracking::ParticleXYZRPY Particlepose::sampleWithVar(tracking::ParticleXYZRPY part, float varDist, float varAng)
{
    tracking::ParticleXYZRPY temp;
//    temp.x = tracking::sampleNormal(part.x, pow(varDist, 2) );
//    temp.y = tracking::sampleNormal(part.y, pow(varDist, 2) );
//    temp.z = tracking::sampleNormal(part.z, pow(varDist, 2) );
//    temp.roll  = tracking::sampleNormal(part.roll,  pow(varAng/180.f*M_PI, 2) );
//    temp.pitch = tracking::sampleNormal(part.pitch, pow(varAng/180.f*M_PI, 2) );
//    temp.yaw   = tracking::sampleNormal(part.yaw ,  pow(varAng/180.f*M_PI, 2) );

    // adaptive variance
    temp.x = tracking::sampleNormal(part.x, pow(fabs(m_vel.x)+0.003f, 2) );
    temp.y = tracking::sampleNormal(part.y, pow(fabs(m_vel.y)+0.003f, 2) );
    temp.z = tracking::sampleNormal(part.z, pow(fabs(m_vel.z)+0.003f, 2) );
    temp.roll  = tracking::sampleNormal(part.roll,  pow(fabs(m_vel.roll)+0.5f/180.f*M_PI, 2) );
    temp.pitch = tracking::sampleNormal(part.pitch, pow(fabs(m_vel.pitch)+0.5f/180.f*M_PI, 2) );
    temp.yaw   = tracking::sampleNormal(part.yaw ,  pow(fabs(m_vel.yaw)+0.5f/180.f*M_PI, 2) );

    temp.weight = 0.f;
    return temp;
}

float Particlepose::distfeature(VFHSignature308 f1, VFHSignature308 f2)
{
    float dist = 0.f;
    for(size_t i=0; i<f1.descriptorSize(); i++){
        dist += pow( (f1.histogram[i] - f2.histogram[i]), 2);
    }
    return sqrt(dist);
}

CloudPtr Particlepose::toCloud()
{
    CloudPtr out(new Cloud);
    for (size_t i =0; i<m_particlenum; i++){
        PointT pt;
        pt.x=m_particles[i].x;  pt.y=m_particles[i].y;  pt.z=m_particles[i].z;
        pt.r=255; pt.g=255-255*(m_particles[i].weight*(1.f/10000.f)*float(m_particlenum)); pt.b=pt.g;
//        if(m_particles[i].weight > (0.8*10000.f/float(m_particlenum))) {pt.g=0; pt.b=0;}
        out->points.push_back(pt);
    }
    return out;
}



std::vector<float> Particlepose::weightGPU(float gridsize, int xnr, int ynr, int znr,
                              Vector4f min, Vector4f max, std::vector<float> refhist)
{
    CSGPU gpuest(gridsize, xnr, ynr, znr, m_particlenum, m_scene->points.size());
//    gpuest.cloudSize = cloud->points.size();
    gpuest.minPt.x=min[0];
    gpuest.minPt.y=min[1];
    gpuest.minPt.z=min[2];
    gpuest.maxPt.x=max[0];
    gpuest.maxPt.y=max[1];
    gpuest.maxPt.z=max[2];
    float h,s,v;
    for (size_t i=0; i<m_scene->points.size(); i++){
        PointT pt = m_scene->points[i];
        gpuest.cloudpos[i].x = pt.x;
        gpuest.cloudpos[i].y = pt.y;
        gpuest.cloudpos[i].z = pt.z;
        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
        gpuest.cloudhsv[i].x = h;
        gpuest.cloudhsv[i].y = s;
        gpuest.cloudhsv[i].z = v;
    }
//    for (size_t i=0; i<m_model->points.size(); i++){
//        PointT pt = m_model->points[i];
//        gpuest.cloudpos[i].x = pt.x;
//        gpuest.cloudpos[i].y = pt.y;
//        gpuest.cloudpos[i].z = pt.z;
//        rgbTohsv(pt.r, pt.g, pt.b, h, s, v);
//        gpuest.cloudhsv[i].x = h;
//        gpuest.cloudhsv[i].y = s;
//        gpuest.cloudhsv[i].z = v;
//    }

    for (size_t i=0; i<m_particlenum; i++){
        Affine3f temp = m_particles[i].toEigenMatrix().inverse();
        float x_,y_,z_,roll_,pitch_,yaw_;
        pcl::getTranslationAndEulerAngles (temp,x_,y_,z_,roll_,pitch_,yaw_);
        gpuest.partpos[i].x = x_;
        gpuest.partpos[i].y = y_;
        gpuest.partpos[i].z = z_;
        gpuest.partrot[i].x = roll_;
        gpuest.partrot[i].y = pitch_;
        gpuest.partrot[i].z = yaw_;
    }
//    for (size_t i=0; i<m_particlenum; i++){
//        gpuest.partpos[i].x = m_particles[i].x;
//        gpuest.partpos[i].y = m_particles[i].y;
//        gpuest.partpos[i].z = m_particles[i].z;
//        gpuest.partrot[i].x = m_particles[i].roll;
//        gpuest.partrot[i].y = m_particles[i].pitch;
//        gpuest.partrot[i].z = m_particles[i].yaw;
//    }
    for (size_t i=0; i<refhist.size(); i++){
        gpuest.refhist[i] = refhist[i];
    }
    return (gpuest.compute());
}

std::vector<float> Particlepose::getPatchScore(CloudPtr cloud){
    CloudPtr cloud_ (new Cloud);
    std::vector<float> scores;
    transformPointCloud(*cloud,*cloud_,m_finalParticle.toEigenMatrix().inverse());
    for (size_t i=0; i<cloud_->points.size(); i++){
        scores.push_back(csest.evalOnePoint(cloud_->points[i]));
    }
    return scores;
}


void Particlepose::updateModel(CloudPtr cloud)
{
    m_modelOrig.reset(new Cloud);
    transformPointCloud(*cloud, *m_modelOrig, m_finalParticle.toEigenMatrix().inverse());
}




