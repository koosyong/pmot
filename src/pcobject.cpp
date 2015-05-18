#include "pcobject.h"
#include <math.h>
#include <iostream>
#define pi 3.141592

bool isDebug = 1;

PCObject::PCObject()
{
    isParamExist = 0;
    imftGaussians = NULL;
    gaussianTracks = NULL;

    topology_graph = NULL;
    topology_nodeMap = NULL;
    topology_edgeMap =  NULL;
    alpha = 1;
    th_edge = 0.80;

    filteredWeight = 0.;

    //    diffWeight = new double[10];
    //    for(int i=0;i<10;i++)
    //        diffWeight[i] = 0.;
    centroid.pos[0] = 0.;
    centroid.pos[1] = 0.;
    centroid.pos[2] = 0.;
    centroid.rgb[0] = 0.;
    centroid.rgb[1] = 0.;
    centroid.rgb[2] = 0.;

    state = NOGMM;
}

PCObject::PCObject(int _id)
    :id(_id)
{
    isParamExist = 0;
    imftGaussians = NULL;
    gaussianTracks = NULL;

    topology_graph = NULL;
    topology_nodeMap = NULL;
    topology_edgeMap = NULL;

    alpha = 1;
    th_edge = 0.80;

    filteredWeight = 0.;

    centroid.pos[0] = 0.;
    centroid.pos[1] = 0.;
    centroid.pos[2] = 0.;
    centroid.rgb[0] = 0.;
    centroid.rgb[1] = 0.;
    centroid.rgb[2] = 0.;
    //    diffWeight = new double[10];
    //    for(int i=0;i<10;i++)
    //        diffWeight[i] = 0.;

    state = NOGMM;
}

PCObject::~PCObject()
{
    if(imftGaussians != 0)  delete imftGaussians;
    if(topology_graph != 0) delete topology_graph;
    if(topology_nodeMap != 0) delete topology_nodeMap;
    if(topology_edgeMap != 0) delete topology_edgeMap;

    if(gaussianTracks != 0) delete gaussianTracks;
}

CloudPtr PCObject::toPointCloud()
{
    CloudPtr pccloud (new Cloud);
    for(int i=0; i<points.size(); i++){
        PointT pt;
        pt.x = (float)points.at(i).pos[0];
        pt.y = (float)points.at(i).pos[1];
        pt.z = (float)points.at(i).pos[2];

        pt.r = (uint8_t)points.at(i).rgb[0];
        pt.g = (uint8_t)points.at(i).rgb[1];
        pt.b = (uint8_t)points.at(i).rgb[2];
        pccloud->push_back(pt);
    }
    return pccloud;
}

CloudPtr PCObject::toPointCloud_model()
{
    if(points_model.size() == 0){
        return toPointCloud();
    }

    CloudPtr pccloud (new Cloud);
    for(int i=0; i<points_model.size(); i++){
        PointT pt;
        pt.x = (float)points_model.at(i).pos[0];
        pt.y = (float)points_model.at(i).pos[1];
        pt.z = (float)points_model.at(i).pos[2];

        pt.r = (uint8_t)points_model.at(i).rgb[0];
        pt.g = (uint8_t)points_model.at(i).rgb[1];
        pt.b = (uint8_t)points_model.at(i).rgb[2];
        pccloud->push_back(pt);
    }
    return pccloud;
}

void PCObject::insert(Point p)
{
    points.push_back(p);
    int n = points.size();
    centroid.dim = p.dim;
    centroid.pos = (centroid.pos * n + p.pos) / (n+1);
    if(centroid.dim == 6)
        centroid.rgb = (centroid.rgb * n + p.rgb) / (n+1);
}

void PCObject::insert_model(Point p)
{
    points_model.push_back(p);
    int n = points_model.size();
    centroid.dim = p.dim;
    centroid.pos = (centroid.pos * n + p.pos) / (n+1);
    if(centroid.dim == 6)
        centroid.rgb = (centroid.rgb * n + p.rgb) / (n+1);
}

void PCObject::setTransParam(vnl_vector<double> param)
{
    isParamExist = 1;
    trans_param = param;
}

Point PCObject::getCentroid()
{
    return centroid;

}

void PCObject::initGaussian(int dim)
{
    dimension = dim;
    gaussian.init(dim, points);
}


void PCObject::initialGMM(double _scale, double _percent)
{
    state = PRIORGMM;

    double lastT, nowT, computationT;
    if(isDebug) cout<<"initialGMM start"<<endl;
    lastT = pcl::getTime ();
    scale = _scale;
    percent = _percent;
    gmm.clear();
    int dim;
    int n = points.size();
    //    cout<<"# of points : "<<n<<endl;
    for(int i=0;i<n;i++){
        if(points.at(i).dim == 3){
            dim = 3;
            Gaussian gaussian(3);
            gaussian.mean[0] = points.at(i).pos[0];
            gaussian.mean[1] = points.at(i).pos[1];
            gaussian.mean[2] = points.at(i).pos[2];

            gaussian.covariance(0,0) = scale*scale;
            gaussian.covariance(0,1) = 0;
            gaussian.covariance(0,2) = 0;

            gaussian.covariance(1,0) = 0;
            gaussian.covariance(1,1) = scale*scale;
            gaussian.covariance(1,2) = 0;

            gaussian.covariance(2,0) = 0;
            gaussian.covariance(2,1) = 0;
            gaussian.covariance(2,2) = scale*scale;

            gaussian.weight = 1./(double)n;
            //        gaussian.weight = 1;
            gaussian.nPoint = 1;
            gaussian.cov_determinant = gaussian.covariance.determinant();
            gaussian.cov_inverse = gaussian.covariance.inverse();
            gaussian.initPrediction();
            gmm.push_back(gaussian);
        }
        if(points.at(i).dim == 6){
            dim = 6;
            Gaussian gaussian(6);

            gaussian.mean[0] = points.at(i).pos[0];
            gaussian.mean[1] = points.at(i).pos[1];
            gaussian.mean[2] = points.at(i).pos[2];
            gaussian.mean[3] = points.at(i).rgb[0];
            gaussian.mean[4] = points.at(i).rgb[1];
            gaussian.mean[5] = points.at(i).rgb[2];

            gaussian.covariance(0,0) = scale*scale;
            gaussian.covariance(0,1) = 0;
            gaussian.covariance(0,2) = 0;
            gaussian.covariance(0,3) = 0;
            gaussian.covariance(0,4) = 0;
            gaussian.covariance(0,5) = 0;

            gaussian.covariance(1,0) = 0;
            gaussian.covariance(1,1) = scale*scale;
            gaussian.covariance(1,2) = 0;
            gaussian.covariance(1,3) = 0;
            gaussian.covariance(1,4) = 0;
            gaussian.covariance(1,5) = 0;

            gaussian.covariance(2,0) = 0;
            gaussian.covariance(2,1) = 0;
            gaussian.covariance(2,2) = scale*scale;
            gaussian.covariance(2,3) = 0;
            gaussian.covariance(2,4) = 0;
            gaussian.covariance(2,5) = 0;

            gaussian.covariance(3,0) = 0;
            gaussian.covariance(3,1) = 0;
            gaussian.covariance(3,2) = 0;
            gaussian.covariance(3,3) = scale*scale;
            gaussian.covariance(3,4) = 0;
            gaussian.covariance(3,5) = 0;

            gaussian.covariance(4,0) = 0;
            gaussian.covariance(4,1) = 0;
            gaussian.covariance(4,2) = 0;
            gaussian.covariance(4,3) = 0;
            gaussian.covariance(4,4) = scale*scale;
            gaussian.covariance(4,5) = 0;

            gaussian.covariance(5,0) = 0;
            gaussian.covariance(5,1) = 0;
            gaussian.covariance(5,2) = 0;
            gaussian.covariance(5,3) = 0;
            gaussian.covariance(5,4) = 0;
            gaussian.covariance(5,5) = scale*scale;

            gaussian.weight = 1./(double)n;
            //        gaussian.weight = 1;
            gaussian.nPoint = 1;
            gaussian.cov_determinant = gaussian.covariance.determinant();
            gaussian.cov_inverse = gaussian.covariance.inverse();
            gaussian.initPrediction();
            gmm.push_back(gaussian);
        }
    }

    //    double percent;
    //    percent = 0.5;
    //    //    percent = 0.45;
    //    //    percent = 0.4;
    //    //    percent = 0.35;
    //    percent = 0.3;
    ////        percent = 0.25;
    ////        percent = 0.2;
    ////        percent = 0.15;
    ////        percent = 0.1;
    ////    percent = 0.05;
    //    //    percent = 0.02;
    ////        percent = 0.01;

    if(percent > 0){
        double last = pcl::getTime ();
        simplify(dim, SIMPLE_HCKL, 1./percent);
        //    simplify(SIMPLE_FA, 1./percent);
        double now = pcl::getTime ();
        double time = now-last;
        //        cout<<"time for simplification "<<time<<" second"<<endl;
        static double total = 0.;
        total += time;
        //        cout<<"TOTAL time for simplification "<<total<<" second"<<endl;
    }
    nowT = pcl::getTime ();
    computationT = (nowT-lastT);
    if(isDebug) cout<<"initialGMM end. computation time : "<<computationT<<endl;

}

void PCObject::simplify(int dim, SIMPLE method, double ratio, int nCluster)
{
    dimension = dim;
    double lastT, nowT, computationT;
    if(isDebug) cout<<"simplify start"<<endl;
    lastT = pcl::getTime ();
    // kmeans clustering
    int dimensions = dim;
    int sampleCount = gmm.size();
    int clusterCount;
    if(nCluster == 0)
        clusterCount = sampleCount/ratio;
    else
        clusterCount = nCluster;
    if(clusterCount<1) clusterCount = 1;

    //    clusterCount = 1;
    cv::Mat points(sampleCount, dimensions, CV_32F);
    cv::Mat labels(sampleCount, 1, CV_16S, 0);
    cv::Mat centers(clusterCount, 1, points.type());
    //    cout<<"clusterCount "<<clusterCount<<endl;

    //    if(clusterCount > 2)
    //    {
    // values of 1st half of data set is set to 10
    // change the values of 2nd half of the data set; i.e. set it to 20


    for(int i =0;i<points.rows;i++)
    {
        if(dimensions == 3){
            points.at<float>(i,0) = gmm.at(i).mean[0];
            points.at<float>(i,1) = gmm.at(i).mean[1];
            points.at<float>(i,2) = gmm.at(i).mean[2];
        }
        if(dimensions == 6){
            points.at<float>(i,0) = gmm.at(i).mean[0];
            points.at<float>(i,1) = gmm.at(i).mean[1];
            points.at<float>(i,2) = gmm.at(i).mean[2];
            points.at<float>(i,3) = gmm.at(i).mean[3];
            points.at<float>(i,4) = gmm.at(i).mean[4];
            points.at<float>(i,5) = gmm.at(i).mean[5];
        }
    }
    double last = pcl::getTime ();
    cv::kmeans(points, clusterCount, labels, cv::TermCriteria( CV_TERMCRIT_ITER, 1, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);
    double now = pcl::getTime ();

    if(method == SIMPLE_HCKL || method == SIMPLE_HCL2){
        //    % our implementation of the method proposed in <Hierarchical clustering of
        //    % a mixture model> By Jacob Goldberger Sam Roweis in NIPS 2005.

        int dim = dimensions;
        int n = gmm.size();
        vector<Gaussian> gmmHC;

        double bear = 0.00001;
        double Err0 = 1e50;

        for(int cycle = 1; cycle<=2; cycle++){
            gmmHC.clear();
            int k = 0;
            // find k
            for(int i=0;i<n;i++){
                if(labels.at<int>(i) > k)
                    k = labels.at<int>(i);
            }
            k = k + 1;

            double last = pcl::getTime ();
            // Refit
            for(int i=0;i<k;i++){
                Gaussian gmmK(dim);
                vector<Gaussian> set;
                gmmK.weight = 0.;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i){
                        set.push_back(gmm.at(j));
                        gmmK.weight += gmm.at(j).weight;
                    }
                }
                int num = set.size();
                gmmK.nPoint = num;
                Eigen::VectorXd t = Eigen::VectorXd::Zero(dim);
                for(int j = 0;j<num;j++){
                    t = t + set.at(j).mean * set.at(j).weight;
                } t = t / gmmK.weight;
                gmmK.mean = t;

                Eigen::MatrixXd cov_set = Eigen::MatrixXd::Zero(dim,dim);
                for(int j = 0;j<num;j++){
                    cov_set = cov_set + set.at(j).weight * (set.at(j).covariance + (gmmK.mean-set.at(j).mean)*(gmmK.mean-set.at(j).mean).transpose());
                }
                gmmK.covariance = cov_set / gmmK.weight;
                gmmK.cov_determinant = gmmK.covariance.determinant();
                gmmK.cov_inverse = gmmK.covariance.inverse();
                gmmK.initPrediction();
                gmmHC.push_back(gmmK);
            }
            double now = pcl::getTime ();
            //            cout<<"time for refit "<<now-last<<" second"<<endl;

            last = pcl::getTime ();
            // Regroup
            double **Dis = new double*[k];
            for(int i=0;i<k;i++)
                Dis[i] = new double[n];

            for(int i = 0;i<k;i++){
                Gaussian gmmi(dim), gmmj(dim);
                gmmi = gmmHC.at(i);
                Eigen::MatrixXd covi = gmmi.covariance;
                Eigen::MatrixXd covi_inv = gmmi.cov_inverse;
                Eigen::VectorXd meani = gmmi.mean;
                double deti = gmmi.cov_determinant;
                for (int j = 0;j<n;j++){
                    gmmj = gmm.at(j);
                    Eigen::MatrixXd covj = gmmj.covariance;
                    Eigen::VectorXd meanj = gmmj.mean;
                    double detj = gmmj.cov_determinant;

                    // KL distance
                    if(method == SIMPLE_HCKL){
                        double a = (meanj-meani).transpose()*covi_inv*(meanj-meani);
                        Dis[i][j] = 1./2.*(log(deti/detj) + (covi_inv*covj).trace() + a-dim);
                    }
                    // L2 distance
                    if(method == SIMPLE_HCL2){
                        double energy1, energy2, energy3;
                        Eigen::MatrixXd invij = (covi+covj).inverse();
                        double a = (meani-meanj).transpose()*invij*(meani-meanj);
                        energy1 = 1./sqrt(pow(2*pi,dim)*(covi+covi).determinant());
                        energy2 = 1./sqrt(pow(2*pi,dim)*(covi+covj).determinant())*exp(-0.5*a);
                        energy3 = 1./sqrt(pow(2*pi,dim)*(covj+covj).determinant());
                        Dis[i][j] = gmmi.weight*gmmj.weight*(energy1-2*energy2+energy3);
                    }
                }
            }
            now = pcl::getTime ();
            //            cout<<"time for regroup "<<now-last<<" second"<<endl;

            double Err1 = 0;
            for(int i=0;i<n;i++){
                // find min
                double min;
                int minK;
                for(int j=0;j<k;j++){
                    if(j == 0) {
                        min = Dis[j][i];
                        minK = j;
                    }
                    else{
                        if(Dis[j][i] < min){
                            min = Dis[j][i];
                            minK = j;
                        }
                    }
                }
                Err1 = Err1 + fabs(gmm.at(i).weight) * min;
                labels.at<int>(i) = minK;
            }
            for(int i=0;i<k;i++)
                delete[] Dis[i];
            delete[] Dis;

            // reordering labels
            int K = k;
            int *siz = new int[K];
            for (int i = 0;i<K;i++){
                siz[i] = 0;
                for(int j=0;j<n;j++){
                    if(labels.at<int>(j) == i)
                        siz[i] ++;
                }
            }
            int *map = new int[K];
            int c = -1;
            for (int i = 0;i<K;i++){
                if(siz[i]>=1){
                    c = c + 1;
                }
                map[i] = c;
            }
            k = c;
            for (int i = 0;i<n;i++){
                labels.at<int>(i) = map[labels.at<int>(i)];
            }
            //            cout<<"cycle"<<cycle<<endl;
            if(fabs(Err1 - Err0)<= bear * fabs(Err0))
                break;
            else
                Err0 = Err1;
            delete[] siz;
            delete[] map;

        }
        gmm = gmmHC;
    }
    nowT = pcl::getTime ();
    computationT = (nowT-lastT);
    if(isDebug) cout<<"simplify end. computation time : "<<computationT<<endl;
}


void PCObject::filteringGMM_EM(PCObject* prior)
{
    state = POSTGMM;

    // EM algorithm for update posterior GMM with prior.gmm and points
    int N_i = points.size();    // # of points
    int N_k;    // # of components

    vector<Gaussian> gmm_prev = prior->gmm;
    double exploglikelihood = 0.;
    double exploglikelihood_old;
    for(int n=0;n<1;n++){
        gmm.clear();
        N_k = gmm_prev.size();

        double** r = new double*[N_i];
        double** l = new double*[N_i];
        for(int i=0;i<N_i;i++)  {
            r[i] = new double[N_k];
            l[i] = new double[N_k];
        }
        double* rk = new double[N_k];

        // E-step
        // responsibility of each point to the gaussian
        exploglikelihood_old = exploglikelihood;
        exploglikelihood = 0.;
        for(int i=0;i<N_i;i++){
            double normSum = 0.;
            for(int k=0;k<N_k;k++){
                double pi_k = gmm_prev.at(k).weight;
                l[i][k] = likelihood(points.at(i), gmm_prev.at(k));   // needed to be updated for 6-dim and surface normal feature
                r[i][k] = pi_k * l[i][k];
                normSum += r[i][k];
            }
            double exp1 = 0.;
            double exp2 = 0.;
            for(int k=0;k<N_k;k++){
                r[i][k] = r[i][k] / normSum;
                exp1 += r[i][k]*log(gmm_prev.at(k).weight);
                exp2 += r[i][k]*log(l[i][k]);
//                cout<<"log "<<exp1<<" "<<exp2<<endl;
            }
            exploglikelihood = exp1 + exp2;
        }
//        cout<<"expected likelihood: "<<exploglikelihood<<endl;
//        cout<<"expected likelihood increment: "<<exploglikelihood-exploglikelihood_old<<endl;
        if(fabs(exploglikelihood-exploglikelihood_old) < 1)
            break;
        // M-step : MAP tlinla~
        float weightSum = 0;
        for(int k=0;k<N_k;k++){
            // weight
            rk[k] = 0.;
            for(int i=0;i<N_i;i++){
                rk[k] += r[i][k];
            }
            double weight = rk[k] / (double)N_i;

            // initialization
            Eigen::VectorXd mean_prv = gmm_prev.at(k).mean;
            Eigen::VectorXd mean;
            Eigen::MatrixXd covariance;
                   if(dimension == 3){
                mean = Eigen::VectorXd(3);
                covariance = Eigen::MatrixXd(3,3);
                mean << 0.,0.,0.;
                covariance << 0.,0.,0.,
                        0.,0.,0.,
                        0.,0.,0.;
            }
            if(dimension == 6){
                mean = Eigen::VectorXd(6);
                covariance = Eigen::MatrixXd(6,6);
                mean << 0.,0.,0.,0.,0.,0.;
                covariance << 0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.;
            }

            // mean
            for(int i=0;i<N_i;i++){
                mean += r[i][k] * points.at(i).pos;
            }
            mean = mean / rk[k];
            // cov
            for(int i=0;i<N_i;i++){
                covariance = covariance + (r[i][k] * points.at(i).pos * points.at(i).pos.transpose());
            }
            covariance = (covariance / rk[k]) - (mean * mean.transpose());
//            cout<<mean<<endl;
//            cout<<covariance<<endl;

            Gaussian newGaussian;
            newGaussian.dim = dimension;
            newGaussian.weight = weight;
            newGaussian.mean = mean;
            newGaussian.covariance = covariance;
            gmm.push_back(newGaussian);
//            cout<<weight<<endl;
            weightSum += weight;
        }
        gmm_prev = gmm;
//        cout<<endl<<"weightSum: "<<weightSum<<endl;

        for(int i=0;i<N_i;i++)  {
            delete[] r[i];
            delete[] l[i];
        }
        delete[] l;
        delete[] r;
        delete[] rk;
    }
}


void PCObject::filteringGMM_incrementalEM(PCObject* prior, double percent)
{
    state = POSTGMM;
    int N_ik = 1./percent;  // # of points in a cluster

    // EM algorithm for update posterior GMM with prior.gmm and points
    int N_i = points.size();    // # of points
    int N_k;    // # of components

    vector<Gaussian> gmm_prev = prior->gmm;
    double exploglikelihood = 0.;
    double exploglikelihood_old;
    for(int n=0;n<400;n++){
        gmm.clear();
        N_k = gmm_prev.size();

        double** r = new double*[N_i];
        double** l = new double*[N_i];
        for(int i=0;i<N_i;i++)  {
            r[i] = new double[N_k];
            l[i] = new double[N_k];
        }
        double* rk = new double[N_k];

        // E-step
        // responsibility of each point to the gaussian
        exploglikelihood_old = exploglikelihood;
        exploglikelihood = 0.;
        for(int i=0;i<N_i;i++){
            double normSum = 0.;
            for(int k=0;k<N_k;k++){
                double pi_k = gmm_prev.at(k).weight;
                l[i][k] = likelihood(points.at(i), gmm_prev.at(k));   // needed to be updated for 6-dim and surface normal feature
                r[i][k] = pi_k * l[i][k];
                normSum += r[i][k];
            }
            double exp1 = 0.;
            double exp2 = 0.;
            for(int k=0;k<N_k;k++){
                r[i][k] = r[i][k] / normSum;
                exp1 += r[i][k]*log(gmm_prev.at(k).weight);
                exp2 += r[i][k]*log(l[i][k]);
                cout<<"log "<<exp1<<" "<<exp2<<endl;
            }
            exploglikelihood = exp1 + exp2;
        }
        cout<<"expected likelihood: "<<exploglikelihood<<endl;
        cout<<"expected likelihood increment: "<<exploglikelihood-exploglikelihood_old<<endl;
        if(fabs(exploglikelihood-exploglikelihood_old) < 1)
            break;
        // M-step : MAP tlinla~
        float weightSum = 0;
        for(int k=0;k<N_k;k++){
            // weight
            rk[k] = 0.;
            for(int i=0;i<N_i;i++){
                rk[k] += r[i][k];
            }
            double weight = rk[k] / (double)N_i;

            // initialization
            Eigen::VectorXd mean_prv = gmm_prev.at(k).mean;
            Eigen::VectorXd mean;
            Eigen::MatrixXd covariance;
                   if(dimension == 3){
                mean = Eigen::VectorXd(3);
                covariance = Eigen::MatrixXd(3,3);
                mean << 0.,0.,0.;
                covariance << 0.,0.,0.,
                        0.,0.,0.,
                        0.,0.,0.;
            }
            if(dimension == 6){
                mean = Eigen::VectorXd(6);
                covariance = Eigen::MatrixXd(6,6);
                mean << 0.,0.,0.,0.,0.,0.;
                covariance << 0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.,
                        0.,0.,0.,0.,0.,0.;
            }

            // mean
            for(int i=0;i<N_i;i++){
                mean += r[i][k] * points.at(i).pos;
            }
            mean = mean / rk[k];
            // cov
            for(int i=0;i<N_i;i++){
                covariance = covariance + (r[i][k] * points.at(i).pos * points.at(i).pos.transpose());
            }
            covariance = (covariance / rk[k]) - (mean * mean.transpose());
//            cout<<mean<<endl;
//            cout<<covariance<<endl;

            Gaussian newGaussian;
            newGaussian.dim = dimension;
            newGaussian.weight = weight;
            newGaussian.mean = mean;
            newGaussian.covariance = covariance;
            gmm.push_back(newGaussian);
//            cout<<weight<<endl;
            weightSum += weight;
        }
        gmm_prev = gmm;
//        cout<<endl<<"weightSum: "<<weightSum<<endl;

        for(int i=0;i<N_i;i++)  {
            delete[] r[i];
            delete[] l[i];
        }
        delete[] l;
        delete[] r;
        delete[] rk;
    }
}

double PCObject::likelihood(Point point, Gaussian gaussian)
{
    Eigen::VectorXd mean;
    mean = gaussian.mean;
    Eigen::MatrixXd inv = gaussian.covariance.inverse();

    double a = (point.pos-mean).transpose()*inv*(point.pos-mean);
    double det = gaussian.covariance.determinant();
    double eval = 1./sqrt(pow(2*pi,dimension)*det) * exp(-0.5*a);

//    if(eval>1000000)
//        return 1000000;
//        cout<<"big eval "<<eval<<endl;
//    else
    return eval;

}

double PCObject::likelihood_standard(Point point, Gaussian gaussian)
{
    Eigen::VectorXd mean;
    mean = gaussian.mean;
    Eigen::MatrixXd inv = gaussian.covariance.inverse();

    double norm = (point.pos-mean).transpose()*inv*(point.pos-mean);

    return norm;

}

/*
double PCObject::L2ofGMMandPoints(double scale)
{
    // L2 distance

    int n = points.size();
    int m = gmm.size();
    vector<Gaussian> gmmPoint;
    for(int i=0;i<n;i++){
        Gaussian point;
        point.mean = points.at(i).pos;
        point.covariance(0,0) = scale*scale;
        point.covariance(0,1) = 0;
        point.covariance(0,2) = 0;
        point.covariance(1,0) = 0;
        point.covariance(1,1) = scale*scale;
        point.covariance(1,2) = 0;
        point.covariance(2,0) = 0;
        point.covariance(2,1) = 0;
        point.covariance(2,2) = scale*scale;
        point.weight = 1./(double)n;

        gmmPoint.push_back(point);
    }
    double energy1 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            Eigen::Matrix3d cov = gmmPoint.at(i).covariance + gmmPoint.at(j).covariance;
            Eigen::Vector3d mean = gmmPoint.at(i).mean - gmmPoint.at(j).mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy1 += gmmPoint.at(i).weight*gmmPoint.at(j).weight*gauss;
        }
    }
    double energy2 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<m;j++){
            Eigen::Matrix3d cov = gmmPoint.at(i).covariance + gmm.at(j).covariance;
            Eigen::Vector3d mean = gmmPoint.at(i).mean - gmm.at(j).mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy2 += gmmPoint.at(i).weight*gmm.at(j).weight*gauss;
        }
    }
    double energy3 = 0.;
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            Eigen::Matrix3d cov = gmm.at(i).covariance + gmm.at(j).covariance;
            Eigen::Vector3d mean = gmm.at(i).mean - gmm.at(j).mean;
            Eigen::Matrix3d invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,3)*cov.determinant())*exp(-0.5*a);
            energy3 += gmm.at(i).weight*gmm.at(j).weight*gauss;
        }
    }
    return energy1 - 2*energy2 + energy3;
}
*/
double PCObject::evalGMM(Point x)
{
    double eval = 0.;
    int dim = x.dim;
    Eigen::VectorXd pos(dim);
    if(dim == 3){
        pos[0] = x.pos[0];
        pos[1] = x.pos[1];
        pos[2] = x.pos[2];
    }
    if(dim == 6){
        pos[0] = x.pos[0];
        pos[1] = x.pos[1];
        pos[2] = x.pos[2];
        pos[3] = x.rgb[0];
        pos[4] = x.rgb[1];
        pos[5] = x.rgb[2];
    }
    for(int i=0;i<gmm.size();i++){
        Eigen::VectorXd mean;
        mean = gmm.at(i).mean;
        double weight = gmm.at(i).weight;
        Eigen::MatrixXd inv = gmm.at(i).cov_inverse;



        double a = (pos-mean).transpose()*inv*(pos-mean);
        double det = gmm.at(i).covariance.determinant();
        eval += weight * 1./sqrt(pow(2*pi,dim)*det) * exp(-0.5*a);
        //        cout<<"pos "<<pos<<endl;
        //        cout<<"mean "<<mean<<endl;

        //        eval += weight * exp(-0.5*a);
    }

    //    return eval * gmm.size();

    return eval;
}


double PCObject::evalNormedGMM(Point x, double den)
{
    double eval = 0.;
    int dim = x.dim;
    for(int i=0;i<gmm.size();i++){
        Eigen::VectorXd mean;
        mean = gmm.at(i).mean;
        Eigen::MatrixXd cov = gmm.at(i).covariance;
        double weight = gmm.at(i).weight;
        Eigen::MatrixXd inv = gmm.at(i).cov_inverse;
        Eigen::VectorXd pos(dim);
        if(dim == 3){
            pos[0] = x.pos[0];
            pos[1] = x.pos[1];
            pos[2] = x.pos[2];
        }
        if(dim == 6){
            pos[0] = x.pos[0];
            pos[1] = x.pos[1];
            pos[2] = x.pos[2];
            pos[3] = x.rgb[0];
            pos[4] = x.rgb[1];
            pos[5] = x.rgb[2];
        }

        double a = (pos-mean).transpose()*inv*(pos-mean);
        double det = gmm.at(i).covariance.determinant();
        eval += weight * 1./sqrt(pow(2*pi,dim)*det) * exp(-0.5*a);
        //        eval += weight * exp(-0.5*a);
    }
    return eval * gmm.size() / den;
}

double PCObject::evalClosestNormedGMM(Point x, double den)
{
    int dim = x.dim;
    double max = 0.;
    Eigen::VectorXd pos(dim), mean(dim);
    if(dim == 3){
        pos[0] = x.pos[0];
        pos[1] = x.pos[1];
        pos[2] = x.pos[2];
    }
    if(dim == 6){
        pos[0] = x.pos[0];
        pos[1] = x.pos[1];
        pos[2] = x.pos[2];
        pos[3] = x.rgb[0];
        pos[4] = x.rgb[1];
        pos[5] = x.rgb[2];
    }
    // finding the closest gaussian of x
    double minDist;
    int minGaussian;
    for(int i=0;i<gmm.size();i++){
        mean = gmm.at(i).mean;
        double dist = (pos-mean).transpose()*(pos-mean);
        if(i == 0){
            minDist = dist;
            minGaussian = i;
        }
        else{
            if(dist < minDist){
                minDist = dist;
                minGaussian = i;
            }
        }
    }
    mean = gmm.at(minGaussian).mean;
    double weight = gmm.at(minGaussian).weight;
    //    double weight = 1.;
    Eigen::MatrixXd inv = gmm.at(minGaussian).cov_inverse;

    double a = (pos-mean).transpose()*inv*(pos-mean);
    double det = gmm.at(minGaussian).covariance.determinant();
    double eval = weight * 1./sqrt(pow(2*pi,dim)*det) * exp(-0.5*a);

    return eval * gmm.size() / den;
}

double PCObject::evalClosestGMM(Point x)
{
    int dim = x.dim;
    double max = 0.;
    Eigen::VectorXd pos(dim), mean(dim);
    if(dim == 3){
        pos[0] = x.pos[0];
        pos[1] = x.pos[1];
        pos[2] = x.pos[2];
    }
    if(dim == 6){
        pos[0] = x.pos[0];
        pos[1] = x.pos[1];
        pos[2] = x.pos[2];
        pos[3] = x.rgb[0];
        pos[4] = x.rgb[1];
        pos[5] = x.rgb[2];
    }
    // finding the closest gaussian of x
    double minDist;
    int minGaussian;
    for(int i=0;i<gmm.size();i++){
        mean = gmm.at(i).mean;
        double dist = (pos-mean).transpose()*(pos-mean);
        if(i == 0){
            minDist = dist;
            minGaussian = i;
        }
        else{
            if(dist < minDist){
                minDist = dist;
                minGaussian = i;
            }
        }
    }
    mean = gmm.at(minGaussian).mean;
    double weight = gmm.at(minGaussian).weight;
    //    double weight = 1.;
    Eigen::MatrixXd inv = gmm.at(minGaussian).cov_inverse;

    double a = (pos-mean).transpose()*inv*(pos-mean);
    double det = gmm.at(minGaussian).covariance.determinant();
    double eval = weight * 1./sqrt(pow(2*pi,dim)*det) * exp(-0.5*a);

    return eval;
}

void PCObject::mergeTwoGMMs(PCObject* gmm1, PCObject* gmm2)
{
    gmm.clear();
    for(int i=0;i<gmm1->gmm.size();i++){
        Gaussian gaussian = gmm1->gmm.at(i);
        gaussian.weight /= 2.;
        gmm.push_back(gaussian);
    }
    for(int i=0;i<gmm2->gmm.size();i++){
        Gaussian gaussian = gmm2->gmm.at(i);
        gaussian.weight /= 2.;
        gmm.push_back(gaussian);
    }
}

void PCObject::gaussianTrackingInit(int _window_short, int _window_long, int _maxID, funcWeightGaussian _weight_gaussian, funcWeightGaussian _weight_gaussian_fast)
{
    window_short = _window_short;
    window_long = _window_long;

    imftGaussians = new IMFT<Gaussian>(window_short, window_long, _maxID, _weight_gaussian, _weight_gaussian_fast);
    gaussianTracks = new TrackContainer<Gaussian>(1000);
    topology_graph = new ListGraph();
    topology_nodeMap = new ListGraph::NodeMap<Gaussian>(*topology_graph);
    topology_edgeMap = new ListGraph::EdgeMap<double>(*topology_graph);

    cnt = 0;

    frame.clear();
    for(int i=0;i<gmm.size();i++){
        frame.push_back(&gmm.at(i));
    }
    imftGaussians->setFrame(frame, cnt);
    imftGaussians->matching();
    //    imftGaussians->confirmDGraph();
    imftGaussians->updateTracks();

    gaussianTracks = (TrackContainer<Gaussian>*)imftGaussians->extractTracks();

    cnt++;
}

void PCObject::gaussianTracking()
{
    frame.clear();
    for(int i=0;i<gmm.size();i++){
        frame.push_back(&gmm.at(i));
    }
    imftGaussians->setFrame(frame, cnt);
    imftGaussians->matching();
    //    imftGaussians->confirmDGraph();
    imftGaussians->updateTracks();

    gaussianTracks = (TrackContainer<Gaussian>*)imftGaussians->extractTracks();

    // update gmm
    gmm.clear();
    for(int i=0;i<gaussianTracks->numTracks();i++){
        gmm.push_back(gaussianTracks->tracks.at(i)->lastFrame().object);
    }


    cnt++;
}

void PCObject::updatePredictiveParameters()
{
    // update predictive parameters in all gaussians in the graph
    //    imftGaussians->confirmDGraph();
    for(ListGraph::NodeIt n(imftGaussians->m_g); n != INVALID; ++n){
        Gaussian *node = &((*imftGaussians->m_gNodeMap)[n].ptr.object);
        node->updateParam(trans_param);
    }
}

void PCObject::makeTopology()
{
    //    if(topology_nodeMap != NULL) delete topology_nodeMap;
    //    if(topology_edgeMap != NULL) delete topology_edgeMap;
    //    if(topology_graph != NULL) delete topology_graph;
    topology_graph->clear();
    edges.clear();

    int dim;
    // make node
    for(int i=0;i<gmm.size();i++){
        ListGraph::Node node = topology_graph->addNode();
        (*topology_nodeMap)[node] = gmm.at(i);
        dim = gmm.at(i).dim;
    }
    // make edge
    double max_pos = 0;
    double max_vel = 0;

    vector<Edge_spatial> edge_init;
    for(ListGraph::NodeIt u(*topology_graph); u != INVALID; ++u){
        ListGraph::NodeIt v = u;
        for(v; v != INVALID; ++v){
            double weight_pos = topology_posweight_rev((*topology_nodeMap)[u], (*topology_nodeMap)[v]);
            double weight_vel = topology_velweight_rev((*topology_nodeMap)[u], (*topology_nodeMap)[v]);

            Edge_spatial edge;
            edge.u = u;
            edge.v = v;
            edge.weight_pos = weight_pos;
            edge.weight_vel = weight_vel;
            edge_init.push_back(edge);

            if(weight_pos > max_pos) max_pos = weight_pos;
            if(weight_vel > max_vel) max_vel = weight_vel;
        }
    }

    // make edge
    for(int i= 0;i<edge_init.size();i++){
        double weight_pos = 1 - edge_init.at(i).weight_pos / max_pos;
        double weight_vel = 1 - edge_init.at(i).weight_vel / max_vel;
        double weight = weight_pos * alpha + weight_vel * (1-alpha);

        if(edge_init.size() <= 15 || weight > th_edge){ // store the edge
            ListGraph::Edge edge = topology_graph->addEdge(edge_init.at(i).u,edge_init.at(i).v);
            (*topology_edgeMap)[edge] = weight;

            Edge e;
            if(dim == 3){
                e.u.pos[0] = (*topology_nodeMap)[edge_init.at(i).u].mean[0];
                e.u.pos[1] = (*topology_nodeMap)[edge_init.at(i).u].mean[1];
                e.u.pos[2] = (*topology_nodeMap)[edge_init.at(i).u].mean[2];

                e.v.pos[0] = (*topology_nodeMap)[edge_init.at(i).v].mean[0];
                e.v.pos[1] = (*topology_nodeMap)[edge_init.at(i).v].mean[1];
                e.v.pos[2] = (*topology_nodeMap)[edge_init.at(i).v].mean[2];

            }
            if(dim == 6){
                e.u.pos[0] = (*topology_nodeMap)[edge_init.at(i).u].mean[0];
                e.u.pos[1] = (*topology_nodeMap)[edge_init.at(i).u].mean[1];
                e.u.pos[2] = (*topology_nodeMap)[edge_init.at(i).u].mean[2];
                e.u.rgb[0] = (*topology_nodeMap)[edge_init.at(i).u].mean[3];
                e.u.rgb[1] = (*topology_nodeMap)[edge_init.at(i).u].mean[4];
                e.u.rgb[2] = (*topology_nodeMap)[edge_init.at(i).u].mean[5];

                e.v.pos[0] = (*topology_nodeMap)[edge_init.at(i).v].mean[0];
                e.v.pos[1] = (*topology_nodeMap)[edge_init.at(i).v].mean[1];
                e.v.pos[2] = (*topology_nodeMap)[edge_init.at(i).v].mean[2];
                e.v.rgb[0] = (*topology_nodeMap)[edge_init.at(i).v].mean[3];
                e.v.rgb[1] = (*topology_nodeMap)[edge_init.at(i).v].mean[4];
                e.v.rgb[2] = (*topology_nodeMap)[edge_init.at(i).v].mean[5];

            }
            e.weight = weight;
            edges.push_back(e);
        }
    }
}

double PCObject::topology_weight(Gaussian g1, Gaussian g2)
{
    int dim = g1.dim;

    //    double maxDist = 23000;   // for 0.02 scale / 0.2 percent : lower -> weaker
    double maxDist = 55000000;   // for 0.02 scale / 0.1 percent : lower -> weaker
    //    double maxDist = 150000;   // for 0.01 scale
    double maxVel = 0.02;
    double energy1 = 0.;
    Eigen::MatrixXd cov = g1.covariance + g1.covariance;
    Eigen::VectorXd mean = g1.mean - g1.mean;
    Eigen::MatrixXd invij = cov.inverse();
    double a = mean.transpose()*invij*mean;
    double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
    energy1 += gauss;

    double energy2 = 0.;

    cov = g1.covariance + g2.covariance;
    mean = g1.mean - g2.mean;
    invij = cov.inverse();
    a = mean.transpose()*invij*mean;
    gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
    energy2 += gauss;

    double energy3 = 0.;
    cov = g2.covariance + g2.covariance;
    mean = g2.mean - g2.mean;
    invij = cov.inverse();
    a = mean.transpose()*invij*mean;
    gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
    energy3 += gauss;
    double posCloseness = (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
    Eigen::VectorXd velDiff = g1.velocity - g2.velocity;
    double velCloseness =  (maxVel - velDiff.norm())/maxVel;
    //    cout<<velDiff.norm()<<endl;
    //        return 0.98 * posCloseness + 0.02 * velCloseness;
    return 1. * posCloseness + 0. * velCloseness;
    //    return 1;
}


double PCObject::topology_posweight_rev(Gaussian g1, Gaussian g2)
{
    // KL divergence between g1 and g2
    int dim = g1.dim;

    double kl12;
    double kl21;

    Eigen::MatrixXd cov = g1.cov_inverse * g2.covariance;
    Eigen::VectorXd mean = g2.mean - g1.mean;
    kl12 = (cov.trace() + mean.transpose()*g2.cov_inverse*mean - log(g1.covariance.norm() / g2.covariance.norm()) - dim)/2.;

    cov = g2.cov_inverse * g1.covariance;
    mean = g1.mean - g2.mean;
    kl21 = (cov.trace() + mean.transpose()*g1.cov_inverse*mean - log(g2.covariance.norm() / g1.covariance.norm()) - dim)/2.;

    return kl12 + kl21;
}


double PCObject::topology_velweight_rev(Gaussian g1, Gaussian g2)
{
    Eigen::VectorXd velDiff = g1.velocity - g2.velocity;
    return velDiff.norm();
}

void PCObject::calculateVelocity()
{
    for(int i=0;i<gaussianTracks->numTracks();i++){
        Gaussian last = gaussianTracks->tracks.at(i)->lastFrame().object;
        Gaussian before = gaussianTracks->tracks.at(i)->getFrameFromLast(1).object;
        gmm.at(i).velocity[0] = last.mean[0] - before.mean[0];
        gmm.at(i).velocity[1] = last.mean[1] - before.mean[1];
        gmm.at(i).velocity[2] = last.mean[2] - before.mean[2];
    }
}

int PCObject::componentGraph(vector<PCObject> &newObjects)
{
    bool connectivity = connected(*topology_graph);
    // check the number of components
    if(connectivity)
        return 0;
    else{
        ListGraph::NodeMap<int> component_nodeMap(*topology_graph);
        //        component_nodeMap = new ListGraph::NodeMap<Gaussian>(*topology_graph);
        int num = connectedComponents(*topology_graph, component_nodeMap);
        //        newObjects.resize(num);
        for(int i=0;i<num;i++){
            PCObject object;
            newObjects.push_back(object);
        }

        for(ListGraph::NodeIt u(*topology_graph); u != INVALID; ++u){
            int id = (component_nodeMap)[u];
            newObjects.at(id).gmm.push_back((*topology_nodeMap)[u]);
        }
        /*
        // check velocity difference
        vector<double> velNorm;
        for(int i=0;i<num;i++){
            int size = newObjects.at(i).gmm.size();
            Eigen::Vector3d velAvg;
            for(int j=0;j<size;j++){
                Eigen::Vector3d vel = velAvg = velAvg + newObjects.at(i).gmm.at(j).velocity;
                velAvg += vel;
            }
            velAvg = velAvg/size;
            velNorm.push_back(velAvg.norm());
//            std::cout<<"vel diff : "<<velNorm[i]<<std::endl;
        }
        double sumVelNorm = 0.;
        for(int i=0;i<velNorm.size();i++){
            sumVelNorm += velNorm.at(i);
        }
        // entropy
        double entropy = 0.;
        for(int i=0;i<velNorm.size();i++){
            entropy += (velNorm.at(i)/sumVelNorm) * log(velNorm.at(i)/sumVelNorm);
        }
        entropy = -entropy;
//        cout<<"ENTROPY "<<entropy<<endl;
        //            if(fabs(velNorm.at(i)-velNorm.at(i-1)) < 50000)
        //                return 0;
        //        }
        */

        // point matching
        double sumSizeGMMs = 0.;
        for(int i=0;i<num;i++){
            sumSizeGMMs += newObjects.at(i).gmm.size();
        }
        for(int i=0;i<points.size();i++){
            Point point = points.at(i);
            int maxTrack = 0;
            double maxProb = 0.;
            for(int j=0;j<num;j++){
                double prob = newObjects.at(j).evalNormedGMM(point, sumSizeGMMs);
                //            double prob = object->evalGMM(point);
                //            double prob = object->evalClosestGMM(point);
                if(prob > maxProb){
                    maxProb = prob;
                    maxTrack = j;
                }
            }
            newObjects.at(maxTrack).points.push_back(point);
        }
        return num;
    }
}
