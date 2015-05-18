#include "pctracking_weight.h"
//double maxDist = 10000000;
/*
GAUSS1 gauss1(PCObject &o)
{
    double covMin = 0.01;
    GAUSS1 gauss;
    int size = o.points.size();

    Eigen::Vector3d mean;
    for(int j=0;j<size;j++){
        mean += o.points.at(j).pos;
    }
    gauss.mean = mean/(double)size;

    // covariance
    Eigen::Vector3d demean;
    Eigen::Vector3d xyz_centroid = gauss.mean;
    Eigen::Vector3d point;
    gauss.covariance.setZero();

    double demean_xy, demean_xz, demean_yz;
    // For each point in the cloud
    for (int idx = 0; idx < size; ++idx)
    {

        point = o.points.at(idx).pos;
        demean = point - xyz_centroid;

        demean_xy = demean[0] * demean[1];
        demean_xz = demean[0] * demean[2];
        demean_yz = demean[1] * demean[2];

        gauss.covariance(0, 0) += demean[0] * demean[0];
        gauss.covariance(0, 1) += demean_xy;
        gauss.covariance(0, 2) += demean_xz;

        gauss.covariance(1, 0) += demean_xy;
        gauss.covariance(1, 1) += demean[1] * demean[1];
        gauss.covariance(1, 2) += demean_yz;

        gauss.covariance(2, 0) += demean_xz;
        gauss.covariance(2, 1) += demean_yz;
        gauss.covariance(2, 2) += demean[2] * demean[2];
    }
    gauss.covariance = gauss.covariance / (double)size;


    //        std::cout << statistic.covariance << std::endl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(gauss.covariance);
    gauss.eigenvalues = eigensolver.eigenvalues();
    gauss.eigenvectors = eigensolver.eigenvectors();    // find an eigenvactor by calling .col(i)

    return gauss;
}

double weight_cov(PCObject &o1, PCObject &o2)
{
    if(o1.points.size() <= 1 || o2.points.size() <= 1)
        return 0.0;

    double maxX = 0.5;
    double maxY = 0.5;
    double maxZ = 0.5;
    double maxCovDist = 10;
    double alpha = 0.8; // weighting for mean

    // covariance matrix
    GAUSS1 o1_gauss = gauss1(o1);
    GAUSS1 o2_gauss = gauss1(o2);

    Eigen::Matrix3d cov1 = o1_gauss.covariance;
    Eigen::Matrix3d cov2 = o2_gauss.covariance;

    //        Eigen::Matrix3f A;
    // sqrt(inv(A))*B*sqrt(inv(A))
    Eigen::Matrix3d sqrtM, C;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver1(cov1.inverse());
    sqrtM = eigensolver1.operatorSqrt();
    C = sqrtM * cov2 * sqrtM;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver2(C);
    Eigen::Vector3d eigenvalues = eigensolver2.eigenvalues();
    double sum = 0;
    for(int j=0;j<3;j++){
        //            std::cout << log(eigenvalues[j]) << std::endl;
        sum += pow(log(eigenvalues[j]),2);
    }
    double covDist = 1 - sqrt(sum) / maxCovDist;
    double meanDist = 1 - sqrt(pow(o1_gauss.mean[0]-o2_gauss.mean[0],2) + pow(o1_gauss.mean[1]-o2_gauss.mean[1],2) + pow(o1_gauss.mean[2]-o2_gauss.mean[2],2))
            / sqrt(maxX*maxX + maxY*maxY + maxZ*maxZ);

    //    double alpha = 0.5;
    double weight = alpha * meanDist + (1-alpha) * covDist;

    return weight;
}

double weight_l2_gmm1(PCObject &o1, PCObject &o2)
{

    if(o1.points.size() <= 1 || o2.points.size() <= 1)
        return 0.0;

    double maxDist = 13.;
    // covariance matrix
    GAUSS1 o1_gauss = gauss1(o1);
    GAUSS1 o2_gauss = gauss1(o2);

    Eigen::Matrix3d cov1 = o1_gauss.covariance;
    Eigen::Matrix3d cov2 = o2_gauss.covariance;
    Eigen::Vector3d mean1 = o1_gauss.mean;
    Eigen::Vector3d mean2 = o2_gauss.mean;
    Eigen::Matrix3d cov;
    Eigen::Vector3d mean;
    double det;
    Eigen::Matrix3d inv;
    double a;

    cov = cov1+cov1;
    mean = mean1-mean1;
    det = cov.determinant();
    if(det<0.01) det = 0.0001;
    inv = cov.inverse();
    a = mean.transpose()*inv*mean;
    double energy1 = 1/sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    printf("energy1 %f\n", energy1);

    // f(x)g(x)
    cov = cov1+cov2;
    mean = mean1-mean2;
    det = cov.determinant();
    if(det<0.01) det = 0.0001;
    inv = cov.inverse();
    a = mean.transpose()*inv*mean;
    double energy2 = 1/sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    printf("energy2 %f\n", energy2);

    // g(x)^2
    cov = cov2+cov2;
    mean = mean2-mean2;
    det = cov.determinant();
    if(det<0.01) det = 0.0001;
    inv = cov.inverse();
    a = mean.transpose()*inv*mean;
    double energy3 = 1/sqrt(pow(2*pi,3)*det) * exp(-0.5*a);
    printf("energy3 %f\n", energy3);
    printf("weight %f\n", (energy1 - 2*energy2 + energy3));

    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
    //    return (energy1 - 2*energy2 + energy3);
}
*/
/*
double weight_l2(PCObject &o1, PCObject &o2)
{
    double last = pcl::getTime ();
    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010
    int n = o1.gmm.size();
    int m = o2.gmm.size();

    double energy1 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            Eigen::MatrixXd cov = o1.gmm.at(i).covariance + o1.gmm.at(j).covariance;
            Eigen::VectorXd mean = o1.gmm.at(i).mean - o1.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy1 += o1.gmm.at(i).weight*o1.gmm.at(j).weight*gauss;
        }
    }
    double energy2 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<m;j++){
            Eigen::MatrixXd cov = o1.gmm.at(i).covariance + o2.gmm.at(j).covariance;
            Eigen::VectorXd mean = o1.gmm.at(i).mean - o2.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy2 += o1.gmm.at(i).weight*o2.gmm.at(j).weight*gauss;
        }
    }
    double energy3 = 0.;
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            Eigen::MatrixXd cov = o2.gmm.at(i).covariance + o2.gmm.at(j).covariance;
            Eigen::VectorXd mean = o2.gmm.at(i).mean - o2.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy3 += o2.gmm.at(i).weight*o2.gmm.at(j).weight*gauss;
        }
    }
    double now = pcl::getTime ();
//    cout << "l2-distance time " << now-last << " second" << endl;
    //    cout<<"l2distance"<<energy1 - 2*energy2 + energy3<<endl;
    double maxDist = 10000000;
    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
}
*/

double weight_l2_rev(PCObject &o1, PCObject &o2)
{
    double last = pcl::getTime ();
    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010
    int n = o1.gmm.size();
    int m = o2.gmm.size();

    double energy1 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            int dim = o1.gmm.at(i).dim;
            Eigen::MatrixXd cov = o1.gmm.at(i).covariance + o1.gmm.at(j).covariance;
            Eigen::VectorXd mean = o1.gmm.at(i).mean - o1.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy1 += o1.gmm.at(i).weight*o1.gmm.at(j).weight*gauss;
        }
    }
    double energy2 = 0.;
    for(int i=0;i<n;i++){
        for(int j=0;j<m;j++){
            int dim = o1.gmm.at(i).dim;
            Eigen::MatrixXd cov = o1.gmm.at(i).covariance + o2.gmm.at(j).covariance;
            Eigen::VectorXd mean = o1.gmm.at(i).mean - o2.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy2 += o1.gmm.at(i).weight*o2.gmm.at(j).weight*gauss;
        }
    }
    double energy3 = 0.;
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            int dim = o2.gmm.at(i).dim;
            Eigen::MatrixXd cov = o2.gmm.at(i).covariance + o2.gmm.at(j).covariance;
            Eigen::VectorXd mean = o2.gmm.at(i).mean - o2.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy3 += o2.gmm.at(i).weight*o2.gmm.at(j).weight*gauss;
        }
    }
    double now = pcl::getTime ();
//    cout << "l2-distance time " << now-last << " second" << endl;
    //    cout<<"l2distance"<<energy1 - 2*energy2 + energy3<<endl;

    return (energy1 - 2*energy2 + energy3);
}

double weight_loc(PCObject &o1, PCObject &o2)
{
    double dist = sqrt(pow(o1.centroid.pos[0]-o2.centroid.pos[0],2) + pow(o1.centroid.pos[1]-o2.centroid.pos[1],2) + pow(o1.centroid.pos[2]-o2.centroid.pos[2],2));
    return dist;
}


double weight_unsymkl_gauss(PCObject &o1, PCObject &o2)
{
    int dim = o1.gaussian.dim;
    Eigen::MatrixXd multicov = Eigen::MatrixXd(3,3);
    multicov = o2.gaussian.cov_inverse * o1.gaussian.covariance;
    Eigen::VectorXd mean = o2.gaussian.mean-o1.gaussian.mean;
    double unsymkl_12 = (multicov.trace()
                         + mean.transpose()*o2.gaussian.cov_inverse*mean
                         + log(o1.gaussian.cov_determinant/o2.gaussian.cov_determinant)-dim) / 2.;
//    cout<<"kl: "<<unsymkl_12<<endl;
    return unsymkl_12;
}

double weight_symkl_gauss(PCObject &o1, PCObject &o2)
{
    double symkl = weight_unsymkl_gauss(o1,o2) + weight_unsymkl_gauss(o2,o1);
 //   cout<<"============"<<symkl<<endl;
    return symkl;

//    double dist = sqrt(pow(o1.gaussian.mean[0]-o2.gaussian.mean[0],2) + pow(o1.gaussian.mean[1]-o2.gaussian.mean[1],2) + pow(o1.gaussian.mean[2]-o2.gaussian.mean[2],2));
//    return dist;

}

double weight_l2_gauss(PCObject &o1, PCObject &o2)
{
    // l2 distance
    Eigen::MatrixXd covsum = o1.gaussian.covariance+o2.gaussian.covariance;
    Eigen::VectorXd meandiff = o1.gaussian.mean - o2.gaussian.mean;
    Eigen::MatrixXd inv = covsum.inverse();
    double det = covsum.determinant();
    double a = meandiff.transpose()*inv*meandiff;
    double l2 = 2.-2.*(1./sqrt(pow(2*pi,3)*det)) * exp(-0.5*a);
    if(l2 < 0) l2 = 0.;
    return l2;
}

double weight_closestPoints(PCObject &o1, PCObject &o2)
{
    int m = o1.points.size();
    int n = o2.points.size();
    double dist = (o1.points[0].pos - o2.points[0].pos).norm();
    cout<<"m: "<<m<<" n: "<<n<<endl;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = (o1.points[i].pos - o2.points[j].pos).norm();
            if(dist_ij < dist)  dist = dist_ij;
        }
    }
    return dist;
}

/*
double weight_l2_points(PCObject &o1, PCObject &o2)
{
    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010

    double energy1 = 0;
    double energy2 = 0;
    double energy3 = 0;
    double cross_term;
    double scale = 0.05;
    int m = o1.points.size();
    int n = o2.points.size();

    //    scale = SQR(scale);
    cross_term = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < m; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o1.points[i].pos[0] - o1.points[j].pos[0]);
            dist_ij += SQR(o1.points[i].pos[1] - o1.points[j].pos[1]);
            dist_ij += SQR(o1.points[i].pos[2] - o1.points[j].pos[2]);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy1 = cross_term / (m * m) / (scale*2*pi*sqrt(4*pi));
    //    printf("energy1 %f\n", energy1);
    cross_term = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o1.points[i].pos[0] - o2.points[j].pos[0]);
            dist_ij += SQR(o1.points[i].pos[1] - o2.points[j].pos[1]);
            dist_ij += SQR(o1.points[i].pos[2] - o2.points[j].pos[2]);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy2 = cross_term / (m * n) / (scale*2*pi*sqrt(4*pi));
    //    printf("energy2 %f\n", energy2);
    cross_term = 0.;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o2.points[i].pos[0] - o2.points[j].pos[0]);
            dist_ij += SQR(o2.points[i].pos[1] - o2.points[j].pos[1]);
            dist_ij += SQR(o2.points[i].pos[2] - o2.points[j].pos[2]);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy3 = cross_term / (n * n) / (scale*2*pi*sqrt(4*pi));
    //    printf("energy3 %f\n", energy3);
    //    printf("weight %f\n", (maxDist - (energy1 - 2*energy2 + energy3))/maxDist);
    double maxDist = 10000000;
    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;
    //    return energy2;
}

double weight_energy2(PCObject &o1, PCObject &o2)
{
    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010

    double energy1 = 0;
    double energy2 = 0;
    double energy3 = 0;
    double cross_term;
    double scale = 0.01;
    int m = o1.points.size();
    int n = o2.points.size();

    cross_term = 0.;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double dist_ij = 0;

            dist_ij += SQR(o1.points[i].pos[0] - o2.points[j].pos[0]);
            dist_ij += SQR(o1.points[i].pos[1] - o2.points[j].pos[1]);
            dist_ij += SQR(o1.points[i].pos[2] - o2.points[j].pos[2]);

            double cost_ij = exp(-0.5 * dist_ij / (SQR(scale)*2));
            cross_term += cost_ij;
        }
    }
    energy2 = cross_term / (m * n) / (scale*2*pi*sqrt(2*pi));
    return energy2;
    //    return energy2;
}
*/
/*
double weight_gaussian(Gaussian &g1, Gaussian &g2)
{
    double energy1 = 0.;
            Eigen::MatrixXd cov = g1.covariance + g1.covariance;
            Eigen::VectorXd mean = g1.mean - g1.mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy1 += gauss;

    double energy2 = 0.;

            cov = g1.covariance + g2.covariance;
            mean = g1.mean - g2.mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy2 += gauss;

    double energy3 = 0.;
            cov = g2.covariance + g2.covariance;
            mean = g2.mean - g2.mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy3 += gauss;
double maxDist = 10000000;
    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;

}

double weight_gaussian_predictive(Gaussian &g1, Gaussian &g2)
{
//    maxDist = 100000;   // for 0.01 scale
    double maxDist = 100000000;    // for 0.02 scale
    double energy1 = 0.;
            Eigen::MatrixXd cov = g1.predictive_covariance + g1.predictive_covariance;
            Eigen::VectorXd mean = g1.predictive_mean - g1.predictive_mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy1 += gauss;

    double energy2 = 0.;

            cov = g1.predictive_covariance + g2.predictive_covariance;
            mean = g1.predictive_mean - g2.predictive_mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy2 += gauss;

    double energy3 = 0.;
            cov = g2.predictive_covariance + g2.predictive_covariance;
            mean = g2.predictive_mean - g2.predictive_mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,6)*cov.determinant())*exp(-0.5*a);
            energy3 += gauss;
//    cout<<(maxDist - (energy1 - 2*energy2 + energy3))/maxDist<<endl;
    return (maxDist - (energy1 - 2*energy2 + energy3))/maxDist;

}
*/

double weight_gaussian_predictive_rev(Gaussian &g1, Gaussian &g2)
{
    int dim = g1.dim;
    double energy1 = 0.;
            Eigen::MatrixXd cov = g1.predictive_covariance + g1.predictive_covariance;
            Eigen::VectorXd mean = g1.predictive_mean - g1.predictive_mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy1 += gauss;

    double energy2 = 0.;

            cov = g1.predictive_covariance + g2.predictive_covariance;
            mean = g1.predictive_mean - g2.predictive_mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy2 += gauss;

    double energy3 = 0.;
            cov = g2.predictive_covariance + g2.predictive_covariance;
            mean = g2.predictive_mean - g2.predictive_mean;
            invij = cov.inverse();
            a = mean.transpose()*invij*mean;
            gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy3 += gauss;
//    cout<<(maxDist - (energy1 - 2*energy2 + energy3))/maxDist<<endl;
    return energy1 - 2*energy2 + energy3;

}
