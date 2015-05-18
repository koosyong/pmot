/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Telerobotics and Control Laboratory, KAIST (http://robot.kaist.ac.kr)
 *  Author : Seongyong Koo (Koosy) (http://koosy.blogspot.com, koosyong@gmail.com)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef GAUSSIAN_H
#define GAUSSIAN_H

//#include "pcobject.h"

#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

#include <vector>
#include <Eigen/Dense>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

using namespace std;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;


class Point
{
public:
    Point();
    Point(PointT point, int _dim);
    PointT toPclPt();


public:
    Eigen::Vector3d pos;
    Eigen::Vector3d rgb;
    int id;
    int dim;
};
typedef vector<Point> Points;

class Gaussian
{
public:
    Gaussian();
    Gaussian(int _dim);
//    Gaussian(CloudPtr points, double scale);
    void init(int _dim, Points points);

public:
    Eigen::VectorXd mean;
    Eigen::Vector3d velocity;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd cov_inverse;
    double cov_determinant;

    Eigen::VectorXd predictive_mean;
    Eigen::MatrixXd predictive_covariance;


    double weight;
    int nPoint;
    bool isEmpty;
    int dim;

    vnl_matrix<double> translation;
    vnl_matrix<double> rotation;

public:
//    double evalPoint(Point point);
    void updateParam(vnl_vector<double> newParam);
    void initPrediction();



    static void quaternion2rotation(vnl_vector<double> q, vnl_matrix<double>& R, vnl_matrix<double>& g1, vnl_matrix<double>& g2, vnl_matrix<double>& g3, vnl_matrix<double>& g4);
    static void quaternion2rotation(vnl_vector<double> q, vnl_matrix<double>& R);

};

#endif // GAUSSIAN_H
