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
#ifndef PCTRACKING_WEIGHT_H
#define PCTRACKING_WEIGHT_H

#include "pcobject.h"

#define SQR(X)  ((X)*(X))
#define pi 3.141592
/*
typedef struct{
    Eigen::Matrix3d covariance;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    Eigen::Vector3d mean;
} GAUSS1;
*/

//GAUSS1 gauss1(PCObject &o);
//double weight_cov(PCObject &o1, PCObject &o2);
//double weight_l2_gmm1(PCObject &o1, PCObject &o2);
//double weight_l2(PCObject &o1, PCObject &o2);
double weight_l2_rev(PCObject &o1, PCObject &o2);
double weight_loc(PCObject &o1, PCObject &o2);
double weight_symkl_gauss(PCObject &o1, PCObject &o2);
double weight_unsymkl_gauss(PCObject &o1, PCObject &o2);
double weight_l2_gauss(PCObject &o1, PCObject &o2);
double weight_closestPoints(PCObject &o1, PCObject &o2);

//double weight_l2_points(PCObject &o1, PCObject &o2);
//double weight_energy2(PCObject &o1, PCObject &o2);
//double weight_gaussian(Gaussian &g1, Gaussian &g2);
//double weight_gaussian_predictive(Gaussian &g1, Gaussian &g2);
double weight_gaussian_predictive_rev(Gaussian &g1, Gaussian &g2);

#endif // PCTRACKING_WEIGHT_H
