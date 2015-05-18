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
#ifndef PCOBJECT_H
#define PCOBJECT_H

#include "vector"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/flann.hpp"

#include <iostream>

#include <lemon/connectivity.h>
//using namespace lemon;

#include "gaussian.h"
#include "imft.h"
#include "gaussiantrackcontainer.h"

typedef enum{
    SIMPLE_HCKL, SIMPLE_HCL2, SIMPLE_FA
} SIMPLE;

typedef enum{
    NOGMM, PRIORGMM, POSTGMM
} STATE;

#define MIN_WEIGHT_RATIO 5

class PCObject
{
public:
    typedef double(*funcWeightGaussian)(Gaussian &g1, Gaussian &g2);
    typedef struct{
        Point u;
        Point v;
        double weight;
    }Edge;
    typedef struct{
        ListGraph::Node u;
        ListGraph::Node v;
        double weight_vel;
        double weight_pos;
    }Edge_spatial;

public:
    PCObject();    
    PCObject(int _id);
    ~PCObject();

public:
    CloudPtr toPointCloud();
    CloudPtr toPointCloud_model();
    void insert(Point p);
    void insert_model(Point p);
    Point getCentroid();
    void initialGMM(double _scale, double _percent);
//    void initialGMM(double _scale);

    void filteringGMM_EM(PCObject* prior);
    void filteringGMM_incrementalEM(PCObject* prior, double percent);
    double likelihood(Point point, Gaussian gaussian);
    double likelihood_standard(Point point, Gaussian gaussian);

    void initGaussian(int dim);
    double evalGMM(Point x);
    double evalClosestGMM(Point x);
    double evalNormedGMM(Point x, double den);
    double evalClosestNormedGMM(Point x, double den);
    void simplify(int dim, SIMPLE method, double ratio, int nCluster=0);
//    double L2ofGMMandPoints(double scale);
    void setTransParam(vnl_vector<double> param);
    void mergeTwoGMMs(PCObject* gmm1, PCObject* gmm2);
    void setScale(double _scale){scale = _scale;};

    void gaussianTrackingInit(int _window_short, int _window_long, int _maxID, funcWeightGaussian _weight_gaussian, funcWeightGaussian _weight_gaussian_fast);
    void gaussianTracking();
    void updatePredictiveParameters();
    void calculateVelocity();
    void makeTopology();
    double topology_weight(Gaussian g1, Gaussian g2);
    double topology_posweight_rev(Gaussian g1, Gaussian g2);
    double topology_velweight_rev(Gaussian g1, Gaussian g2);
    int componentGraph(vector<PCObject> &newObjects);

    void setid(int _id){id = _id;};
    int getid(){return id;};

public:
    vector<Point> points;
    vector<Point> points_model; // for object model
    vector<Gaussian> gmm;
    Point centroid;
    Gaussian gaussian;
    STATE state;
    int dimension;

    int id;
    vnl_vector<double> trans_param;
    bool isParamExist;

    IMFT<Gaussian> *imftGaussians;
//    GaussianTrackContainer *gaussianTracks;
    TrackContainer<Gaussian> *gaussianTracks;
    int cnt;
    vector<Gaussian*> frame;

public: // topology
    ListGraph* topology_graph;
    ListGraph::NodeMap<Gaussian> *topology_nodeMap;
    ListGraph::EdgeMap<double> *topology_edgeMap;
    vector<Edge> edges;
    double alpha;
    double th_edge;
//    double avgWeight;
    double *diffWeight;
    double filteredWeight;

public:
    double scale;
    double percent;

private:
    int window_short;
    int window_long;

};

#endif // PCOBJECT_H
