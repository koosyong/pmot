#ifndef PCTRACKING_H
#define PCTRACKING_H
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

#include "pcobjectcontainer.h"
#include "pctrackcontainer.h"
#include "imft.h"
#include "particlepose.h"

//#include "gmmfiltering.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/tracking/tracker.h>
#include <pcl/filters/filter.h>


#include <pcl/registration/icp.h>
#include <boost/thread.hpp>


#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>

//#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/gpu_extract_labeled_clusters.h>

#include <pcl/gpu/segmentation/impl/gpu_extract_labeled_clusters.hpp>
//#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>

#include <pcl/tracking/particle_filter_omp.h>


#include <pcl/surface/mls.h>

#include <lemon/list_graph.h>
using namespace lemon;

#define SQR(X)  ((X)*(X))
#define pi 3.141592
#define L2CUT -1
//#define L2CUT 1

// Useful macros
#define FPS_CALC(_WHAT_) \
    do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
{ \
    ROS_INFO("FPS of %s: %f", _WHAT_, double(count)/double(now - last)); \
    count = 0; \
    last = now; \
    } \
    }while(false)

typedef vector<PCObject*> VecObjectPtr;
typedef vector<PCObject> VecObject;
typedef struct{
    VecObjectPtr models;
    PCObject* scene;
    VecObject trasformedModels;
} Scene;

typedef struct{
    int pointID;
    int trackID;
} TrackPoint;

typedef struct{
    vector<TrackPoint> trackPoints;
    int time;
} Frame;

typedef enum{
    UNDEFINED, PROBLEM, CLEAR, PARENT, CHILD, DEFINED
} Type;

typedef enum{
    PREVIOUS, CURRENT
} Time;

typedef struct{
    Type type;
    Time time;
    PCObject object;
    int nOut, nIn;
} ObjectNode;

typedef struct{
    double kl, l2;
    Type type;
} Weight;

typedef struct{
    PCObject parent;
    vector<PCObject> childs;
} FalseObjects;

typedef struct{
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    bool isInit;
} BoundingBox;

class PCTracking
{
public:
    PCTracking(int _isDebug, string _frame_id, ros::Publisher* _pub_scene, ros::Publisher* _pub_model, int dimension, double cont_sampling, double cont_simplify, double segTol);
    ~PCTracking();

public:
    void run(CloudPtr _pCloud);
    inline CloudPtr getFilteredPC(){return pCloud;};
    inline CloudPtr getSegmentedPC(){return pCloud_seg;};
    inline CloudPtr getLastModel(){return pCloud_lastModel;}
    inline CloudPtr getParticles(){return pCloud_particles;}

private:
    void segmentationGMM(CloudPtr pCloud, PCObjectContainer& objects);
    void segmentation(CloudPtr pCloud, PCObjectContainer& objects);
   void pointMatching(PCObjectContainer& predictiveObjects, PCObject& parentObject, CloudPtr unmatchedPoints, PCObjectContainer& objects_separated);
    void trackingGaussians(PCObject* object, CloudPtr observedPoints, PCObjectContainer& updatedObjects);

    void indvLoc();

    void ambiguityTest();
    void typeProblem(ListDigraph::Node& node);
    void typeClear(ListDigraph::Node& node);
    void initdg(Type type);
    void confirmDigraph();

    void clearObjects();
    void modifyFalselySeparated();
    void modifyFalselyMerged(); // individuation by feature
    bool isBoundary(PCObject& object);

    void indexing();
    void updateTracks();

//    void procRegistration();
//    void procPointmatching();
//    void procGaussianTracking();
//    void procObjectTracking();Ek

private:
    string frame_id;
    ros::Publisher* pub_scene;
    ros::Publisher* pub_model;
    CloudPtr pCloud, pCloud_seg, pCloud_lastModel, pCloud_particles;
    PCObjectContainer objects_prev;
    PCObjectContainer objects_loc;
    PCObjectContainer objects_modified;
    PCObjectContainer objects_clear, objects_falseSep, objects_falseMerg;
    vector<FalseObjects> falselySepareted, falselyMerged;
    BoundingBox boundary;
    double boundaryMargin;

//    PCObjectContainer objects;
//    PCObjectContainer oldObjectsUpdated, oldObjectsNonupdated, predictiveObjects, updatedObjects, newObjects;
    CloudPtr unmatchedPoints;
    vector<CloudPtr> observedPointsList;
    int nObjects;
    int cnt;
    bool isDebug;

    vector<ListDigraph::Node> problemNodes;
    vector<ListDigraph::Arc> problemArcs;
    vector<ListDigraph::Node> clearNodes;
    vector<ListDigraph::Arc> clearArcs;
//    static double maxDist;

public:
    IMFT<PCObject> *imft;
    IMFT<PCObject> *imft_loc;

    PCTrackContainer *object_tracks;
    ListDigraph dg_ambiguity;
    ListDigraph::NodeMap<ObjectNode> *dg_ambiguity_nodes;
    ListDigraph::ArcMap<Weight> *dg_ambiguity_arcs;

private:
    vector<Particlepose> m_partFilters;
    double scale;
    double percent;
    int dim;
    double segmentation_tolerance;
    double segmentation_minSize;
    double segmentation_maxSize;
    double maxProbAssociation;
    double thrProbScene;
    int minPoints;

    int maxID;
    int *r, *g, *b;

    int maxFrame;

};

#endif // PCTRACKING_H
