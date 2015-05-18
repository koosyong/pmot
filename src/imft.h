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
#ifndef IMFT_H
#define IMFT_H

#include <vector>
using namespace std;

#include <lemon/list_graph.h>
#include <lemon/matching.h>
using namespace lemon;

#include "trackcontainer.h"

//************************************************************* cafeful~!! ***********************
// you have to correct updateTracks() function
// in the case of false hypothesis,
// the tracks having false edges are not deleted in trackcontainer
//************************************************************* cafeful~!! ***********************

#define MAXWEIGHT 1000.

template<class Object>
class IMFT
{
public:
    typedef TrackContainer<Object> TrackContainerT;
    typedef Track<Object> TrackT;
    typedef typename TrackT::Frame Ptr;
    typedef vector<Ptr> VecPtr;
    typedef typename TrackT::V V;
    typedef vector<TrackT> VecTrack;
    typedef double(*funcWeight)(Object &o1, Object &o2);
    typedef Object* ObjectPtr;
    typedef vector<Object*> VecObjectPtr;

public:
    IMFT(int _window_short = 10, int _window_long = 20, int _maxID=100, funcWeight _weight = 0, funcWeight _weight_fast = 0);
    ~IMFT();

public:
    void setFrame(vector<Object*> objects, int stamp);
    void confirmDGraph();
    void extension();
    void matching();
    void updateTracks();
    TrackContainerT* extractTracks();
    VecObjectPtr getUnmatchedObjects();
    VecObjectPtr getTerminalNodes();
    VecObjectPtr getTerminalNodesLastFrame();
    VecObjectPtr getUnmatchedTracks();
    void getMaximumMatchedTrack(ObjectPtr object, ObjectPtr &maxTrack, double &wHypothesis, ObjectPtr &objectOrigin, double &wOrigin);
    void getMaximumMatchedObject(ObjectPtr trackUnmatched, ObjectPtr &maxObject, double &wHypothesis, ObjectPtr &trackOrigin, double &wOrigin);
//    void filtering();
    bool deleteLastFrame();

    funcWeight weight, weight_fast;
    TrackContainerT *trackContainer;
    int cnt;    

private:
    void movingWindow();
    void addToDGraph(VecPtr ptrs);
    void twoFrameCorresponding(vector<ListGraph::Node> vecUFrame, vector<ListGraph::Node> vecVFrame);

public:
    int window_short, window_long;
    int maxID;

    bool m_isDebug;
    ListGraph m_g;
    ListGraph::NodeMap<V> *m_gNodeMap;
    ListGraph::EdgeMap<double> *m_gEdgeMap;
    double m_maxWeight;
    vector<int> m_vecOldEdge;
    double m_wsum;
    int m_currentT;
    int m_newTrackID;


};

#include "imft.hpp"
#endif // IMFT_H
