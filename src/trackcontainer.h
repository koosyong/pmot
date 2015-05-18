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
#ifndef TRACKCONTAINER_H
#define TRACKCONTAINER_H

#include "track.h"
#include <vector>
using namespace std;

template<class Object>
class TrackContainer
{
    typedef Track<Object> TrackT;
    typedef vector<TrackT*> VecTrackPtr;
public:
    TrackContainer(int _maxFrame = 1000, int _maxID = 1000);
    ~TrackContainer();

public:
    inline int numTracks(){return tracks.size();};
    bool createTrack(Object &object, int initT);
    bool createTrack(Object &object, int initT, int id);
    bool merge(TrackContainer<Object> container);
    bool push_back_track(int id, Object &object, int time);
    void deleteNoUpdatedTracks(int size);
    int newId();
    bool updateObject(int id, int time, Object &object);
    bool isNewlyUpdated(int at);
    vector<int> deletedTrackIDs;

public:
    VecTrackPtr tracks;
    int currentT;

protected:
    int maxFrame;
    int maxID;
    int *r, *g, *b;

    int oldCnt;
    int oldCnt_gaussians;
};

#include "trackcontainer.hpp"
#endif // TRACKCONTAINER_H
