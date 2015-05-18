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
#ifndef TRACK_H
#define TRACK_H

template<class Object>
class Track
{
public:
    typedef struct{
        Object object;
        int id;
        int time;
    } Frame;
    typedef struct{
        Frame ptr;
        int frame;
        int nodeId;
    } Node;
    typedef struct{
        int id;
        bool isIn;
        int nFrame;
        int edgeID;
        Frame ptr;
        bool isTrack;
        int nTrack;
    } V;

public:
    Track(int _id, int _maxFrame = 1000);
    void insert(Object &object, int time);
    Frame lastFrame(){return frames.at(frames.size()-1);};
    Frame* lastFramePtr(){return &(frames.at(frames.size()-1));};
    Frame getFrameFromLast(int n){return frames.at(frames.size()-1-n);};
    Frame* framePtrFromLast(int n){return &(frames.at(frames.size()-1-n));};
    void updateObjectAtFrame(int time, Object& object);
    void terminate();
    int frameSize(){return frames.size();};

public:
    vector<Frame> frames;
    int id;
    int maxFrame;
};

#include "track.hpp"
#endif // TRACK_H
