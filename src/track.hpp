#include "track.h"

template<class Object>
Track<Object>::Track(int _id, int _maxFrame)
    :id(_id), maxFrame(_maxFrame)
{
}

template<class Object>
void Track<Object>::insert(Object &object, int time)
{    
    Frame frame;
    frame.object = object;
    frame.time = time;
    frames.push_back(frame);
    if(frames.size() == maxFrame)
        frames.erase(frames.begin());

}

template<class Object>
void Track<Object>::terminate()
{
    frames.clear();
}

template<class Object>
void Track<Object>::updateObjectAtFrame(int time, Object& object)
{
    for(int i=0;i<frames.size();i++){
        if(frames.at(i).time == time){
            frames.at(i).object = object;
        }
    }
}
