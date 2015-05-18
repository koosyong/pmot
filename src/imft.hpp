#include "imft.h"

template<class Object>
IMFT<Object>::IMFT(int _window_short, int _window_long, int _maxID, funcWeight _weight, funcWeight _weight_fast)
    :window_short(_window_short), window_long(_window_long), maxID(_maxID), weight(_weight), weight_fast(_weight_fast)
{
    trackContainer = new TrackContainerT(window_long, maxID);
    cnt = 0;
    m_isDebug = 0;
    m_gNodeMap = new ListGraph::NodeMap<V>(m_g);
    m_gEdgeMap = new ListGraph::EdgeMap<double>(m_g);
    m_newTrackID = 0;
}

template<class Object>
IMFT<Object>::~IMFT()
{
    delete trackContainer;
    delete m_gNodeMap;
    delete m_gEdgeMap;
}

template<class Object>
void IMFT<Object>::setFrame(vector<Object*> objects, int stamp)
{
    cnt++;
    m_currentT = stamp;
    vector<Ptr> ptrs;
    for(int i=0;i<objects.size();i++){
        Ptr ptr;
        ptr.object = *objects.at(i);
        ptr.time = stamp;
        ptr.id = i;
        ptrs.push_back(ptr);
    }

    if(cnt > window_short) movingWindow();
    addToDGraph(ptrs);  // extension
}

template<class Object>
void IMFT<Object>::confirmDGraph()
{
    printf("-----------------------Graph----------------------\n");
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n)
        printf("<Node> id:%d nFrame:%d isIn:%d edge:%d IsTrack:%d, nTrack:%d, ptrID:%d, time:%d\n", m_g.id(n), (*m_gNodeMap)[n].nFrame, (*m_gNodeMap)[n].isIn, (*m_gNodeMap)[n].edgeID, (*m_gNodeMap)[n].isTrack, (*m_gNodeMap)[n].nTrack, (*m_gNodeMap)[n].ptr.id, (*m_gNodeMap)[n].ptr.time);
    for(ListGraph::EdgeIt a(m_g); a != INVALID; ++a)
        printf("<Edge> id:%d weight:%f uNode:%d vNode:%d uID:%d  vID:%d\n", m_g.id(a), (*m_gEdgeMap)[a], m_g.id(m_g.u(a)), m_g.id(m_g.v(a)), (*m_gNodeMap)[m_g.u(a)].ptr.id, (*m_gNodeMap)[m_g.v(a)].ptr.id);
    printf("--------------------------------------------------\n");
}

template<class Object>
void IMFT<Object>::movingWindow()
{
    if(m_isDebug)   printf("moving window\n");
    vector<ListGraph::Node> vecDelNode;
    // erase cnt-m_nWindow frame nodes
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].nFrame <= cnt-window_short){
            // IMFT : delete a node outside of window_short that connected in the future
            //        if not (terminal node) remain the node till in the window_long
            //        remain a terminal node only in a track
            if(!(*m_gNodeMap)[n].isIn && ((*m_gNodeMap)[n].edgeID >= 0 || ((*m_gNodeMap)[n].edgeID < 0 && (*m_gNodeMap)[n].isTrack!=1))){
                // make edgeID of connected node -1
                if((*m_gNodeMap)[n].edgeID >= 0){
                    ListGraph::Edge e = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                    ListGraph::Node v = m_g.v(e);
                    (*m_gNodeMap)[v].edgeID = -1;

                    if(m_isDebug)   printf("edge initialization (-1) of node # %d\n", m_g.id(v));
                    // erase edge connected with the node
                    m_g.erase(e);
                }
                vecDelNode.push_back(n);
                vecDelNode.push_back(m_g.nodeFromId(m_g.id(n)-1));
            }
            // delete a node outside of window_long
            else if(!(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].nFrame <= cnt-window_long){
                if((*m_gNodeMap)[n].edgeID >= 0){
                    // make edgeID of connected node -1
                    ListGraph::Edge e = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                    ListGraph::Node v = m_g.v(e);
                    (*m_gNodeMap)[v].edgeID = -1;
                    if(m_isDebug)   printf("edge initialization (-1) of node # %d\n", m_g.id(v));
                    // erase edge connected with the node
                    m_g.erase(e);
                }
                // erase node
                vecDelNode.push_back(n);
                vecDelNode.push_back(m_g.nodeFromId(m_g.id(n)-1));
            }
        }
    }
    for(int i=0;i<vecDelNode.size();i++){
        ListGraph::Node n = vecDelNode.at(i);
        if(m_isDebug)   printf("erase old node %d of frame %d\n",m_g.id(n), (*m_gNodeMap)[n].nFrame);
        m_g.erase(n);
    }
}

template<class Object>
void IMFT<Object>::addToDGraph(VecPtr ptrs)
{
    // save old edges
    m_vecOldEdge.clear();
    for(ListGraph::EdgeIt e(m_g); e != INVALID; ++e){
        m_vecOldEdge.push_back(m_g.id(e));
    }

    m_maxWeight = 0.;
    // making nodes
    for(int i=0;i<ptrs.size();i++){
        // inNode
        ListGraph::Node xi = m_g.addNode();
        V vi;
        vi.id = m_g.id(xi);
        vi.isIn = 1;
        vi.ptr = ptrs[i];
        vi.nFrame = cnt;
        vi.edgeID = -1;
        vi.isTrack = 0;
        vi.nTrack = 0;
        (*m_gNodeMap)[xi] = vi;

        // outNode
        ListGraph::Node xo = m_g.addNode();
        V vo;
        vo.id = m_g.id(xo);
        vo.isIn = 0;
        vo.ptr = ptrs[i];
        vo.nFrame = cnt;
        vo.edgeID = -1;
        vo.isTrack = 0;
        vo.nTrack = 0;
        (*m_gNodeMap)[xo] = vo;

        // connection to previous nodes

        for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
            if((*m_gNodeMap)[n].nFrame != vi.nFrame && !(*m_gNodeMap)[n].isIn){
                double w = weight((*m_gNodeMap)[m_g.nodeFromId(m_g.id(n)-1)].ptr.object, vi.ptr.object);
                (*m_gEdgeMap)[m_g.addEdge(n,xi)] = w;
                if(w > m_maxWeight) m_maxWeight = w;


            }
        }

    }
    // convert weights to reversed weights from 0 to 1
    for(ListGraph::EdgeIt a(m_g); a != INVALID; ++a){
//        (*m_gEdgeMap)[a] = 1.-(*m_gEdgeMap)[a]/m_maxWeight;
        (*m_gEdgeMap)[a] = 1.-(*m_gEdgeMap)[a]/MAXWEIGHT;
    }
}

template<class Object>
void IMFT<Object>::extension()
{

}

template<class Object>
void IMFT<Object>::matching()
{
    // maximum weighted matching
    MaxWeightedMatching<ListGraph, ListGraph::EdgeMap<double> > mwm(m_g, *m_gEdgeMap);
    mwm.run();
    double wsum = mwm.matchingWeight();
    if(m_isDebug)   printf("Max = %f\n", wsum);
    m_wsum = wsum;

    // make maximum path cover C
    /* for all edges
    // if it is not matched, delete edges, if it was old edge -> save oldEdge
    // if it is matched, liking edge to nodes  : save edge id to the connedted nodes
    // find correction edges : matching edges having a node of old edges
    */
    vector<ListGraph::Edge> vecDelOldEdge;
    for(ListGraph::EdgeIt e(m_g); e != INVALID; ++e){
        if(mwm.matching(e)){
            if(m_isDebug)   printf("Edge of Maximum Path Cover : %d\n", m_g.id(e));
            (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
            (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);
        }
        else{
            for(int i=0;i<m_vecOldEdge.size();i++){
                if(m_g.id(e) == m_vecOldEdge.at(i)) {   // edges deleted by correction edge
                    vecDelOldEdge.push_back(e);
                    (*m_gNodeMap)[m_g.v(e)].edgeID = -1;
                    break;
                }
            }
            m_g.erase(e);
        }
    }
    // false hypothesis
    /* for all correction edges
    // find false hypothesis : edges having a directed path from an edge replaced by a correction edge
    //                       : find a previous track having an edge replaced by a correction edge
    //                       : false hypotheses = edges of tracks
    // delete all false hypotheses
    */
    for(int i=0;i<vecDelOldEdge.size();i++){
        ListGraph::Edge delEdge = vecDelOldEdge.at(i);
        if(m_isDebug)
            printf("At cnt %d, edge %d is an old edge deleted by a correction edge\n",cnt, m_g.id(delEdge));
        // delete false hypotheses
        int vId = m_g.id(m_g.v(delEdge));
        (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack = 0;
        (*m_gNodeMap)[m_g.nodeFromId(vId)].isTrack = 0;
        (*m_gNodeMap)[m_g.nodeFromId(vId+1)].nTrack = 0;
        (*m_gNodeMap)[m_g.nodeFromId(vId+1)].isTrack = 0;

        if(m_isDebug)
            printf("delete node %d from the track %d\n", vId ,(*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack);

        ListGraph::Node uNode = m_g.nodeFromId(vId+1);
        int fhId = (*m_gNodeMap)[uNode].edgeID;
        while(fhId != -1){
            if(m_isDebug)
                printf("False Hypothsis %d is deleted\n", fhId);
            ListGraph::Edge fhEdge = m_g.edgeFromId(fhId);
            (*m_gNodeMap)[m_g.u(fhEdge)].edgeID = -2;   // check for a node connected with a deleted edge
            (*m_gNodeMap)[m_g.v(fhEdge)].edgeID = -2;   // check for a node connected with a deleted edge

            vId = m_g.id(m_g.v(fhEdge));
            (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack = 0;
            (*m_gNodeMap)[m_g.nodeFromId(vId)].isTrack = 0;
            (*m_gNodeMap)[m_g.nodeFromId(vId+1)].nTrack = 0;
            (*m_gNodeMap)[m_g.nodeFromId(vId+1)].isTrack = 0;

            if(m_isDebug)
                printf("At cnt %d, delete node %d(frame %d) from the track %d\n", cnt, vId, (*m_gNodeMap)[m_g.nodeFromId(vId)].nFrame, (*m_gNodeMap)[m_g.nodeFromId(vId)].nTrack);

            uNode = m_g.nodeFromId(vId+1);
            fhId = (*m_gNodeMap)[uNode].edgeID;
            m_g.erase(fhEdge);
        }
    }
    // new edges
    /* if cnt < m_nWindow : for i=1 to cnt
       if cnt <= m_nWindow : for i=cnt-m_nWindow+1 to cnt
    // make two frame correspondance Vi ~ Vi+1
    */
    vector<ListGraph::Node> vecUFrame, vecVFrame;
    if(cnt < window_short){
        for(int i=1; i<cnt; i++){
            vecUFrame.clear();
            vecVFrame.clear();
            // 2-frame corresponding
            for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
                //                if((*m_gNodeMap)[n].nFrame > i+1) break;
                if((*m_gNodeMap)[n].nFrame == i && !(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecUFrame.push_back(n);
                if((*m_gNodeMap)[n].nFrame == i+1 && (*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecVFrame.push_back(n);
            }
            if(vecUFrame.size() != 0 && vecVFrame.size() != 0){
                if(m_isDebug)   printf("2-Frame Corresponding of %d and %d\n", i, i+1);
                twoFrameCorresponding(vecUFrame, vecVFrame);
            }
        }
    }
    else{
        for(int i=cnt-window_short+1; i<cnt; i++){
            vecUFrame.clear();
            vecVFrame.clear();
            // 2-frame corresponding
            for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
                //                if((*m_gNodeMap)[n].nFrame > i+1) break;
                if((*m_gNodeMap)[n].nFrame == i && !(*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecUFrame.push_back(n);
                if((*m_gNodeMap)[n].nFrame == i+1 && (*m_gNodeMap)[n].isIn && (*m_gNodeMap)[n].edgeID == -2)
                    vecVFrame.push_back(n);
            }
            if(vecUFrame.size() != 0 && vecVFrame.size() != 0){
                if(m_isDebug)   printf("2-Frame Corresponding of %d and %d\n", i, i+1);
                twoFrameCorresponding(vecUFrame, vecVFrame);
            }
        }
    }
}


template<typename Object>
void IMFT<Object>::twoFrameCorresponding(vector<ListGraph::Node> vecUFrame, vector<ListGraph::Node> vecVFrame)
{
    if(m_isDebug)   printf("2-Frame Corresponding : # F1 - %d, # F2 - %d\n", vecUFrame.size(), vecVFrame.size());
    // make graph, weight map
    ListGraph g;
    ListGraph::NodeMap<V> gNodeMap(g);
    ListGraph::EdgeMap<double> gEdgeMap(g);

    // make nodes of UFrame : save node id of UFrame
    for(int i=0;i<vecUFrame.size();i++){
        ListGraph::Node n = g.addNode();
        V v;
        v.id = m_g.id(vecUFrame.at(i));
        v.ptr = (*m_gNodeMap)[vecUFrame.at(i)].ptr;
        v.nFrame = 1;
        gNodeMap[n] = v;
    }
    // make nodes of VFrame : save node id of VFrame
    double max = 0;
    for(int i=0;i<vecVFrame.size();i++){
        ListGraph::Node n = g.addNode();
        V v;
        v.id = m_g.id(vecVFrame.at(i));
        v.ptr = (*m_gNodeMap)[vecVFrame.at(i)].ptr;
        v.nFrame = 2;
        gNodeMap[n] = v;

        // connection

        for(ListGraph::NodeIt pn(g); pn != INVALID; ++pn){
            if(gNodeMap[pn].nFrame != v.nFrame){
                double w = weight(gNodeMap[pn].ptr.object, v.ptr.object);
                gEdgeMap[g.addEdge(pn,n)] = w;
                if(w > max) max = w;
            }
        }



    }
    // convert weights to reversed weights from 0 to 1
    for(ListGraph::EdgeIt a(g); a != INVALID; ++a)
        gEdgeMap[a] = 1.-gEdgeMap[a]/max;

    // maximum weighted matching
    MaxWeightedMatching<ListGraph, ListGraph::EdgeMap<double> > mwm(g, gEdgeMap);
    mwm.run();
    int wsum = mwm.matchingWeight();
    if(m_isDebug)   printf("2-Frame Max = %d\n", wsum);

    // make edges of original graph using original nodes' ids
    for(ListGraph::EdgeIt e(g); e != INVALID; ++e){
        if(mwm.matching(e)){
            int origUId = gNodeMap[g.u(e)].id;
            int origVId = gNodeMap[g.v(e)].id;
            ListGraph::Node newU, newV;
            newU = m_g.nodeFromId(origUId);
            newV = m_g.nodeFromId(origVId);
            if(m_isDebug)   printf("2-Frame Connection %d, %d nodes\n", origUId, origVId);

            ListGraph::Edge e = m_g.addEdge(newU,newV);
            double w = weight((*m_gNodeMap)[newU].ptr.object, (*m_gNodeMap)[newV].ptr.object);
            (*m_gEdgeMap)[e] = 1.-w/m_maxWeight;
            (*m_gNodeMap)[m_g.u(e)].edgeID = m_g.id(e);
            (*m_gNodeMap)[m_g.v(e)].edgeID = m_g.id(e);
        }
    }
}
/*
#include "gmmfiltering.h"
template<class Object>
void IMFT<Object>::filtering()
{
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        // update privious frame objects
        if((*m_gNodeMap)[n].ptr.time == m_currentT-1){
            if((*m_gNodeMap)[n].isIn != 1 && (*m_gNodeMap)[n].edgeID != -1 && (*m_gNodeMap)[n].isTrack == 1){
                Object prior = (*m_gNodeMap)[n].ptr.object;
                if(prior.isParamExist){
                    cout<<"*********************************************Filtering start"<<endl;

                    ListGraph::Edge edge = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                    ListGraph::Node nodeIn = m_g.nodeFromId(m_g.id(m_g.v(edge)));
                    ListGraph::Node nodeOut = m_g.nodeFromId(m_g.id(nodeIn)+1);
                    Object measurement = (*m_gNodeMap)[nodeIn].ptr.object;
                    cout<<"# track of prior : "<<(*m_gNodeMap)[n].nTrack<<endl;
                    cout<<"# track of measurement : "<<(*m_gNodeMap)[nodeIn].nTrack<<endl;
                    GMMFiltering filter(prior, measurement, prior.trans_param);
                    (*m_gNodeMap)[nodeIn].ptr.object = filter.posterior;
                    (*m_gNodeMap)[nodeOut].ptr.object = filter.posterior;
                }
            }
        }
    }
}
*/
template<class Object>
void IMFT<Object>::updateTracks()
{
    trackContainer->currentT = m_currentT;
    // if m_currentT == 0 : make all objects in a frame initial tracks.
    if(m_currentT == 0){

    }
    // 각 node에 있는 nTrack을 update
    //    m_g.id(n), (*m_gNodeMap)[n].nFrame, (*m_gNodeMap)[n].isIn, (*m_gNodeMap)[n].edgeID, (*m_gNodeMap)[n].isTrack, (*m_gNodeMap)[n].nTrack, (*m_gNodeMap)[n].ptr.id, (*m_gNodeMap)[n].ptr.time
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        // update privious frame objects
        if((*m_gNodeMap)[n].ptr.time == m_currentT-1){
            if((*m_gNodeMap)[n].isIn != 1 && (*m_gNodeMap)[n].isTrack == 1){
                trackContainer->updateObject((*m_gNodeMap)[n].nTrack, (*m_gNodeMap)[n].ptr.time, (*m_gNodeMap)[n].ptr.object);
            }
        }

        // new frame objects
        if((*m_gNodeMap)[n].ptr.time == m_currentT){
            if((*m_gNodeMap)[n].isIn == 1 && (*m_gNodeMap)[n].edgeID != -1 && (*m_gNodeMap)[n].isTrack == 0){
                // follow edge
                ListGraph::Edge edge = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                ListGraph::Node node = m_g.nodeFromId(m_g.id(m_g.u(edge)));
                // if connected node is a track
                if((*m_gNodeMap)[node].nTrack != 0){
                    (*m_gNodeMap)[n].isTrack = 1;
                    int nTrack = (*m_gNodeMap)[node].nTrack;
                    (*m_gNodeMap)[n].nTrack = nTrack;
                    ListGraph::Node nOut = m_g.nodeFromId(m_g.id(n)+1);
                    (*m_gNodeMap)[nOut].isTrack = 1;
                    (*m_gNodeMap)[nOut].nTrack = nTrack;

                    //                    // filtering
                    //                    Object prior = (*m_gNodeMap)[node].ptr.object;
                    //                    if(prior.isParamExist){
                    //                        cout<<"*********************************************Filtering start"<<endl;

                    //                        Object measurement = (*m_gNodeMap)[n].ptr.object;
                    //                        GMMFiltering filter(prior, measurement, prior.trans_param);
                    //                        (*m_gNodeMap)[n].ptr.object = filter.posterior;
                    //                        (*m_gNodeMap)[nOut].ptr.object = filter.posterior;
                    //                    }
                    // update track of nTrack
                    trackContainer->push_back_track(nTrack, (*m_gNodeMap)[n].ptr.object, (*m_gNodeMap)[n].ptr.time);
                }
                // if connected node is not a track --> make a new track
                else{
                    //                    m_newTrackID ++;
                    m_newTrackID = trackContainer->newId();
                    (*m_gNodeMap)[node].isTrack = 1;
                    (*m_gNodeMap)[node].nTrack = m_newTrackID;
                    ListGraph::Node nodeOut = m_g.nodeFromId(m_g.id(node)-1);
                    (*m_gNodeMap)[nodeOut].isTrack = 1;
                    (*m_gNodeMap)[nodeOut].nTrack = m_newTrackID;

                    (*m_gNodeMap)[n].isTrack = 1;
                    (*m_gNodeMap)[n].nTrack = m_newTrackID;
                    ListGraph::Node nOut = m_g.nodeFromId(m_g.id(n)+1);
                    (*m_gNodeMap)[nOut].isTrack = 1;
                    (*m_gNodeMap)[nOut].nTrack = m_newTrackID;

                    //                    printf("CreateTrack\n");
                    trackContainer->createTrack((*m_gNodeMap)[node].ptr.object, (*m_gNodeMap)[node].ptr.time, m_newTrackID);
                    trackContainer->push_back_track(m_newTrackID, (*m_gNodeMap)[n].ptr.object, (*m_gNodeMap)[n].ptr.time);
                }
            }
            if((*m_gNodeMap)[n].isIn == 1 && (*m_gNodeMap)[n].edgeID == -1 && (*m_gNodeMap)[n].isTrack == 0){
                m_newTrackID = trackContainer->newId();

                (*m_gNodeMap)[n].isTrack = 1;
                (*m_gNodeMap)[n].nTrack = m_newTrackID;
                ListGraph::Node nOut = m_g.nodeFromId(m_g.id(n)+1);
                (*m_gNodeMap)[nOut].isTrack = 1;
                (*m_gNodeMap)[nOut].nTrack = m_newTrackID;

                //                    printf("CreateTrack\n");
                trackContainer->createTrack((*m_gNodeMap)[n].ptr.object, (*m_gNodeMap)[n].ptr.time, m_newTrackID);
            }
        }
    }
    // delete tracks that is not inside of window_long
    trackContainer->deleteNoUpdatedTracks(window_long);
}



template<class Object>
typename IMFT<Object>::TrackContainerT* IMFT<Object>::extractTracks()
{
    return trackContainer;
}

template<class Object>
typename IMFT<Object>::VecObjectPtr IMFT<Object>::getUnmatchedObjects()
{
    VecObjectPtr objects;
    if(trackContainer->numTracks() == 0)
        return objects;

    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time == m_currentT){
            if((*m_gNodeMap)[n].isIn == 1 && (*m_gNodeMap)[n].edgeID == -1){
                objects.push_back(&((*m_gNodeMap)[n].ptr.object));
            }
        }
    }
    return objects;
}

template<class Object>
typename IMFT<Object>::VecObjectPtr IMFT<Object>::getTerminalNodes()
{
    VecObjectPtr objects;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time != m_currentT){
            if((*m_gNodeMap)[n].isIn != 1 && (*m_gNodeMap)[n].edgeID == -1){
                objects.push_back(&((*m_gNodeMap)[n].ptr.object));
            }
        }
    }
    return objects;
}

template<class Object>
typename IMFT<Object>::VecObjectPtr IMFT<Object>::getTerminalNodesLastFrame()
{
    VecObjectPtr objects;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time == m_currentT-1){
            if((*m_gNodeMap)[n].isIn != 1 && (*m_gNodeMap)[n].edgeID == -1){
                objects.push_back(&((*m_gNodeMap)[n].ptr.object));
            }
        }
    }
    return objects;
}


template<class Object>
typename IMFT<Object>::VecObjectPtr IMFT<Object>::getUnmatchedTracks()
{
    VecObjectPtr objects;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time <= m_currentT-1){
            if((*m_gNodeMap)[n].isIn != 1 && (*m_gNodeMap)[n].edgeID == -1 && (*m_gNodeMap)[n].isTrack == 1){
                objects.push_back(&((*m_gNodeMap)[n].ptr.object));
            }
        }
    }
    return objects;
}

template<class Object>
void IMFT<Object>::getMaximumMatchedTrack(ObjectPtr object, ObjectPtr &maxTrack, double &wHypothesis, ObjectPtr &objectOrigin, double &wOrigin)
{
    VecObjectPtr tracks, objects;
    vector<double> weights_hypo, weights_origin;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time == m_currentT-1){
            if((*m_gNodeMap)[n].isIn != 1 && (*m_gNodeMap)[n].edgeID != -1 && (*m_gNodeMap)[n].isTrack == 1){
                tracks.push_back(&((*m_gNodeMap)[n].ptr.object));
                weights_hypo.push_back(weight_fast(((*m_gNodeMap)[n].ptr.object), *object));
                ListGraph::Edge edge = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                ListGraph::Node nodeObject = m_g.nodeFromId(m_g.id(m_g.v(edge)));
                objects.push_back(&((*m_gNodeMap)[nodeObject].ptr.object));
                weights_origin.push_back((*m_gEdgeMap)[edge]);


            }
        }
    }
    // find max weight and object
    double max = 0.;
    int maxIndex = 0;
    for(int i=0;i<weights_hypo.size();i++){
        if(weights_hypo.at(i) >= max){
            max = weights_hypo.at(i);
            maxIndex = i;
        }
    }
    maxTrack = tracks.at(maxIndex);
    wHypothesis = weights_hypo.at(maxIndex);
    objectOrigin = objects.at(maxIndex);
    wOrigin = weights_origin.at(maxIndex);

}

template<class Object>
void IMFT<Object>::getMaximumMatchedObject(ObjectPtr trackUnmatched, ObjectPtr &maxObject, double &wHypothesis, ObjectPtr &trackOrigin, double &wOrigin)
{
    VecObjectPtr tracks, objects;
    vector<double> weights_hypo, weights_origin;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time == m_currentT){
            if((*m_gNodeMap)[n].isIn == 1 && (*m_gNodeMap)[n].edgeID != -1){
                objects.push_back(&((*m_gNodeMap)[n].ptr.object));
                weights_hypo.push_back(weight_fast(((*m_gNodeMap)[n].ptr.object), *trackUnmatched));
                ListGraph::Edge edge = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                ListGraph::Node nodeObject = m_g.nodeFromId(m_g.id(m_g.u(edge)));
                tracks.push_back(&((*m_gNodeMap)[nodeObject].ptr.object));
                weights_origin.push_back((*m_gEdgeMap)[edge]);
            }
        }
    }
    // find max weight and object
    double max = 0.;
    int maxIndex = 0;
    for(int i=0;i<weights_hypo.size();i++){
        //        printf("weight of %d : %f\n", i, weights_hypo.at(i));
        if(weights_hypo.at(i) >= max){
            max = weights_hypo.at(i);
            maxIndex = i;
        }
    }
    maxObject = objects.at(maxIndex);
    wHypothesis = weights_hypo.at(maxIndex);
    trackOrigin = tracks.at(maxIndex);
    wOrigin = weights_origin.at(maxIndex);
}

template<class Object>
bool IMFT<Object>::deleteLastFrame()
{
    if(cnt <= 1) return 0;

    vector<ListGraph::Node> vecDelNode;
    for(ListGraph::NodeIt n(m_g); n != INVALID; ++n){
        if((*m_gNodeMap)[n].ptr.time == m_currentT && (*m_gNodeMap)[n].isIn){
            // delete node n
            // make edgeID of connected node -1
            if((*m_gNodeMap)[n].edgeID != -1){
                ListGraph::Edge e = m_g.edgeFromId((*m_gNodeMap)[n].edgeID);
                ListGraph::Node u = m_g.u(e);
                (*m_gNodeMap)[u].edgeID = -1;
                // erase edge connected with the node
                m_g.erase(e);
            }
            vecDelNode.push_back(m_g.nodeFromId(m_g.id(n)+1));
            vecDelNode.push_back(n);
        }
    }
    for(int i=0;i<vecDelNode.size();i++){
        ListGraph::Node n = vecDelNode.at(i);
        m_g.erase(n);
    }
    return 1;
}
