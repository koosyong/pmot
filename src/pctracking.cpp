#include "pctracking.h"
#include "pctracking_weight.h"
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

bool isDebug_comp = 1;

PCTracking::PCTracking(int _isDebug, string _frame_id, ros::Publisher* _pub_scene, ros::Publisher* _pub_model, int dimension, double cont_sampling, double cont_simplify, double segTol)
{
    isDebug = _isDebug;
    frame_id = _frame_id;
    pub_scene = _pub_scene;
    pub_model = _pub_model;
    maxFrame = 1000;
    object_tracks = new PCTrackContainer(maxFrame);
    imft_loc = new IMFT<PCObject>(100, maxFrame, 20, &weight_symkl_gauss, &weight_symkl_gauss);

    dg_ambiguity_nodes = new ListDigraph::NodeMap<ObjectNode>(dg_ambiguity);
    dg_ambiguity_arcs = new ListDigraph::ArcMap<Weight>(dg_ambiguity);

    scale = cont_sampling;
    percent = cont_simplify;

    dim = dimension;
    if(dim == 3)
        thrProbScene = 100; // dim = 3
    if(dim == 6)
        thrProbScene = 2000000; // dim = 6

    segmentation_tolerance = segTol;
    segmentation_minSize = 15;
    segmentation_maxSize = 25000000;

    maxProbAssociation = 0.001;

    cnt = 0;
    maxID = 100;

    boundary.isInit = 1;
    boundaryMargin = 0.02;

    boundary.x_max = 10.8;
    boundary.x_min = -10.8;
    boundary.y_max = 10.8;
    boundary.y_min = -10.8;
    boundary.z_min = -12;
    boundary.z_max = 12;

    srand(time(NULL));
    r = new int[maxID];
    g = new int[maxID];
    b = new int[maxID];

    for(int i=0;i<maxID;i++){
        r[i] = rand() % 256;
        g[i] = rand() % 256;
        b[i] = rand() % 256;
    }
    r[0] = 255; r[1] = 0;    r[2] = 0;  r[3] = 255;
    g[0] =  0;  g[1] = 255;  g[2] = 0;  g[3] = 255;
    b[0] =  0;  b[1] = 0;    b[2] = 255;b[3] = 0;

    pCloud_lastModel.reset(new Cloud);
    pCloud_particles.reset(new Cloud);

    m_partFilters.resize(30);
}

PCTracking::~PCTracking()
{
    delete object_tracks;
    delete imft;

    delete[] r;
    delete[] g;
    delete[] b;
}

void PCTracking::run(CloudPtr _pCloud)
{
    double lastT, nowT, computationT;

    pCloud.reset(new Cloud);
    pCloud_lastModel.reset(new Cloud);
    pCloud_lastModel.reset(new Cloud);
    pCloud_particles.reset(new Cloud);

    pCloud = _pCloud;
    if(object_tracks->numTracks() < 1 && pCloud->points.size() <= segmentation_minSize) return;

    objects_prev.clearAll();
    objects_loc.clearAll();
    objects_modified.clearAll();
    objects_clear.clearAll();
    objects_falseSep.clearAll();
    objects_falseMerg.clearAll();


    falselySepareted.clear();
    falselyMerged.clear();


    // individuation by location
    lastT = pcl::getTime ();
    indvLoc();
    if(isDebug) cout<<"Individuation by location end. computation time : "<<pcl::getTime()-lastT<<", # of objects : "<<objects_loc.objects.size()<<endl;

    // modification
    if(object_tracks->numTracks()!=0){
        // object tracks in the previous time as a feedback
        for(int i=0;i<object_tracks->numTracks();i++){
            PCObject object;
            if(object_tracks->isNewlyUpdated(i)){
                object = object_tracks->tracks.at(i)->lastFrame().object;
                object.id = object_tracks->tracks.at(i)->id;
                objects_prev.objects.push_back(object); // to avoid assinging new object id
            }
        }
        if(isDebug) cout<<"# of feedback objects : "<<objects_prev.objects.size()<<endl;

        // ambiguity test
        lastT = pcl::getTime ();
        ambiguityTest();
        if(isDebug) cout<<"Ambiguitytest end. computation time : "<<pcl::getTime()-lastT<<endl;
        clearObjects();
        if(isDebug) cout<<"# of clear objects : "<<objects_clear.objects.size()<<endl;


        modifyFalselySeparated();
        if(isDebug) cout<<"# of false separated objects : "<<objects_falseSep.objects.size()<<endl;

        // individuation by feature
        lastT = pcl::getTime ();
        modifyFalselyMerged();


        if(isDebug) cout<<"Ambiguitytest end. computation time : "<<pcl::getTime()-lastT<<endl;
        if(isDebug) cout<<"# of false merged objects : "<<objects_falseMerg.objects.size()<<endl;

    }
    else{
        for(int i=0;i<objects_loc.objects.size();i++){
            objects_modified.makeNewObject(objects_loc.objects.at(i));
        }
    }

    // object indexing
    lastT = pcl::getTime ();

    indexing();
    updateTracks();

    if(isDebug) cout<<"Indexing by location end. computation time : "<<pcl::getTime()-lastT<<endl;

//    // ------------ for defrand: GMM update for each track
//    for(int i=0;i<object_tracks->numTracks();i++){
//        PCObject* objectLast = &(object_tracks->tracks.at(i)->lastFramePtr()->object);
//        PCObject* objectPrev = &(object_tracks->tracks.at(i)->framePtrFromLast(1)->object);
//        //        objectLast->initGaussian(3);

//        if(objectLast->state == NOGMM){
//            if(objectPrev->state == NOGMM){
//                //                objectPrev->initialGMM(scale, percent);
//            }
//            if(objectPrev->state == PRIORGMM || objectPrev->state == POSTGMM){
////                cout<<"Prev's state: "<<objectPrev->state<<endl;
////                cout<<"Prev's gmm size "<<objectPrev->gmm.size()<<endl;
////                cout<<"Last's id: "<<objectLast->id<<endl;
//                //                objectLast->initialGMM(scale, percent);
//                //                objectLast->filteringGMM_EM(objectPrev);
//                //                objectLast->filteringGMM_incrementalEM(objectPrev, percent);

//            }
//        }
//    }
    cnt ++;
}

void PCTracking::updateTracks()
{
    cout<<"update tracks"<<endl;
    object_tracks->currentT = cnt;
    PCTrackContainer* mft_tracks = (PCTrackContainer*)(imft_loc->extractTracks());
    //    bool *isUpdated = new bool[object_tracks->tracks.size()];
    //    for(int i=0;i<object_tracks->tracks.size();i++) isUpdated[i] = 0;

    //    cout<<"mft size: "<<mft_tracks->tracks.size()<<" object track size: "<<object_tracks->tracks.size()<<endl;

    int mftsize = mft_tracks->tracks.size();
    int objectsize = object_tracks->tracks.size();

    //    if(objectsize == 0 && mftsize != 0){
    //        for(int i=0;i<mftsize;i++){
    //            object_tracks->tracks.push_back(mft_tracks->tracks.at(i));
    //        }
    //    }

    for(int i=0;i<mftsize;i++){
        int trackID = mft_tracks->tracks.at(i)->id;
        //        cout<<"mft track ID: "<<trackID<<endl;
        // find trackID in object_tracks
        bool isTrackFound = 0;
        for(int j=0;j<objectsize;j++){
            //            cout<<"object track ID: "<<object_tracks->tracks.at(j)->id<<endl;
            if(object_tracks->tracks.at(j)->id == trackID){
                isTrackFound = 1;
                PCObject object = mft_tracks->tracks.at(i)->lastFrame().object;
                int time = mft_tracks->tracks.at(i)->lastFrame().time;
                object_tracks->push_back_track(trackID, object, time);
            }
        }
        if(!isTrackFound){
            // make a new track with the ID
            int framesize = mft_tracks->tracks.at(i)->frames.size();

            object_tracks->createTrack(mft_tracks->tracks.at(i)->frames.at(0).object, mft_tracks->tracks.at(i)->frames.at(0).time, mft_tracks->tracks.at(i)->id);
            if(framesize > 1){
                for(int j=1;j<framesize;j++){
                    object_tracks->push_back_track(mft_tracks->tracks.at(i)->id, mft_tracks->tracks.at(i)->frames.at(j).object, mft_tracks->tracks.at(i)->frames.at(j).time);
                }
            }
        }
    }
    //  delete tracks that is not inside of window_long
    object_tracks->deleteNoUpdatedTracks(maxFrame);
}

void PCTracking::indvLoc()
{    
    pCloud_seg.reset(new Cloud);
    segmentation(pCloud, objects_loc);
}

void PCTracking::ambiguityTest()
{
    int nObjectPrev = objects_prev.objects.size();
    int nObjectLoc = objects_loc.objects.size();

    // generate a directed graph
    dg_ambiguity.clear();

    // make nodes of the previous objects
    for(int i=0;i<nObjectPrev;i++){
        ObjectNode newNode;
        newNode.object = objects_prev.objects.at(i);
        newNode.type = UNDEFINED;
        newNode.time = PREVIOUS;
        (*dg_ambiguity_nodes)[dg_ambiguity.addNode()] = newNode;
    }
    // make nodes of the located objects
    double max_kl = 0.;
    for(int i=0;i<nObjectLoc;i++){
        ObjectNode newNode;
        newNode.object = objects_loc.objects.at(i);
        newNode.type = UNDEFINED;
        newNode.time = CURRENT;
        ListDigraph::Node nodeCurrent  = dg_ambiguity.addNode();
        (*dg_ambiguity_nodes)[nodeCurrent] = newNode;

        // building arcs
        for(ListDigraph::NodeIt nodePrev(dg_ambiguity); nodePrev != INVALID; ++nodePrev){
            if((*dg_ambiguity_nodes)[nodePrev].time == PREVIOUS && !isBoundary((*dg_ambiguity_nodes)[nodePrev].object)){
                Weight weight_p2c, weight_c2p;

                weight_p2c.kl = weight_unsymkl_gauss((*dg_ambiguity_nodes)[nodeCurrent].object, (*dg_ambiguity_nodes)[nodePrev].object);
                if(weight_p2c.kl > max_kl) max_kl = weight_p2c.kl;
                weight_p2c.l2 = 1. - weight_l2_gauss((*dg_ambiguity_nodes)[nodePrev].object, (*dg_ambiguity_nodes)[nodeCurrent].object)/2.;
                weight_p2c.type = UNDEFINED;

                weight_c2p.kl = weight_unsymkl_gauss((*dg_ambiguity_nodes)[nodePrev].object, (*dg_ambiguity_nodes)[nodeCurrent].object);
                if(weight_c2p.kl > max_kl) max_kl = weight_c2p.kl;
                weight_c2p.l2 = weight_p2c.l2;
                weight_c2p.type = UNDEFINED;

                // max eigen value of objectPrev
                Eigen::Matrix3d cov3d;
                for(int i=0;i<3;i++)
                    for(int j=0;j<3;j++)
                        cov3d(i,j)=(*dg_ambiguity_nodes)[nodePrev].object.gaussian.covariance(i,j);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov3d);
                Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();

                float maxVar = eigenvalues.maxCoeff();
                maxVar = sqrt(maxVar)*2;
//                double dist = ((*dg_ambiguity_nodes)[nodePrev].object.gaussian.mean-(*dg_ambiguity_nodes)[nodeCurrent].object.gaussian.mean).norm();
                float factor = 10;
//                if(dist < maxVar*factor && !isBoundary((*dg_ambiguity_nodes)[nodeCurrent].object)){
                double dist = weight_closestPoints((*dg_ambiguity_nodes)[nodePrev].object, (*dg_ambiguity_nodes)[nodeCurrent].object);
                cout<<"dist: "<<dist<<endl;
                if(dist < 0.01){// && !isBoundary((*dg_ambiguity_nodes)[nodeCurrent].object)){
//                if(weight_c2p.l2 < 0.5){// && !isBoundary((*dg_ambiguity_nodes)[nodeCurrent].object)){
                    (*dg_ambiguity_arcs)[dg_ambiguity.addArc(nodePrev,nodeCurrent)] = weight_p2c;
                    (*dg_ambiguity_arcs)[dg_ambiguity.addArc(nodeCurrent, nodePrev)] = weight_c2p;
//                    cout<<"connected "<<endl;
                }
            }
        }
    }

    //                if(weight_c2p.kl > L2CUT && !isBoundary((*dg_ambiguity_nodes)[nodeCurrent].object)){
    //                    (*dg_ambiguity_arcs)[dg_ambiguity.addArc(nodePrev,nodeCurrent)] = weight_p2c;
    //                    (*dg_ambiguity_arcs)[dg_ambiguity.addArc(nodeCurrent, nodePrev)] = weight_c2p;
    //                    cout<<"connected "<<endl;
    //                }


    // start loop
    int cnt_it = 0;
    while(1){
        initdg(UNDEFINED);

        // finding and assigning CLEAR nodes and specific PROBLEM nodes
        for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
            vector<ListDigraph::Arc> outArcs, inArcs;
            for (ListDigraph::OutArcIt a(dg_ambiguity, n); a != INVALID; ++a)
                outArcs.push_back(a);
            for (ListDigraph::InArcIt a(dg_ambiguity, n); a != INVALID; ++a)
                inArcs.push_back(a);
            (*dg_ambiguity_nodes)[n].nOut = outArcs.size();
            (*dg_ambiguity_nodes)[n].nIn = inArcs.size();

            // finding and assigning CLEAR nodes and specific PROBLEM nodes
            vector<ListDigraph::Arc> next_outArcs, next_inArcs;
            if(outArcs.size() == 1){
                // follow target node
                ListDigraph::Node nextNode = dg_ambiguity.target(outArcs.at(0));
                for (ListDigraph::OutArcIt a(dg_ambiguity, nextNode); a != INVALID; ++a)
                    next_outArcs.push_back(a);
                for (ListDigraph::InArcIt a(dg_ambiguity, nextNode); a != INVALID; ++a)
                    next_inArcs.push_back(a);

                if(inArcs.size() == 0 && next_inArcs.size() == 1){
                    if(next_outArcs.size() == 0){  // pi ----> ci : CLEAR
                        typeClear(n);
                        typeClear(nextNode);
                    }
                    else{   // pi ---> ci ---> pj : ci PROBLEM
                        typeProblem(nextNode);
                    }
                }
                if(inArcs.size() == 1){
                    if(next_inArcs.size() == 1 && next_outArcs.size() == 1){
                        int id_pi = dg_ambiguity.id(n);
                        int id_ci_next = dg_ambiguity.id(dg_ambiguity.target(next_outArcs.at(0)));
                        if(id_pi == id_ci_next) { // pi <---> ci : CLEAR
                            typeClear(n);
                            typeClear(nextNode);
                        }
                        else{   // cj ---> pi ---> ci ----> pj : pi, ci PROBLEM
                            typeProblem(n);
                            typeProblem(nextNode);
                        }
                    }
                    else if(next_inArcs.size() > 1 || next_outArcs.size() > 1){ // pi <---> ci <---> pj: ci PROBLEM
                        typeProblem(nextNode);
                    }
                }
            }
            else if(outArcs.size() == 0 && inArcs.size() == 1){
                // follow source node
                ListDigraph::Node beforeNode = dg_ambiguity.source(inArcs.at(0));
                for (ListDigraph::OutArcIt a(dg_ambiguity, beforeNode); a != INVALID; ++a)
                    next_outArcs.push_back(a);
                for (ListDigraph::InArcIt a(dg_ambiguity, beforeNode); a != INVALID; ++a)
                    next_inArcs.push_back(a);

                if(next_outArcs.size() == 1 && next_inArcs.size() == 0){    // pi <--- ci : pi, ci CLEAR
                    typeClear(n);
                    typeClear(beforeNode);
                }
                else if(next_inArcs.size() != 0){ // pi <--- ci <--- pj : ci PROBLEM
                    typeProblem(beforeNode);
                }
            }
            else if(outArcs.size() == 0 && inArcs.size() == 0){
                typeClear(n);
            }
        }

        // finding and assigning general PROBLEM nodes, if there still remain UNDEFINED nodes
        for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
            if((*dg_ambiguity_nodes)[n].type == UNDEFINED){
                vector<ListDigraph::Arc> outArcs, inArcs;
                for (ListDigraph::OutArcIt a(dg_ambiguity, n); a != INVALID; ++a)
                    outArcs.push_back(a);
                for (ListDigraph::InArcIt a(dg_ambiguity, n); a != INVALID; ++a)
                    inArcs.push_back(a);
                if(inArcs.size() > 1){  // cj ---> pi <--- ci : pi PROBLEM
                    typeProblem(n);
                }
                // when the node is not a CLEAR node (pi <---> ci)
                if(inArcs.size() == 1 && outArcs.size() >= 1){    // cj ---> pi ---> ci, ck : pi PROBLEM
                    typeProblem(n);
                }
            }
        }

 //               cout<<"---------------------------------#"<<cnt_it<<" After assigning type --------------------------------"<<endl;
   //             confirmDigraph();

        // break condition
        if(problemNodes.size() == 0 && problemArcs.size() == 0)
            break;

        // Pruning: erase arcs to make the minimum valued problem arc to be unproblematic
        // finding minimum valued arc
        ListDigraph::Arc minArc;
        double min_kl;
        int cnt_min = 0;
        for(ListDigraph::ArcIt a(dg_ambiguity); a != INVALID; ++a){
            if((*dg_ambiguity_arcs)[a].type == PROBLEM){
                double kl = (*dg_ambiguity_arcs)[a].kl;
                if(cnt_min == 0) {
                    min_kl = kl;
                    minArc = a;
                }
                else if(kl < min_kl){
                    min_kl = kl;
                    minArc = a;
                }
                cnt_min ++;
            }
        }
        (*dg_ambiguity_arcs)[minArc].type = DEFINED;
        ListDigraph::Node targetNode = dg_ambiguity.target(minArc);
        ListDigraph::Node sourceNode = dg_ambiguity.source(minArc);

        // finding self arc id, dual arc id
        int id_selfArc = dg_ambiguity.id(minArc);
        int id_dualArc = -1;
        for(ListDigraph::OutArcIt a(dg_ambiguity, targetNode); a != INVALID; ++a){
            int id_source = dg_ambiguity.id(dg_ambiguity.target(a));
            if(id_source == dg_ambiguity.id(sourceNode)){
                id_dualArc = dg_ambiguity.id(a);
            }

        }
        //        cout<<"selfArc: "<<id_selfArc<<", dualArc: "<<id_dualArc<<endl;
        // delete false arcs related with target node of the minarc (all inArcs except self arc, all other outArcs)
        vector<ListDigraph::Arc> delArcs;
        for(ListDigraph::InArcIt a(dg_ambiguity, targetNode); a != INVALID; ++a){
            if((*dg_ambiguity_arcs)[a].type != DEFINED){
                if(dg_ambiguity.id(a) != id_selfArc){
                    //                                    cout<<"Erase InArcs on Target: "<<dg_ambiguity.id(a)<<endl;
                    delArcs.push_back(a);
                }
            }
        }
        for(ListDigraph::OutArcIt a(dg_ambiguity, targetNode); a != INVALID; ++a){
            if((*dg_ambiguity_arcs)[a].type != DEFINED){
                //                                cout<<"Erase OutArcs on Target: "<<dg_ambiguity.id(a)<<endl;
                delArcs.push_back(a);
            }
        }

        // delete false arcs related with source node of the minarc (all other inArcs)
        for(ListDigraph::InArcIt a(dg_ambiguity, sourceNode); a != INVALID; ++a){
            if((*dg_ambiguity_arcs)[a].type != DEFINED){
                //                                cout<<"Erase InArcs on source: "<<dg_ambiguity.id(a)<<endl;
                delArcs.push_back(a);
            }
        }
        //        cout<<"delArcs Size:"<<delArcs.size()<<endl;
        if(delArcs.size() == 0){
            if(dg_ambiguity.valid(minArc))
                dg_ambiguity.erase(minArc);
        }
        else{
            for(int i=0;i<delArcs.size();i++)
                if(dg_ambiguity.valid(delArcs.at(i)))
                    dg_ambiguity.erase(delArcs.at(i));
        }
 //               cout<<"---------------------------------#"<<cnt_it<<" After pruning --------------------------------"<<endl;
  //              confirmDigraph();
        cnt_it ++;
    }
}

bool PCTracking::isBoundary(PCObject& object)
{
    if(fabs(object.gaussian.mean[0] - boundary.x_max) < boundaryMargin) return 1;
    if(fabs(object.gaussian.mean[0] - boundary.x_min) < boundaryMargin) return 1;
    if(fabs(object.gaussian.mean[1] - boundary.y_max) < boundaryMargin) return 1;
    if(fabs(object.gaussian.mean[1] - boundary.y_min) < boundaryMargin) return 1;
    //    if(fabs(object.gaussian.mean[2] - boundary.z_max) < boundaryMargin) return 1;
    //    if(fabs(object.gaussian.mean[2] - boundary.z_min) < boundaryMargin) return 1;
    return 0;
}

void PCTracking::initdg(Type type)
{
    problemArcs.clear();
    clearArcs.clear();
    problemNodes.clear();
    clearNodes.clear();
    // make all nodes and arcs as the given type
    for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
        (*dg_ambiguity_nodes)[n].type = type;
    }
    for(ListDigraph::ArcIt a(dg_ambiguity); a != INVALID; ++a){
        if((*dg_ambiguity_arcs)[a].type != DEFINED)
            (*dg_ambiguity_arcs)[a].type = type;
    }
}

void PCTracking::typeClear(ListDigraph::Node& node)
{
    (*dg_ambiguity_nodes)[node].type = CLEAR;
    clearNodes.push_back(node);
    // finding all related arcs
    for (ListDigraph::OutArcIt a(dg_ambiguity, node); a != INVALID; ++a){
        if((*dg_ambiguity_arcs)[a].type == UNDEFINED){
            (*dg_ambiguity_arcs)[a].type = CLEAR;
            clearArcs.push_back(a);
        }
    }
    for (ListDigraph::InArcIt a(dg_ambiguity, node); a != INVALID; ++a){
        if((*dg_ambiguity_arcs)[a].type == UNDEFINED){
            (*dg_ambiguity_arcs)[a].type = CLEAR;
            clearArcs.push_back(a);
        }
    }
}

void PCTracking::typeProblem(ListDigraph::Node& node)
{
    (*dg_ambiguity_nodes)[node].type = PROBLEM;
    problemNodes.push_back(node);
    // finding all related arcs
    for (ListDigraph::OutArcIt a(dg_ambiguity, node); a != INVALID; ++a){
        if((*dg_ambiguity_arcs)[a].type == UNDEFINED){
            (*dg_ambiguity_arcs)[a].type = PROBLEM;
            problemArcs.push_back(a);
        }
    }
    for (ListDigraph::InArcIt a(dg_ambiguity, node); a != INVALID; ++a){
        if((*dg_ambiguity_arcs)[a].type == UNDEFINED){
            (*dg_ambiguity_arcs)[a].type = PROBLEM;
            problemArcs.push_back(a);
        }
    }
}

void PCTracking::confirmDigraph()
{
    cout<<"---------------------------------DiGraph --------------------------------"<<endl;
    cout<<"---------Previous Nodes"<<endl;
    for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
        if((*dg_ambiguity_nodes)[n].time == PREVIOUS){
            cout<<"<Node "<<dg_ambiguity.id(n)<<"> Type:";
            switch((*dg_ambiguity_nodes)[n].type){
            case UNDEFINED: cout<<"UNDEFINED"; break;
            case CLEAR: cout<<"CLEAR"; break;
            case PROBLEM: cout<<"PROBLEM"; break;
            case DEFINED: cout<<"DEFINED"; break;
            }
            cout<<", #inArcs:"<<(*dg_ambiguity_nodes)[n].nIn<<", #outArcs:"<<(*dg_ambiguity_nodes)[n].nOut;
            cout<<", TrackID:"<<(*dg_ambiguity_nodes)[n].object.id<<endl;
            // out arcs
            for (ListDigraph::OutArcIt a(dg_ambiguity, n); a != INVALID; ++a){
                cout<<"    <Arc "<<dg_ambiguity.id(a)<<"> Type:";
                switch((*dg_ambiguity_arcs)[a].type){
                case UNDEFINED: cout<<"UNDEFINED"; break;
                case CLEAR: cout<<"CLEAR"; break;
                case PROBLEM: cout<<"PROBLEM"; break;
                case DEFINED: cout<<"DEFINED"; break;
                }
                cout<<", "<<dg_ambiguity.id(dg_ambiguity.source(a))<<" --------> "<<dg_ambiguity.id(dg_ambiguity.target(a))<<", kl:"<<(*dg_ambiguity_arcs)[a].kl<<", l2:"<<(*dg_ambiguity_arcs)[a].l2<<endl;
            }
        }
    }
    cout<<"---------Current Nodes"<<endl;
    for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
        if((*dg_ambiguity_nodes)[n].time == CURRENT){
            cout<<"<Node "<<dg_ambiguity.id(n)<<"> Type:";
            switch((*dg_ambiguity_nodes)[n].type){
            case UNDEFINED: cout<<"UNDEFINED"; break;
            case CLEAR: cout<<"CLEAR"; break;
            case PROBLEM: cout<<"PROBLEM"; break;
            case DEFINED: cout<<"DEFINED"; break;
            }
            cout<<", #inArcs:"<<(*dg_ambiguity_nodes)[n].nIn<<", #outArcs:"<<(*dg_ambiguity_nodes)[n].nOut;
            cout<<endl;
            // out arcs
            for (ListDigraph::OutArcIt a(dg_ambiguity, n); a != INVALID; ++a){
                cout<<"    <Arc "<<dg_ambiguity.id(a)<<"> Type:";
                switch((*dg_ambiguity_arcs)[a].type){
                case UNDEFINED: cout<<"UNDEFINED"; break;
                case CLEAR: cout<<"CLEAR"; break;
                case PROBLEM: cout<<"PROBLEM"; break;
                case DEFINED: cout<<"DEFINED"; break;
                }
                cout<<", "<<dg_ambiguity.id(dg_ambiguity.target(a))<<" <-------- "<<dg_ambiguity.id(dg_ambiguity.source(a))<<", kl:"<<(*dg_ambiguity_arcs)[a].kl<<", l2:"<<(*dg_ambiguity_arcs)[a].l2<<endl;
            }
        }
    }
    cout<<"------------------------------------------------------------------------"<<endl;

    //    cout<<"---------Arcs"<<endl;
    //    for(ListDigraph::ArcIt a(dg_ambiguity); a != INVALID; ++a){
    //        cout<<"<Arc> id:"<<dg_ambiguity.id(a)<<", source:"<<dg_ambiguity.id(dg_ambiguity.source(a))<<", target:"<<dg_ambiguity.id(dg_ambiguity.target(a))<<", kl:"<<(*dg_ambiguity_arcs)[a].kl<<", l2:"<<(*dg_ambiguity_arcs)[a].l2<<endl;
    //    }

}

void PCTracking::clearObjects()
{
    for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
        if((*dg_ambiguity_nodes)[n].time == CURRENT && (*dg_ambiguity_nodes)[n].type == CLEAR){
//            (*dg_ambiguity_nodes)[n].object.initGaussian(dim);
            objects_clear.makeNewObject((*dg_ambiguity_nodes)[n].object);
            objects_modified.makeNewObject((*dg_ambiguity_nodes)[n].object);
        }
    }
}

void PCTracking::modifyFalselySeparated()
{
    // Finding falsely segmented: prev, no clear, 0 inarcs, multi outarcs
    for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
        if((*dg_ambiguity_nodes)[n].time == PREVIOUS && (*dg_ambiguity_nodes)[n].type != CLEAR){
            if((*dg_ambiguity_nodes)[n].nIn == 0 &&  (*dg_ambiguity_nodes)[n].nOut > 1){
                FalseObjects falseObjects;
                falseObjects.parent = (*dg_ambiguity_nodes)[n].object;
                PCObject mergedObject;
                for (ListDigraph::OutArcIt a(dg_ambiguity, n); a != INVALID; ++a){
                    PCObject childObject = (*dg_ambiguity_nodes)[dg_ambiguity.target(a)].object;
                    falseObjects.childs.push_back(childObject);
                    // merge
                    for(int i=0;i<childObject.points.size();i++){
                        mergedObject.insert(childObject.points.at(i));
                    }

                }
                mergedObject.initGaussian(dim);
                objects_falseSep.makeNewObject(mergedObject);
                objects_modified.makeNewObject(mergedObject);
                falselySepareted.push_back(falseObjects);
            }
        }
    }
}


void PCTracking::modifyFalselyMerged()
{
    // Finding falsely merged: current, no clear, 0 inarcs, multi outarcs
    for(ListDigraph::NodeIt n(dg_ambiguity); n != INVALID; ++n){
        if((*dg_ambiguity_nodes)[n].time == CURRENT && (*dg_ambiguity_nodes)[n].type != CLEAR){
            if((*dg_ambiguity_nodes)[n].nIn == 0 &&  (*dg_ambiguity_nodes)[n].nOut > 1){
                cout<< "Iteration !!!!!!!!!!!!!!!!!!!"<<endl;

                FalseObjects falseObjects;
                PCObject parentObject = (*dg_ambiguity_nodes)[n].object;
                PCObjectContainer childObjects, registeredObjects;
                falseObjects.parent = parentObject;
                for (ListDigraph::OutArcIt a(dg_ambiguity, n); a != INVALID; ++a){
                    PCObject childObject = (*dg_ambiguity_nodes)[dg_ambiguity.target(a)].object;
                    falseObjects.childs.push_back(childObject);
                    childObjects.makeNewObject(childObject);
                }
                falselyMerged.push_back(falseObjects);
                // separation
                // registration
                //                registrationGMMs(childObjects, parentObject, registeredObjects);
                // pointmatching
                PCObjectContainer object_separated;
                //                pointMatching(childObjects, parentObject, unmatchedPoints, object_separated);
                pointMatching(childObjects, parentObject, unmatchedPoints, object_separated);
                //                pointMatching(childObjects, parentObject, unmatchedPoints, object_separated);
                for(int i=0;i<object_separated.objects.size();i++){
                    objects_modified.makeNewObject(object_separated.objects.at(i));
                    objects_falseMerg.makeNewObject(object_separated.objects.at(i));
                }
            }
        }
    }
}

void PCTracking::pointMatching(PCObjectContainer& predictiveObjects, PCObject& parentObject, CloudPtr unmatchedPoints, PCObjectContainer& objects_separated)
{
    double lastT, nowT, computationT;
    if(isDebug_comp) cout<<"pointMatching start"<<endl;
    lastT = pcl::getTime ();
    //setting up search trees for the predictiveObjects
    vector<CloudPtr> predictiveObjectsPcl;
    CloudPtr parentObjPcl = parentObject.toPointCloud();

    float newRatio;
    ros::param::get("/dhri/multipleObjectTracking/gpu/newratio", newRatio);

    double beginPart=pcl::getTime();

    std::vector<int> theinds;
    for (size_t i=0; i<predictiveObjects.objects.size(); i++){
        CloudPtr cloudtemp(new Cloud);
        cloudtemp = predictiveObjects.objects.at(i).toPointCloud_model();
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloudtemp, *cloudtemp, indices);

        float minDist =100.f ; int partInd;
        Vector3d thisc = predictiveObjects.objects.at(i).centroid.pos;
        for(int j=0; j<object_tracks->tracks.size(); j++){
            Vector3d c1 = object_tracks->tracks.at(j)->getFrameFromLast(0).object.centroid.pos;
            float thisdist = (thisc-c1).norm();
            if (thisdist<minDist){
                minDist = thisdist; partInd = j;
            }
        }


        cout<<"partInd is "<<partInd<<endl;
        theinds.push_back(partInd);
        m_partFilters[partInd].setdata(parentObjPcl,  cloudtemp, cnt);
        cloudtemp = m_partFilters[partInd].getResult();

//        transformPointCloud(*obj, *cloudtemp, trans);


        cout<<"cloudtemp size is "<<cloudtemp->points.size()<<endl;

//        pCloud_particles = m_partFilters[partInd].toCloud();
        *pCloud_particles += *m_partFilters[partInd].toCloud();
        *pCloud_lastModel += *cloudtemp;
        predictiveObjectsPcl.push_back(cloudtemp);
    }

    computationT=pcl::getTime()-beginPart;
    if(isDebug_comp) cout<<"particle filtering done computation time : "<<computationT<<endl;


    int nObjects = predictiveObjects.objects.size();
    for(int i=0;i<nObjects;i++){
        PCObject object;
        objects_separated.makeNewObject(object);
    }
    nowT = pcl::getTime ();
    computationT = (nowT-lastT);
    if(isDebug_comp) cout<<"icp end. computation time : "<<computationT<<endl;

    lastT = pcl::getTime ();



    vector<pcl::KdTreeFLANN<PointT>::Ptr > trees;
//    vector<pcl::search::Octree<PointT>::Ptr> trees;
    for (size_t i=0; i<predictiveObjects.objects.size(); i++){
        pcl::KdTreeFLANN<PointT>::Ptr temptree(new pcl::KdTreeFLANN<PointT>);
//        pcl::search::Octree<PointT>::Ptr temptree(new pcl::search::Octree<PointT>(0.005));

        temptree->setInputCloud(predictiveObjectsPcl.at(i));
        trees.push_back(temptree);
    }
    cout<<"tree setting finished "<<endl;

    for(int i=0;i<parentObject.points.size();i++){
        Point point = parentObject.points.at(i);
        PointT searchPoint = point.toPclPt();

        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        float minDist = 1000;
        float secondDist=10000;
        int theInd = -1;
        PointT nearestPt;
        for(int j=0; j<nObjects; j++){            
            trees.at(j)->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
            float thisdist = 0.f;
            for (size_t k = 0; k < pointIdxNKNSearch.size (); k++){
                thisdist += sqrt(pointNKNSquaredDistance.at(k));
//                thisdist += colordist(searchPoint, predictiveObjectsPcl[j]->points[pointIdxNKNSearch[0]]);
            }
//            if (thisdist < 0.003f){
//                   objects_separated.objects.at(j).insert(point);
//            }
            if (thisdist<minDist){
                secondDist = minDist;
                minDist = thisdist; theInd = j;
                nearestPt = predictiveObjectsPcl[j]->points[pointIdxNKNSearch[0]];
            }else if(thisdist<secondDist){
                secondDist = thisdist;
            }
            //            std::vector<int> pointIdxNKNSearch;
            //            std::vector<float> pointNKNSquaredDistance;
//            int thisnumber = trees.at(j)->radiusSearch(searchPoint, 0.004f, pointIdxNKNSearch, pointNKNSquaredDistance);
//            if (thisnumber > minDist){
//                minDist = thisnumber; theInd = j;
//            }
        }

        if (minDist /*< 0.02f*/ ){
            objects_separated.objects.at(theInd).insert(point);
         //   searchPoint.r = r[theinds[theInd]];
         //   searchPoint.g = g[theinds[theInd]];
         //   searchPoint.b = b[theinds[theInd]];
         //   pCloud_seg->points.push_back(searchPoint);
//            if (minDist < 0.01f /*&& secondDist>0.1f*/){
//                cout<<"mindist: "<<minDist <<" second dist " <<secondDist<<endl;

                float beta = 0.5f;
                point.pos[0] = point.pos[0] + beta*(nearestPt.x - point.pos[0]);
                point.pos[1] = point.pos[1] + beta*(nearestPt.y - point.pos[1]);
                point.pos[2] = point.pos[2] + beta*(nearestPt.z - point.pos[2]);


                float rand_ = static_cast <float> (rand()) / static_cast <float> (RAND_MAX)*100.f;
                if (rand_< newRatio )
                   objects_separated.objects.at(theInd).insert_model(point);  // new observation
 //           }
        }
    }


    std::vector<int> indexes(1);
    std::vector<float> sqDists(1);

    for (size_t i=0; i<predictiveObjectsPcl.size(); i++){
        for (size_t j=0; j<predictiveObjectsPcl[i]->points.size(); j++){
            float rand_ = static_cast <float> (rand()) / static_cast <float> (RAND_MAX)*100.f;
            if (rand_< (100.f -newRatio)/* && sqDists[0]<(0.02*0.02)*/ ) {
                PointT pt1 = predictiveObjectsPcl[i]->points[j];
                Point temppt1 (pt1, dim);
                objects_separated.objects.at(i).insert_model(temppt1);    // from old object
            }
        }
    }

    for(int i=0;i<nObjects;i++){
        m_partFilters[theinds[i]].updateModel(objects_separated.objects.at(i).toPointCloud_model());

        cout<<"objects_sep"<<i<<"size "<< objects_separated.objects.at(i).points.size()<<endl;
        objects_separated.objects.at(i).initGaussian(dim);
    }
    nowT = pcl::getTime ();
    computationT = (nowT-lastT);
    if(isDebug_comp) cout<<"octree search end. computation time : "<<computationT<<endl;
}

void PCTracking::indexing()
{
    cout<<"object indexing, # of modified objects"<<objects_modified.objects.size()<<endl;
    // IMFT_LOC
    if(objects_modified.objects.size() > 0){
        vector<PCObject*> objectPtrs;
        for(int i=0;i<objects_modified.objects.size();i++){
            PCObject* objectPtr = &(objects_modified.objects.at(i));
            objectPtrs.push_back(objectPtr);
        }
        imft_loc->setFrame(objectPtrs, cnt);
      //          imft_loc->confirmDGraph();
        imft_loc->matching();
        imft_loc->updateTracks();
        //        imft_loc->confirmDGraph();
    }
    // extrack tracks from imft
}

void PCTracking::segmentation(CloudPtr pCloud, PCObjectContainer& objects)
{
    double lastT, nowT, computationT;
    if(isDebug_comp) cout<<"segmentation start"<<endl;
    lastT = pcl::getTime ();
    // Creating the KdTree object for the search method of the extraction
    //    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (pCloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (segmentation_tolerance); // 2cm
    ec.setMinClusterSize (segmentation_minSize);
    ec.setMaxClusterSize (segmentation_maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud( pCloud);
    ec.extract (cluster_indices);

    int id = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PCObject object;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            Point point(pCloud->points[*pit], dim);
            object.insert(point);

            PointT pointseg = pCloud->points[*pit];
            pointseg.r = r[id];
            pointseg.g = g[id];
            pointseg.b = b[id];
            pCloud_seg->points.push_back(pointseg);

        }
        // generate gaussian distribution of object
        object.initGaussian(dim);
        objects.makeNewObject(object);

        id++;
    }
}
