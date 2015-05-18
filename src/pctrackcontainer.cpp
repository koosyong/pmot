#include "pctrackcontainer.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

PCTrackContainer::PCTrackContainer(int _maxFrame)
    :TrackContainer<PCObject>::TrackContainer(_maxFrame)
{
    oldCnt = 0;
    isUpdated = 1;
}

PCTrackContainer::PCTrackContainer()
    :TrackContainer<PCObject>::TrackContainer(1000)
{
    oldCnt = 0;
    isUpdated = 1;
}

void PCTrackContainer::iros2014()
{
    // hand avg : 107, 88, 82
    // black avg : 36, 35, 42
    // white avg : 216, 218, 232
    double avg;
    int kind;
    int cnt[3]; // 0: black, 1: white, 2: hand
    int error=0;
    int total=0;

    double ref[3];
    ref[0] = (36.1661 + 35.5952 + 42.2318) / 3.;
    ref[1] = (216 + 217.127 + 231.618) / 3.;
    ref[2] = (107.212 + 88.0612 + 82.3061) / 3.;
    for(int i=0;i<numTracks();i++){
        avg = 0.;
        cnt[0] = cnt[1] = cnt[2] = 0;
        int id = tracks.at(i)->id;
        if(tracks.at(i)->lastFrame().time == currentT){
            PCObject object;
            object = tracks.at(i)->lastFrame().object;
            total += object.points.size();
            for(int j=0;j<object.points.size();j++){
                int R = object.points.at(j).rgb[0];
                int G = object.points.at(j).rgb[1];
                int B = object.points.at(j).rgb[2];

                double RGB = (R + G + B) / 3.;
                avg += RGB;
                if(fabs(RGB-ref[0]) <= fabs(RGB-ref[1]) && fabs(RGB-ref[0]) <= fabs(RGB-ref[2]))
                    cnt[0] ++;
                else if(fabs(RGB-ref[1]) <= fabs(RGB-ref[0]) && fabs(RGB-ref[1]) <= fabs(RGB-ref[2]))
                    cnt[1] ++;
                else if(fabs(RGB-ref[2]) <= fabs(RGB-ref[1]) && fabs(RGB-ref[2]) <= fabs(RGB-ref[0]))
                    cnt[2] ++;
            }

            avg /= object.points.size();
            if(fabs(avg-ref[0]) <= fabs(avg-ref[1]) && fabs(avg-ref[0]) <= fabs(avg-ref[2])){
                kind = 0;
                error += cnt[1] + cnt[2];
            }
            else if(fabs(avg-ref[1]) <= fabs(avg-ref[0]) && fabs(avg-ref[1]) <= fabs(avg-ref[2])){
                kind = 1;
                error += cnt[0] + cnt[2];
            }
            else if(fabs(avg-ref[2]) <= fabs(avg-ref[1]) && fabs(avg-ref[2]) <= fabs(avg-ref[0])){
                kind = 2;
                error += cnt[1] + cnt[0];
            }
        }
        //        cout<<"ID: "<<id<<" kind: "<< kind<<" total: "<< total<<" error: "<< error<<endl;
    }
    cout<<total<<"  "<<error<<endl;

    //    numTruePoints = 0;
    //    numFalsePoints = 0;
    //    numTotalPoints = 0;

    //    for(int i=0;i<numTracks();i++){
    //        if(tracks.at(i)->lastFrame().time == currentT){
    //            PCObject object;
    //            object = tracks.at(i)->lastFrame().object;
    //            int numBlack = 0;
    //            int numWhite = 0;
    //            for(int j=0;j<object.points.size();j++){
    //                Point point = object.points.at(j);
    //                double avgrgb = (point.rgb[0]*3000 + point.rgb[1]*3000 + point.rgb[2]*3000) / 3;

    //                if(avgrgb > 128) numWhite ++;
    //                else numBlack ++;
    //            }
    //            if(numWhite > numBlack){
    //                numTruePoints += numWhite;
    //                numFalsePoints += numBlack;
    //            }
    //            else{
    //                numTruePoints += numBlack;
    //                numFalsePoints += numWhite;
    //            }
    //            numTotalPoints += object.points.size();
    //        }
    //    }
}

void PCTrackContainer::toPointCloudXYZI(Cloud &cloudOut)
{
    for(int i=0;i<numTracks();i++){
        if(!isNewlyUpdated(i))  continue;
        PCObject object;
        object = tracks.at(i)->lastFrame().object;
        int id = tracks.at(i)->id;
        for(int j=0;j<object.points.size();j++){
            PointT point;
            point.x = object.points.at(j).pos[0];
            point.y = object.points.at(j).pos[1];
            point.z = object.points.at(j).pos[2];
            point.r = r[id];
            point.g = g[id];
            point.b = b[id];
            //                point.intensity = id;
            cloudOut.points.push_back(point);
        }
    }
}

void PCTrackContainer::toPointCloudXYZI_model(Cloud &cloudOut)
{
    for(int i=0;i<numTracks();i++){
        if(!isNewlyUpdated(i))  continue;
        PCObject object;
        object = tracks.at(i)->lastFrame().object;
        int id = tracks.at(i)->id;
        if(object.points_model.size() == 0){
            PointT point;
            point.x = point.y = point.y = 0;
            point.r = point.g = point.b = 0;
            point.a = 0;
            cloudOut.points.push_back(point);
        }
        for(int j=0;j<object.points_model.size();j++){
            PointT point;
            point.x = object.points_model.at(j).pos[0];
            point.y = object.points_model.at(j).pos[1];
            point.z = object.points_model.at(j).pos[2];
            point.r = r[id];
            point.g = g[id];
            point.b = b[id];
            //                point.intensity = id;
            cloudOut.points.push_back(point);
        }
    }
}

visualization_msgs::Marker PCTrackContainer::toMarkerEdges()
{
    visualization_msgs::Marker edgeMarker;

    edgeMarker.header.frame_id = "/origin";
    edgeMarker.header.stamp = ros::Time();
    edgeMarker.ns = "edge";
    edgeMarker.id = 1;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.lifetime = ros::Duration(0);
    edgeMarker.scale.x = 0.003;
    edgeMarker.color.a = 1;
    edgeMarker.color.r = 1;
    edgeMarker.color.g = 0;
    edgeMarker.color.b = 0;


    for(int i=0;i<numTracks();i++){
        if(tracks.at(i)->lastFrame().time == currentT){
            PCObject object;
            object = tracks.at(i)->lastFrame().object;
            for(int j=0;j<object.edges.size();j++){
                Point p = object.edges.at(j).u;
                geometry_msgs::Point point;
                point.x = p.pos[0];
                point.y = p.pos[1];
                point.z = p.pos[2];
                edgeMarker.points.push_back(point);
                p = object.edges.at(j).v;
                point.x = p.pos[0];
                point.y = p.pos[1];
                point.z = p.pos[2];
                edgeMarker.points.push_back(point);
            }
        }
    }
    return edgeMarker;
}


visualization_msgs::MarkerArray PCTrackContainer::toMarkerGaussians()
{
    oldGaussiansId.clear();
    double margin = 0.1;
    double stepsize = 0.01;

    visualization_msgs::MarkerArray gaussianMarkers;

    int cnt = 0;
    for(int i=0;i<numTracks();i++){
        if(!isNewlyUpdated(i))  continue;
        PCObject object;
        object = tracks.at(i)->lastFrame().object;
        int id = tracks.at(i)->id;
        // make a gmm eval points of the object

        cnt ++;
        visualization_msgs::Marker gaussianMarker;
        gaussianMarker.header.frame_id = "/origin";
        gaussianMarker.header.stamp = ros::Time();
        gaussianMarker.ns = "gaussian";
        gaussianMarker.id = cnt;
        oldGaussiansId.push_back(cnt);

        Gaussian gaussian = object.gaussian;
        gaussianMarker.type = visualization_msgs::Marker::SPHERE;
        gaussianMarker.action = visualization_msgs::Marker::ADD;
        gaussianMarker.lifetime = ros::Duration(0);
        gaussianMarker.pose.position.x = gaussian.mean[0];
        gaussianMarker.pose.position.y = gaussian.mean[1];
        gaussianMarker.pose.position.z = gaussian.mean[2];

        Eigen::Matrix3d cov3d;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                cov3d(i,j)=gaussian.covariance(i,j);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov3d);
        Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
        Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
        Eigen::Matrix3d rotation;
        // ordering
        Eigen::Vector3d eigenvalues_ordered;
        eigenOrdering(eigenvalues, eigenvectors, eigenvalues_ordered, rotation);

        double m00 = rotation(0,0);
        double m01 = rotation(0,1);
        double m02 = rotation(0,2);
        double m10 = rotation(1,0);
        double m11 = rotation(1,1);
        double m12 = rotation(1,2);
        double m20 = rotation(2,0);
        double m21 = rotation(2,1);
        double m22 = rotation(2,2);

        // euler z-y-x sequence from orientation metrix
        double yaw = atan2(m10, m00);
        double pitch = -atan2(m20 , sqrt(m00*m00+m10*m10-m20*m20));
        double roll = atan2(sqrt(m12*m12+m02*m02), fabs(m22));
        //        if(m22<0) roll = PI-roll;

        if(m22>0 && m12<0 && m02>0) {
            if(yaw<0 && pitch>0)
                roll = -roll;
            else roll = roll;
        }
        else if (m22>0 && m12>0 && m02>0) roll = -roll;
        else if (m22<0 && m12>0 && m02<0){
            if(yaw>0 && pitch<0) roll = roll;
            else roll = -roll;
        }
        else if (m22<0 && m12<0 && m02<0) roll = -roll;


        // euler z-y-x sequence to quaternion
        double q0 = sin(yaw/2.)*sin(pitch/2.)*sin(roll/2.) + cos(yaw/2.)*cos(pitch/2.)*cos(roll/2.);
        double q1 = 0. - sin(yaw/2.)*sin(pitch/2.)*cos(roll/2.) + cos(yaw/2.)*cos(pitch/2.)*sin(roll/2.);
        double q2 = sin(yaw/2.)*cos(pitch/2.)*sin(roll/2.) + cos(yaw/2.)*sin(pitch/2.)*cos(roll/2.);
        double q3 = sin(yaw/2.)*cos(pitch/2.)*cos(roll/2.) - cos(yaw/2.)*sin(pitch/2.)*sin(roll/2.);

        gaussianMarker.pose.orientation.w = q0;
        gaussianMarker.pose.orientation.x = q1;
        gaussianMarker.pose.orientation.y = q2;
        gaussianMarker.pose.orientation.z = q3;

        gaussianMarker.frame_locked = 0;
        // confidence interval:
        // 95%: s=5.991,
        // 99%: s=9.210
        // 90%: s=4.605
//        gaussianMarker.scale.x = sqrt(eigenvalues_ordered[0]*3)*2;
//        gaussianMarker.scale.y = sqrt(eigenvalues_ordered[1]*3)*2;
//        gaussianMarker.scale.z = sqrt(eigenvalues_ordered[2]*3)*2;

        gaussianMarker.scale.x = sqrt(eigenvalues_ordered[0])*2;
        gaussianMarker.scale.y = sqrt(eigenvalues_ordered[1])*2;
        gaussianMarker.scale.z = sqrt(eigenvalues_ordered[2])*2;

//        gaussianMarker.scale.x = 1;
//        gaussianMarker.scale.y = 1;
//        gaussianMarker.scale.z = 1;

        gaussianMarker.color.a = 0.5;
        gaussianMarker.color.r = ((double)r[id])/256;
        gaussianMarker.color.g = ((double)g[id])/256;
        gaussianMarker.color.b = ((double)b[id])/256;


        gaussianMarkers.markers.push_back(gaussianMarker);
    }
    //    // delete markers
    //    for(int i=0;i<oldCnt_gaussians;i++){
    //        visualization_msgs::Marker delGauss;
    //        delGauss.header.frame_id = "/origin";
    //        delGauss.header.stamp = ros::Time();
    //        delGauss.ns = "gaussian";
    //        delGauss.id = i;
    //        delGauss.type = visualization_msgs::Marker::SPHERE;
    //        delGauss.action = visualization_msgs::Marker::DELETE;
    //        gaussianMarkers.markers.push_back(delGauss);
    //    }
    //    oldCnt_gaussians = cnt;
    return gaussianMarkers;
}

visualization_msgs::MarkerArray PCTrackContainer::oldGaussians()
{
    visualization_msgs::MarkerArray gaussianMarkers;

    // delete markers
    for(int i=0;i<oldGaussiansId.size();i++){
        visualization_msgs::Marker delGauss;
        delGauss.header.frame_id = "/origin";
        delGauss.header.stamp = ros::Time();
        delGauss.ns = "gaussian";
        delGauss.id = oldGaussiansId.at(i);
        delGauss.type = visualization_msgs::Marker::SPHERE;
        delGauss.action = visualization_msgs::Marker::DELETE;
        gaussianMarkers.markers.push_back(delGauss);
    }
    return gaussianMarkers;
}

visualization_msgs::MarkerArray PCTrackContainer::toMarkerIDs()
{
    oldTrackIDs.clear();
    visualization_msgs::MarkerArray idMarkers;

    int cnt = 0;
    for(int i=0;i<numTracks();i++){
        if(!isNewlyUpdated(i))  continue;
        Point centroid = tracks.at(i)->lastFrame().object.getCentroid();
        int id = tracks.at(i)->id;

        stringstream strm;
        string sID;
        strm << id;
        strm >> sID;

        visualization_msgs::Marker trackID;
        trackID.header.frame_id = "/origin";
        trackID.header.stamp = ros::Time();
        trackID.ns = "id";
        trackID.id = id;
        oldTrackIDs.push_back(id);
        trackID.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trackID.action = visualization_msgs::Marker::ADD;
        trackID.lifetime = ros::Duration(0);
        trackID.pose.position.x = centroid.pos[0];
        trackID.pose.position.y = centroid.pos[1];
        trackID.pose.position.z = centroid.pos[2];
        trackID.pose.orientation.x = 0.0;
        trackID.pose.orientation.y = 0.0;
        trackID.pose.orientation.z = 0.0;
        trackID.pose.orientation.w = 1.0;
        trackID.color.a = 1.0;
        trackID.color.r = 1.0;
        trackID.color.g = 1.0;
        trackID.color.b = 1.0;
        trackID.text = sID;
        trackID.scale.z = 0.02;
        //        trackID.
        idMarkers.markers.push_back(trackID);
    }
    return idMarkers;
}

visualization_msgs::MarkerArray PCTrackContainer::oldMarkerIDs()
{
    visualization_msgs::MarkerArray idMarkers;

    // delete markers
    for(int i=0;i<oldTrackIDs.size();i++){
        visualization_msgs::Marker delId;

        delId.header.frame_id = "/origin";
        delId.header.stamp = ros::Time();
        delId.ns = "id";
        delId.id = oldTrackIDs.at(i);
        delId.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        delId.action = visualization_msgs::Marker::DELETE;

        idMarkers.markers.push_back(delId);
    }
    return idMarkers;

}

void PCTrackContainer::eigenOrdering(const Eigen::Vector3d& values, const Eigen::Matrix3d& vectors, Eigen::Vector3d& values_ordered, Eigen::Matrix3d& vectors_ordered)
{
    if(values[0] > values[1] && values[0] > values[2]){
        values_ordered[0] = values[0];
        vectors_ordered(0,0) = vectors.col(0)[0];
        vectors_ordered(1,0) = vectors.col(0)[1];
        vectors_ordered(2,0) = vectors.col(0)[2];
        if(values[1] > values[2]){
            values_ordered[1] = values[1];
            vectors_ordered(0,1) = vectors.col(1)[0];
            vectors_ordered(1,1) = vectors.col(1)[1];
            vectors_ordered(2,1) = vectors.col(1)[2];

            values_ordered[2] = values[2];
            vectors_ordered(0,2) = vectors.col(2)[0];
            vectors_ordered(1,2) = vectors.col(2)[1];
            vectors_ordered(2,2) = vectors.col(2)[2];
        }
        else{
            values_ordered[1] = values[2];
            vectors_ordered(0,1) = vectors.col(2)[0];
            vectors_ordered(1,1) = vectors.col(2)[1];
            vectors_ordered(2,1) = vectors.col(2)[2];

            values_ordered[2] = values[1];
            vectors_ordered(0,2) = vectors.col(1)[0];
            vectors_ordered(1,2) = vectors.col(1)[1];
            vectors_ordered(2,2) = vectors.col(1)[2];
        }
    }
    else if(values[1] > values[0] && values[1] > values[2]){
        values_ordered[0] = values[1];
        vectors_ordered(0,0) = vectors.col(1)[0];
        vectors_ordered(1,0) = vectors.col(1)[1];
        vectors_ordered(2,0) = vectors.col(1)[2];
        if(values[0] > values[2]){
            values_ordered[1] = values[0];
            vectors_ordered(0,1) = vectors.col(0)[0];
            vectors_ordered(1,1) = vectors.col(0)[1];
            vectors_ordered(2,1) = vectors.col(0)[2];

            values_ordered[2] = values[2];
            vectors_ordered(0,2) = vectors.col(2)[0];
            vectors_ordered(1,2) = vectors.col(2)[1];
            vectors_ordered(2,2) = vectors.col(2)[2];
        }
        else{
            values_ordered[1] = values[2];
            vectors_ordered(0,1) = vectors.col(2)[0];
            vectors_ordered(1,1) = vectors.col(2)[1];
            vectors_ordered(2,1) = vectors.col(2)[2];

            values_ordered[2] = values[0];
            vectors_ordered(0,2) = vectors.col(0)[0];
            vectors_ordered(1,2) = vectors.col(0)[1];
            vectors_ordered(2,2) = vectors.col(0)[2];
        }
    }
    else if(values[2] > values[0] && values[2] > values[1]){
        values_ordered[0] = values[2];
        vectors_ordered(0,0) = vectors.col(2)[0];
        vectors_ordered(1,0) = vectors.col(2)[1];
        vectors_ordered(2,0) = vectors.col(2)[2];
        if(values[0] > values[1]){
            values_ordered[1] = values[0];
            vectors_ordered(0,1) = vectors.col(0)[0];
            vectors_ordered(1,1) = vectors.col(0)[1];
            vectors_ordered(2,1) = vectors.col(0)[2];

            values_ordered[2] = values[1];
            vectors_ordered(0,2) = vectors.col(1)[0];
            vectors_ordered(1,2) = vectors.col(1)[1];
            vectors_ordered(2,2) = vectors.col(1)[2];
        }
        else{
            values_ordered[1] = values[1];
            vectors_ordered(0,1) = vectors.col(1)[0];
            vectors_ordered(1,1) = vectors.col(1)[1];
            vectors_ordered(2,1) = vectors.col(1)[2];

            values_ordered[2] = values[0];
            vectors_ordered(0,2) = vectors.col(0)[0];
            vectors_ordered(1,2) = vectors.col(0)[1];
            vectors_ordered(2,2) = vectors.col(0)[2];
        }
    }
}


visualization_msgs::MarkerArray PCTrackContainer::toMarkerGMMs()
{

    double margin = 0.1;
    double stepsize = 0.01;

    visualization_msgs::MarkerArray gmmMarkers;

    int cnt = 0;
    for(int i=0;i<numTracks();i++){
        if(!isNewlyUpdated(i))   continue;
        PCObject object;
        object = tracks.at(i)->lastFrame().object;
        if(object.state != NOGMM){
            int id = tracks.at(i)->id;
            // make a gmm eval points of the object

            for(int j=0;j<object.gmm.size();j++){
                cnt ++;
                visualization_msgs::Marker gmmMarker;
                gmmMarker.header.frame_id = "/origin";
                gmmMarker.header.stamp = ros::Time();
                gmmMarker.ns = "gmm";
                gmmMarker.id = cnt;

                Gaussian gmm = object.gmm.at(j);
                gmmMarker.type = visualization_msgs::Marker::SPHERE;
                gmmMarker.action = visualization_msgs::Marker::ADD;
                gmmMarker.lifetime = ros::Duration(0);
                gmmMarker.pose.position.x = gmm.mean[0];
                gmmMarker.pose.position.y = gmm.mean[1];
                gmmMarker.pose.position.z = gmm.mean[2];

                Eigen::Matrix3d cov3d;
                for(int i=0;i<3;i++)
                    for(int j=0;j<3;j++)
                        cov3d(i,j)=gmm.covariance(i,j);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov3d);
                Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
                Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
                // ordering
                Eigen::Vector3d eigenvalues_ordered;
                Eigen::Matrix3d rotation;
                eigenOrdering(eigenvalues, eigenvectors, eigenvalues_ordered, rotation);

                double m00 = rotation(0,0);
                double m01 = rotation(0,1);
                double m02 = rotation(0,2);
                double m10 = rotation(1,0);
                double m11 = rotation(1,1);
                double m12 = rotation(1,2);
                double m20 = rotation(2,0);
                double m21 = rotation(2,1);
                double m22 = rotation(2,2);

                // euler z-y-x sequence from orientation metrix
                double yaw = atan2(m10, m00);
                double pitch = -atan2(m20 , sqrt(m00*m00+m10*m10-m20*m20));
                double roll = atan2(sqrt(m12*m12+m02*m02), fabs(m22));
                //        if(m22<0) roll = PI-roll;

                if(m22>0 && m12<0 && m02>0) {
                    if(yaw<0 && pitch>0)
                        roll = -roll;
                    else roll = roll;
                }
                else if (m22>0 && m12>0 && m02>0) roll = -roll;
                else if (m22<0 && m12>0 && m02<0){
                    if(yaw>0 && pitch<0) roll = roll;
                    else roll = -roll;
                }
                else if (m22<0 && m12<0 && m02<0) roll = -roll;
                //
                //                cout<<id<<" m00: "<<m00<<endl;
                //                cout<<id<<" m10: "<<m10<<endl;
                //                cout<<id<<" m20: "<<m20<<endl;
                //                cout<<id<<"eigenv1: "<<eigenvalues_ordered[0]<<endl;
                //                cout<<id<<"eigenv2: "<<eigenvalues_ordered[1]<<endl;
                //                cout<<id<<"eigenv3: "<<eigenvalues_ordered[2]<<endl;
                //                cout<<id<<" yaw: "<<yaw*180./PI<<endl;
                //                cout<<id<<" pitch: "<<pitch*180./PI<<endl;
                //                cout<<id<<" roll: "<<roll*180./PI<<endl;

                // euler z-y-x sequence to quaternion
                double q0 = sin(yaw/2.)*sin(pitch/2.)*sin(roll/2.) + cos(yaw/2.)*cos(pitch/2.)*cos(roll/2.);
                double q1 = 0. - sin(yaw/2.)*sin(pitch/2.)*cos(roll/2.) + cos(yaw/2.)*cos(pitch/2.)*sin(roll/2.);
                double q2 = sin(yaw/2.)*cos(pitch/2.)*sin(roll/2.) + cos(yaw/2.)*sin(pitch/2.)*cos(roll/2.);
                double q3 = sin(yaw/2.)*cos(pitch/2.)*cos(roll/2.) - cos(yaw/2.)*sin(pitch/2.)*sin(roll/2.);


                gmmMarker.pose.orientation.w = q0;
                gmmMarker.pose.orientation.x = q1;
                gmmMarker.pose.orientation.y = q2;
                gmmMarker.pose.orientation.z = q3;

                gmmMarker.frame_locked = 0;

                // confidence interval:
                // 95%: s=5.991,
                // 99%: s=9.210
                // 90%: s=4.605
                gmmMarker.scale.x = sqrt(2*eigenvalues_ordered[0])*2;
                gmmMarker.scale.y = sqrt(2*eigenvalues_ordered[1])*2;
                gmmMarker.scale.z = sqrt(2*eigenvalues_ordered[2])*2;


                gmmMarker.color.a = gmm.weight*object.gmm.size()/2;
                gmmMarker.color.r = ((double)r[id])/256;
                gmmMarker.color.g = ((double)g[id])/256;
                gmmMarker.color.b = ((double)b[id])/256;

                gmmMarkers.markers.push_back(gmmMarker);

            }
        }
    }

    // delete markers
    for(int i=cnt+1;i<=oldCnt;i++){
        visualization_msgs::Marker delGauss;
        delGauss.header.frame_id = "/origin";
        delGauss.header.stamp = ros::Time();
        delGauss.ns = "gmm";
        delGauss.id = i;
        delGauss.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        delGauss.action = visualization_msgs::Marker::DELETE;
        gmmMarkers.markers.push_back(delGauss);
    }
    oldCnt = cnt;
    return gmmMarkers;
}

void PCTrackContainer::evaluate()
{
    numTruePoints = 0;
    numFalsePoints = 0;
    numTotalPoints = 0;

    for(int i=0;i<numTracks();i++){
        if(tracks.at(i)->lastFrame().time == currentT){
            PCObject object;
            object = tracks.at(i)->lastFrame().object;
            int numBlack = 0;
            int numWhite = 0;
            for(int j=0;j<object.points.size();j++){
                Point point = object.points.at(j);
                double avgrgb = (point.rgb[0]*3000 + point.rgb[1]*3000 + point.rgb[2]*3000) / 3;

                if(avgrgb > 128) numWhite ++;
                else numBlack ++;
            }
            if(numWhite > numBlack){
                numTruePoints += numWhite;
                numFalsePoints += numBlack;
            }
            else{
                numTruePoints += numBlack;
                numFalsePoints += numWhite;
            }
            numTotalPoints += object.points.size();
        }
    }
}
