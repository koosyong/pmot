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
#ifndef PCTRACKCONTAINER_H
#define PCTRACKCONTAINER_H

#define PI 3.141592
// pcl
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>


//#include "pctrack.h"
#include "pcobject.h"
#include "trackcontainer.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class PCTrackContainer : public TrackContainer<PCObject>
{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

public:
    PCTrackContainer();
    PCTrackContainer(int _maxFrame = 1000);

public:
    void toPointCloudXYZI(Cloud &cloudOut);
    void toPointCloudXYZI_model(Cloud &cloudOut);
    visualization_msgs::MarkerArray toMarkerGaussians();
    visualization_msgs::MarkerArray toMarkerGMMs();
    visualization_msgs::MarkerArray oldGaussians();
    visualization_msgs::MarkerArray toMarkerIDs();
    visualization_msgs::MarkerArray oldMarkerIDs();
    visualization_msgs::Marker toMarkerEdges();

    void evaluate();
    void iros2014();

public:
    int numTruePoints;
    int numFalsePoints;
    int numTotalPoints;
    bool isUpdated;
private:
    inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
    inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}


    void eigenOrdering(const Eigen::Vector3d& values, const Eigen::Matrix3d& vectors, Eigen::Vector3d& values_ordered, Eigen::Matrix3d& vectors_ordered);
private:
    int maxFrame;
    vector<int> oldGaussiansId;
    vector<int> oldTrackIDs;

};

#endif // PCTRACKCONTAINER_H
