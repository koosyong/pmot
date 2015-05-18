#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GridCells.h>

// PCTracking
#include "pctracking.h"

string param_frame_id;
string param_sub_topic;
string param_pub_points_trackID;
string param_pub_markers_trackID, param_pub_markers_gaussians, param_pub_markers_gmms, param_pub_markers_edges;
string param_pub_points_filtered;
string param_pub_points_segmented;
string param_pub_points_objectmodel;
string param_pub_gmmreg_model, param_pub_gmmreg_scene;
string param_workspace_topic;
int param_3d6d;
double param_samplingRatio, param_simplifyRatio, param_segmentTolerance;
double param_filtering_range;
double param_workspace_x;
double param_workspace_y;
double param_workspace_z;
double param_workspace_width;
double param_workspace_height;
double param_workspace_zheight;
int param_buffernum ;

ros::Publisher pub_filtered;
ros::Publisher pub_segmented;
ros::Publisher pub_track;
ros::Publisher pub_trackID;
ros::Publisher pub_objectmodel;
ros::Publisher pub_model;
ros::Publisher pub_scene;
ros::Publisher pub_gaussians;
ros::Publisher pub_gmms;
ros::Publisher pub_edges;
ros::Publisher pub_workspace;
ros::Publisher pub_lastmodel;
ros::Publisher pub_particles;

tf::TransformListener *tf_listener;
nav_msgs::GridCells workspace;


double cont_sampling;
double cont_simplify;

int  sumTotalPoint = 0;
int sumTruePoint  = 0;
int sumErrorPoint  = 0;
double computationT = 0;
bool isdebug = 1;

bool readParameters();
void transform(string frame);
void cuttingRange(double range);
void downsampling(double scale);
void planeExtraction(int nPlane);
void workingspace();

PCTracking *pctracking;
CloudPtr pCloud_input, pCloud;
double lastT, nowT;

void cb_rgb(const pcl::PCLPointCloud2ConstPtr& input)
{
    static int cnt = 0;

    // input
    pCloud_input.reset(new Cloud);
    pCloud.reset(new Cloud);
    pcl::fromPCLPointCloud2(*input, *pCloud_input);
    if(isdebug) cout<<"I heard RGB, # of points: "<<pCloud_input->points.size()<<endl;

    // downsampling
    lastT = pcl::getTime ();
    downsampling(cont_sampling);
    if(isdebug) cout<<"downsampling computation time : "<<pcl::getTime()-lastT<<endl;
    if(isdebug) cout<<"downsampling end, # of points: "<<pCloud_input->points.size()<<endl;

    // transform to the given frame
    lastT = pcl::getTime ();
    transform(param_frame_id);
    if(isdebug) cout<<"transform computation time : "<<pcl::getTime()-lastT<<endl;

    // workspace
    lastT = pcl::getTime ();
    workingspace();
    if(isdebug) cout<<"workingspace computation time : "<<pcl::getTime()-lastT<<endl;
    if(isdebug) cout<<"workingspace end, # of points: "<<pCloud_input->points.size()<<endl;

//    // planeExtraction
//    lastT = pcl::getTime ();
//    planeExtraction(1);
//    if(isdebug) cout<<"planeExtraction computation time : "<<pcl::getTime()-lastT<<endl;
//    if(isdebug) cout<<"planeExtraction end, # of points: "<<pCloud_input->points.size()<<endl;

    // tracking
    lastT = pcl::getTime ();
    pctracking->run(pCloud_input);
    nowT = pcl::getTime ();
    if(isdebug) cout<<"total tracking computation time : "<<nowT-lastT<<endl;

    // publish filtered pointcloud
    pcl::PCLPointCloud2 output_filtered;
    pcl::toPCLPointCloud2(*pctracking->getFilteredPC(), output_filtered);
    output_filtered.header.frame_id=param_frame_id.data();    
    pub_filtered.publish (output_filtered);

    // publish segmented pointcloud
    pcl::PCLPointCloud2 output_segmented;
    pcl::toPCLPointCloud2(*pctracking->getSegmentedPC(), output_segmented);
    output_segmented.header.frame_id=param_frame_id.data();
    pub_segmented.publish (output_segmented);

    // publish model pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudOut_model;
    pCloudOut_model.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pctracking->object_tracks->toPointCloudXYZI_model(*pCloudOut_model);
    pcl::PCLPointCloud2 output_model;
    pcl::toPCLPointCloud2(*pCloudOut_model, output_model);
    output_model.header.frame_id=param_frame_id.data();
    pub_objectmodel.publish (output_model);

    // output tracks pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudOut;
    pCloudOut.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pctracking->object_tracks->toPointCloudXYZI(*pCloudOut);
    //    ROS_INFO("# of track points: %d", pCloudOut->points.size());

    // publish pointcloud(currentT) of tracks
    pcl::PCLPointCloud2 output;
    pcl::toPCLPointCloud2(*pCloudOut, output);
    output.header.frame_id=param_frame_id.data();
    pub_track.publish (output);


    //    // publish lastmodel
    //    pcl::PCLPointCloud2 output_lastmodel;
    //    pcl::toPCLPointCloud2(*pctracking->getLastModel(), output_lastmodel);
    //    output_lastmodel.header.frame_id=param_frame_id.data();
    //    pub_lastmodel.publish (output_lastmodel);

    //    // publish particles
    //    pcl::PCLPointCloud2 output_particles;
    //    pcl::toPCLPointCloud2(*pctracking->getParticles(), output_particles);
    //    output_particles.header.frame_id=param_frame_id.data();
    //    pub_particles.publish (output_particles);


    // publish gaussians
    pub_gaussians.publish(pctracking->object_tracks->oldGaussians());
    pub_gaussians.publish(pctracking->object_tracks->toMarkerGaussians());

    // publish gmms
//    pub_gmms.publish(pctracking->object_tracks->toMarkerGMMs());
//    pub_edges.publish(pctracking->object_tracks->toMarkerEdges());

    // publish delete id marker of deleted tracks
    pub_trackID.publish(pctracking->object_tracks->oldMarkerIDs());
    pub_trackID.publish(pctracking->object_tracks->toMarkerIDs());

    pCloudOut.reset();
    cnt++;

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "dhri_multipleObjectTracking");
    ros::NodeHandle n;

    // read parameters
    if(!readParameters())   return 0;


    tf_listener = new tf::TransformListener(ros::Duration(5000));


    // publishers
    pub_filtered = n.advertise<pcl::PCLPointCloud2>(param_pub_points_filtered.data(), 1000);
    pub_segmented = n.advertise<pcl::PCLPointCloud2>(param_pub_points_segmented.data(), 1000);
    pub_track = n.advertise<pcl::PCLPointCloud2>(param_pub_points_trackID.data(), 1000);
    pub_objectmodel = n.advertise<pcl::PCLPointCloud2>(param_pub_points_objectmodel.data(), 1000);
    pub_trackID = n.advertise<visualization_msgs::MarkerArray>(param_pub_markers_trackID.data(), 1000);
    pub_gaussians = n.advertise<visualization_msgs::MarkerArray>(param_pub_markers_gaussians.data(), 1000);
    pub_workspace = n.advertise<nav_msgs::GridCells> (param_workspace_topic, 1000);
    pub_lastmodel = n.advertise<sensor_msgs::PointCloud2>("models", 1000);
    pub_particles = n.advertise<sensor_msgs::PointCloud2>("particles", 1000);

    pctracking = new PCTracking(isdebug, param_frame_id, &pub_scene, &pub_model, 3, cont_sampling, cont_simplify, param_segmentTolerance);

    tf_listener->waitForTransform(param_frame_id.data(), "/camera_rgb_frame", ros::Time::now(), ros::Duration(5.0));
    ros::Subscriber sub_id = n.subscribe(param_sub_topic.data(), param_buffernum, cb_rgb);

    ros::spin();
    delete pctracking;
    return 0;
}


bool readParameters()
{

    bool isOK = 1;
    if(ros::param::has("/dhri/multipleObjectTracking/sub/topic"))
        ros::param::get("/dhri/multipleObjectTracking/sub/topic", param_sub_topic);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/frame/id"))
        ros::param::get("/dhri/multipleObjectTracking/frame/id", param_frame_id);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/points/trackID"))
        ros::param::get("/dhri/multipleObjectTracking/pub/points/trackID", param_pub_points_trackID);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/markers/trackID"))
        ros::param::get("/dhri/multipleObjectTracking/pub/markers/trackID", param_pub_markers_trackID);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/points/filtered"))
        ros::param::get("/dhri/multipleObjectTracking/pub/points/filtered", param_pub_points_filtered);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/points/segmented"))
        ros::param::get("/dhri/multipleObjectTracking/pub/points/segmented", param_pub_points_segmented);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/points/objectmodel"))
        ros::param::get("/dhri/multipleObjectTracking/pub/points/objectmodel", param_pub_points_objectmodel);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/pub/markers/gaussians"))
        ros::param::get("/dhri/multipleObjectTracking/pub/markers/gaussians", param_pub_markers_gaussians);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/samplingRatio"))
        ros::param::get("/dhri/multipleObjectTracking/param/samplingRatio", param_samplingRatio);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/segmentTolerance"))
        ros::param::get("/dhri/multipleObjectTracking/param/segmentTolerance", param_segmentTolerance);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/param/buffernum"))
        ros::param::get("/dhri/multipleObjectTracking/param/buffernum", param_buffernum);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/topic"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/topic", param_workspace_topic);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/x"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/x", param_workspace_x);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/y"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/y", param_workspace_y);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/z"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/z", param_workspace_z);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/width"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/width", param_workspace_width);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/height"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/height", param_workspace_height);
    else isOK = 0;
    if(ros::param::has("/dhri/multipleObjectTracking/workspace/zheight"))
        ros::param::get("/dhri/multipleObjectTracking/workspace/zheight", param_workspace_zheight);
    else isOK = 0;

    if(isOK){
    }
    else{
        ROS_WARN("Run 'rosparam load param.yaml' first");
        return 0;
    }

    cont_sampling = param_samplingRatio / 1000.;
    cont_simplify = param_simplifyRatio / 100.;

    workspace.cell_width = param_workspace_width;
    workspace.cell_height = param_workspace_height;
    geometry_msgs::Point point;
    point.x = param_workspace_x;
    point.y = param_workspace_y;
    point.z = param_workspace_z;
    workspace.cells.push_back(point);
}

void workingspace()
{
    double top = param_workspace_y + param_workspace_height/2;
    double bottom = param_workspace_y - param_workspace_height/2;
    double left = param_workspace_x - param_workspace_width/2;
    double right = param_workspace_x + param_workspace_width/2;
    double zbottom = param_workspace_z;
    double ztop = param_workspace_z + param_workspace_zheight;

    for(int i=0;i<pCloud_input->points.size();i++){
        PointT temp_point = pCloud_input->points[i];
        // cut off
        if(temp_point.x<right && temp_point.x>left && temp_point.y<top && temp_point.y>bottom && temp_point.z>zbottom && temp_point.z<ztop)
                pCloud->points.push_back(temp_point);
    }
    pCloud_input.reset(new Cloud);
    pCloud_input = pCloud;
    pCloud.reset(new Cloud);

    workspace.header.frame_id = param_frame_id.data();
    workspace.header.stamp = ros::Time::now();
    pub_workspace.publish (workspace);
}

void transform(string frame)
{    
    tf_listener->waitForTransform(frame.data(), pCloud_input->header.frame_id, ros::Time::now(), ros::Duration(5.0));
    pcl_ros::transformPointCloud(frame.data(), *(pCloud_input), *pCloud, *tf_listener);

    pCloud_input.reset(new Cloud);
    pCloud_input = pCloud;
    pCloud.reset(new Cloud);
}

void cuttingRange(double range)
{
    for(int i=0;i<pCloud_input->points.size();i++){
        double dist = sqrt(pow(pCloud_input->points.at(i).x,2)+pow(pCloud_input->points.at(i).y,2)+pow(pCloud_input->points.at(i).z,2));
        if(dist<range){
            pCloud->points.push_back(pCloud_input->points.at(i));
        }
    }
    pCloud_input.reset(new Cloud);
    pCloud_input = pCloud;
    pCloud.reset(new Cloud);
}

void downsampling(double scale)
{
    CloudConstPtr pCloudConst = (CloudConstPtr)pCloud_input;

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(pCloudConst);
    vg.setLeafSize(scale, scale, scale); // down sampling using a leaf size of 'scale'
    vg.filter(*pCloud);

    pCloud_input.reset(new Cloud);
    pCloud_input = pCloud;
    pCloud.reset(new Cloud);
}

void planeExtraction(int nPlane)
{
    CloudConstPtr pCloudConst = (CloudConstPtr)pCloud_input;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    // plane segmentation
    for(int i=0;i<nPlane;i++)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(pCloud_input);
        seg.segment (*inliers, *coefficients); //*
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (pCloudConst);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        CloudPtr cloud_plane;
        cloud_plane.reset(new Cloud);
        extract.filter (*cloud_plane); //*
//        clouds_plane.push_back(cloud_plane);
        //      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*pCloud); //*
    }

    pCloud_input.reset(new Cloud);
    pCloud_input = pCloud;
    pCloud.reset(new Cloud);
}

