#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <vector>
#include <algorithm>
#include <hark_msgs/HarkSourceVal.h>
#include <hark_msgs/HarkSource.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/calib3d.hpp>
#include <thread>
#include <string>
#include "Hungarian.h"
#include "KalmanTracker.h"

using namespace std;
using namespace hark_msgs;

// Create octree used for initial background subtraction with 0.15 voxel size
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *stage1_octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.15);
// Create octree used for the second round of background subtraction
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *stage2_octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.1);

// Publisher nodes
ros::Publisher subtracted_publisher;
ros::Publisher source_publisher;

ros::Publisher orig_cloud_publisher;

// Keep track of original frame for background subtraction purposes
pcl::PointCloud<pcl::PointXYZRGB>::Ptr origFrame(new pcl::PointCloud<pcl::PointXYZRGB>());
// Rotation and translation matrix from camera to the AprilTag frame
Eigen::Matrix3d rotationMatrix;
Eigen::Vector3d translationMatrix(0, 0, 0);
Eigen::Vector3d zeroVectorTransformed;
// Constants that define how much movement is occuring in the scene
// Dictates resolution of the resulting pointcloud
int initialDownsampleRate = 2;
int secondDownsampleRate = 11;
// ID number of identified person
int personIDNum = 0;
// Respeaker will point in the +y direction in AprilTag Frame
const double respeakerX = 0;
const double respeakerY = 0;
const double respeakerZ = 0;
//Tracker constants
const double iouThreshold = 0.3;
const int max_age = 15;
const int min_hits = 3;

const std::string CAM1_NAME = "f1151000";
const std::string CAM2_NAME = "f1271477";

double processStart = 0;

struct Coord {
    double x;
    double y;
    double z;
};

struct Person {
    int id;
    Coord center;
    double bbx;
    double bby;
    double bbz;
};

std::vector<Person> arrOfPeople;
std::vector<KalmanTracker> trackers;
std::vector<Rect_<double>> predictedBoxes;
std::vector<std::vector<double>> iouMatrix;
std::vector<int> assignment;
set<int> unmatchedDetections;
set<int> unmatchedTrajectories;
set<int> allItems;
set<int> matchedItems;
vector<cv::Point> matchedPairs;
vector<Person> frameTrackingResult;
unsigned int trkNum = 0;
unsigned int detNum = 0;

// Simply publish empty pointcloud as well as empty array, used when there is no subject in scene
void publishDummy()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    empty_cloud->header.frame_id = "backgone2";
    empty_cloud->header.stamp = (long)(ros::Time::now().toSec() * 1e6);
    subtracted_publisher.publish(empty_cloud);
    HarkSource empty_source;
    empty_source.header.stamp.sec = (long)(ros::Time::now().toSec());
    empty_source.header.stamp.nsec = (ros::Time::now().toSec() - (long)(ros::Time::now().toSec())) * 1e9;
    source_publisher.publish(empty_source);
}

std::vector<pcl::PointIndices> getClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_ptr)
{
    // Search for meaningful clusters
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(diff_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.15); // 15cm, increase for speed?
    ec.setMinClusterSize(300);    // Adjust this to detect people only, depends on how camera is oriented
    ec.setMaxClusterSize(90000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(diff_cloud_ptr);
    ec.extract(cluster_indices);
    return cluster_indices;
}

double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
    float in = (bb_test & bb_gt).area();
    float un = bb_test.area() + bb_gt.area() - in;

    if (un < FLT_EPSILON)
        return 0;

    return (double)(in / un);
}

// process function that takes in a pointcloud and publishes the centroid coordinates as well as the subtracted cloud
void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr data_down, int frameNum)
{
    // Utilize a VoxelGrid filter to match dimensions of the input w the reference, 1 ms
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(data_down);
    filter.setLeafSize(0.03f, 0.03f, 0.03f); // Might be able to adjust this to a larger value since we don't need high resolution for this pointcloud
    filter.filter(*data_down);

    // Fill octree with downsampled pointcloud data
    stage1_octree->setInputCloud(data_down);
    stage1_octree->addPointsFromInputCloud();
    std::vector<int> diff_point_vector;
    stage1_octree->getPointIndicesFromNewVoxels(diff_point_vector);
    int size = diff_point_vector.size();
    // If octree freaks out and somehow deletes the reference buffer
    if (size > 0.8 * data_down->points.size())
    {
        filter.setInputCloud(origFrame);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        filter.filter(*downCloud);
        stage1_octree->deleteTree();
        // Saved the original frame for this purpose
        stage1_octree->setInputCloud(downCloud);
        stage1_octree->addPointsFromInputCloud();
        stage1_octree->switchBuffers();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        *tempCloud = *downCloud;
        stage1_octree->setInputCloud(tempCloud);
        stage1_octree->addPointsFromInputCloud();
        stage1_octree->switchBuffers();
        for (int i = 0; i < 10; i++)
        {
            std::cout << "OH NOOOOOOOOOOOOOOOOOOO" << std::endl;
        }
        return;
    }
    else
    {
        stage1_octree->deleteCurrentBuffer(); // Otherwise, delete current buffer and proceed
    }

    // In case of small point cloud, simply return
    if (diff_point_vector.size() < 300)
    {
        publishDummy();
        for (auto it = trackers.begin(); it != trackers.end();)
        {
            if (it != trackers.end() && (*it).m_time_since_update > max_age) {
                it = trackers.erase(it);     
            }
            else {
                Rect_<float> pBox = (*it).predict();
                predictedBoxes.push_back(pBox);
                it++;
            }
        }
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); // Create new pointcloud to fill with the points that differ
    // Build point cloud of differing voxels, very fast, 0.7 ms?
    for (int pc_counter = 0; pc_counter < size; pc_counter++)
    {
        diff_cloud_ptr->points.push_back(data_down->points[diff_point_vector[pc_counter]]);
    }
    
    
    //Break point cloud into clusters, ~1ms
    
    std::vector<pcl::PointIndices> cluster_indices = getClusters(diff_cloud_ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
        
    // Add all meaningful clusters to a new pointcloud object, single subject is 4.4ms
    int clusterIndex = -1;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        // Temp cloud is an intermediate containing the filtered clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::CentroidPoint<pcl::PointXYZRGB> centroid; // Used to compute centroid of pointcloud
        int totalCount = 1;
        //in LiDAR coordinate frame
        double minX;
        double minY;
        double minZ;
        double maxX;
        double maxY;
        double maxZ;
        double minXT;
        double minYT;
        double minZT;
        double maxXT;
        double maxYT;
        double maxZT;
        //in AprilTag coordinate frame
        
        bool firstTime = true;
        // Computes min/max x, y, z in both frames for a given cluster
        for (const auto &idx : it->indices)
        {
            pcl::PointXYZRGB tempPoint = (*diff_cloud_ptr)[idx];
            //Transform to AprilTag coordinate system, bc we must find minX, Y, and Z in the frame of the AprilTag!
            Eigen::Vector3d pointVector(tempPoint.x + translationMatrix(0), tempPoint.y + translationMatrix(1), tempPoint.z + translationMatrix(2));
	        pointVector = rotationMatrix * pointVector;
            if (firstTime)
            {
                minX = maxX = tempPoint.x;
                minY = maxY = tempPoint.y;
                minZ = maxZ = tempPoint.z;
                minXT = maxXT = pointVector(0);
                minYT = maxYT = pointVector(1);
                minZT = maxZT = pointVector(2);
                firstTime = false;
            }
            if (tempPoint.x < minX)
            {
                minX = tempPoint.x;
            }
            else if (tempPoint.x > maxX)
            {
                maxX = tempPoint.x;
            }
            if (tempPoint.y < minY)
            {
                minY = tempPoint.y;
            }
            else if (tempPoint.y > maxY)
            {
                maxY = tempPoint.y;
            }
            if (tempPoint.z < minZ)
            {
                minZ = tempPoint.z;
            }
            else if (tempPoint.z > maxZ)
            {
                maxZ = tempPoint.z;
            }
            if (pointVector(0) < minXT) {
                minXT = pointVector(0);
            }
            else if (pointVector(0) > maxXT) {
                maxXT = pointVector(0);
            }
            if (pointVector(1) < minYT) {
                minYT = pointVector(1);
            }
            else if (pointVector(1) > maxYT) {
                maxYT = pointVector(1);
            }
            if (pointVector(2) < minZT) {
                minZT = pointVector(2);
            }
            else if (pointVector(2) > maxZT) {
                maxZT = pointVector(2);
            }
            totalCount++;
            centroid.add(tempPoint);
        }

        double xDiff = maxX - minX;
        double yDiff = maxY - minY;
        double zDiff = maxZ - minZ;

        // Retrieve centroid
        pcl::PointXYZ orig_centroid;
        centroid.get(orig_centroid);
        // Perform filtering on original pointcloud (undownsampled) with a box filter
        pcl::CropBox<pcl::PointXYZRGB> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(orig_centroid.x - (xDiff / 2 + 0.05), orig_centroid.y - (yDiff / 2 + 0.05), orig_centroid.z - (zDiff / 2 + 0.05), 1.0));
        boxFilter.setMax(Eigen::Vector4f(orig_centroid.x + (xDiff / 2 + 0.05), orig_centroid.y + (yDiff / 2 + 0.05), orig_centroid.z + (zDiff / 2 + 0.05), 1.0));
        boxFilter.setInputCloud(data);
        boxFilter.filter(*temp_cloud);
        *person_clusters += *temp_cloud; // Add the result to a pointcloud of people clusters
        boxFilter.setInputCloud(origFrame);
        boxFilter.filter(*temp_cloud);
        *background_points += *temp_cloud; // Filter on original frame to provide background to subtract
        // Compute transform to APRILTAG frame and assign to Coord object
        Coord cvtd_centroid;
        Eigen::Vector3d camVector(orig_centroid.x + translationMatrix(0), orig_centroid.y + translationMatrix(1),
                                  orig_centroid.z + translationMatrix(2));
        camVector = rotationMatrix * camVector;
        cvtd_centroid.x = camVector(0);
        cvtd_centroid.y = camVector(1);
        cvtd_centroid.z = camVector(2);

        //All coordinates and measurements are in the AprilTag reference frame
        Person tempPerson;
        tempPerson.id = ++clusterIndex;
        tempPerson.center = cvtd_centroid;
        tempPerson.bbx = maxXT - minXT;
        tempPerson.bby = maxYT - minYT;
        tempPerson.bbz = maxZT - minZT;
        arrOfPeople.push_back(tempPerson);

    }

    // Nothing, publish empty pointcloud
    if (clusterIndex == -1)
    {
        publishDummy();
        for (auto it = trackers.begin(); it != trackers.end();)
        {
            if (it != trackers.end() && (*it).m_time_since_update > max_age) {
                it = trackers.erase(it);     
            }
            else {
                Rect_<float> pBox = (*it).predict();
                predictedBoxes.push_back(pBox);
                it++;
            }
        }
        return;
    }
    
    std::cout << "Frame number " << frameNum << " round 1 and clustering done at " << ros::Time::now().toSec() << std::endl;

    //TRACKER BEGIN 4.6e-5 performance time
    KalmanTracker::kf_count = 0;
    if (trackers.size() == 0)
    {
        //Get bounding box, add to tracker
        for (size_t i = 0; i < arrOfPeople.size(); ++i)
        {
            Person p = arrOfPeople.at(i);
            float left = p.center.x - p.bbx / 2;
            float top = p.center.y - p.bby / 2;
            float right = p.center.x + p.bbx / 2;
            float bottom = p.center.y + p.bby / 2;
            Rect_<float> box = Rect_<float>(Point_<float>(left, top), Point_<float>(right, bottom));
            //Tracker uses the max height
            KalmanTracker trk = KalmanTracker(box, p.center.z + p.bbz / 2);
            trackers.push_back(trk);
        }
        //Return out of function, no need to track because first frame
        return;
    }

    predictedBoxes.clear();
    for (auto it = trackers.begin(); it != trackers.end(); it++)
    {
        Rect_<float> pBox = (*it).predict();
        predictedBoxes.push_back(pBox);
    }

    trkNum = predictedBoxes.size();
    detNum = arrOfPeople.size();
    iouMatrix.clear();
    iouMatrix.resize(trkNum, vector<double>(detNum, 0));

    for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
    {
        for (unsigned int j = 0; j < detNum; j++)
        {
            Person p = arrOfPeople.at(j);
            float left = p.center.x - p.bbx / 2;
            float top = p.center.y - p.bby / 2;
            float right = p.center.x + p.bbx / 2;
            float bottom = p.center.y + p.bby / 2;
            iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], Rect_<float>(Point_<float>(left, top), Point_<float>(right, bottom)));
            if (isnan(iouMatrix[i][j])) {
                iouMatrix[i][j] = 1;
            }
        }
    }
    // solve the assignment problem using hungarian algorithm.
    // the resulting assignment is [track(prediction) : detection], with len=preNum
    HungarianAlgorithm HungAlgo;
    assignment.clear();
    HungAlgo.Solve(iouMatrix, assignment);
    //Clear all vectors to prepare for new information, easier than updating
    unmatchedTrajectories.clear();
    unmatchedDetections.clear();
    allItems.clear();
    matchedItems.clear();
    //Unmatched Detections
    if (detNum > trkNum)
    {
        
        for (unsigned int n = 0; n < detNum; n++)
            allItems.insert(n);

        for (unsigned int i = 0; i < trkNum; ++i)
            matchedItems.insert(assignment[i]);

        set_difference(allItems.begin(), allItems.end(),
                        matchedItems.begin(), matchedItems.end(),
                        insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
    }
    //Ppl be gone
    else if (detNum < trkNum) // there are unmatched trajectory/predictions
    {
        for (unsigned int i = 0; i < trkNum; ++i)
            if (assignment[i] == -1) { // unassigned label will be set as -1 in the assignment algorithm
                unmatchedTrajectories.insert(i);
            }
    }
    //Find matched pairs, and unmatched bounding boxes based on IOU
    matchedPairs.clear();
    for (unsigned int i = 0; i < trkNum; ++i)
    {
        if (assignment[i] == -1) // pass over invalid values
            continue;
        if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
        {
            unmatchedTrajectories.insert(i);
            unmatchedDetections.insert(assignment[i]);
        }
        else
            matchedPairs.push_back(cv::Point(i, assignment[i]));
    }
    
    

    //Iterate through the matched trackers and update them with the new information
    int detIdx, trkIdx;
    for (unsigned int i = 0; i < matchedPairs.size(); i++)
    {
        trkIdx = matchedPairs[i].x;
        detIdx = matchedPairs[i].y;
        Person p = arrOfPeople.at(detIdx);
        float left = p.center.x - p.bbx / 2;
        float top = p.center.y - p.bby / 2;
        float right = p.center.x + p.bbx / 2;
        float bottom = p.center.y + p.bby / 2;
        Rect_<float> box(Point_<float>(left, top), Point_<float>(right, bottom));
        trackers[trkIdx].update(box, p.center.z + p.bbz / 2);
    }

    // create and initialise new trackers for unmatched detections
    for (auto umd : unmatchedDetections)
    {
        Person p = arrOfPeople.at(umd);
        float left = p.center.x - p.bbx / 2;
        float top = p.center.y - p.bby / 2;
        float right = p.center.x + p.bbx / 2;
        float bottom = p.center.y + p.bby / 2;
        Rect_<float> box = Rect_<float>(Point_<float>(left, top), Point_<float>(right, bottom));
        KalmanTracker tracker = KalmanTracker(box, p.center.z + p.bbz / 2);
        trackers.push_back(tracker);

    }

    frameTrackingResult.clear();
    //Places all of the live trackers into an array, removing all the dead ones
    for (auto it = trackers.begin(); it != trackers.end();)
    {
        //Hit streak must be greater than min_hits
        if (((*it).m_time_since_update < max_age) &&
				((*it).m_hit_streak >= min_hits))
        {
            Person p;
            Rect_<float> box = (*it).get_state();
            Coord center;
            
            center.x = (double) (box.x + 0.5 * box.width);
            center.y = (double) (box.y + 0.5 * box.height);
            center.z = (*it).m_height;
            p.center = center;
            p.id = (*it).m_tracker_id;
            frameTrackingResult.push_back(p);
            it++;
        }
        // remove dead trackers that haven't been updated in 5 frames
        else if (it != trackers.end() && (*it).m_time_since_update > max_age) {
            it = trackers.erase(it);         
        }
        else {
            it++;
        }
    }
    
    std::cout << "Frame number " << frameNum << " tracking done at " << ros::Time::now().toSec() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_clusters_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // Create new pointcloud to fill with the points that differ
    //Timestamp the ros messages, send out the sources first to have more time in audio processing pipeline
    person_clusters_filtered->header.stamp = (long)(ros::Time::now().toSec() * 1e6);
    HarkSource sources;
    sources.header.stamp.sec = (long)(ros::Time::now().toSec());
    sources.header.stamp.nsec = (ros::Time::now().toSec() - (long)(ros::Time::now().toSec())) * 1e9;
    person_clusters_filtered->header.frame_id = "backgone2";

    //NOTE SOURCES MUST ALREADY BE IN RESPEAKER FRAME!
    //Iterate through frameTrackingResult and place in Hark Compatible Message for publishing
    for (int i = 0; i < frameTrackingResult.size(); i++)
    {
        HarkSourceVal s;
        s.id = frameTrackingResult.at(i).id;
        // Check to see if this is respeaker or camera frame
        s.x = frameTrackingResult.at(i).center.x - respeakerX;
        s.y = frameTrackingResult.at(i).center.y - respeakerY;
        s.z = frameTrackingResult.at(i).center.z - respeakerZ;
        double xDiff = s.x - respeakerX;
        double yDiff = s.y - respeakerY;
        //Respeaker orientation for angle is OPPOSITE apriltag
        s.azimuth = 180 * atan2(yDiff, xDiff) / 3.14159;
        s.elevation = s.z - 0.12; 
        s.power = 0;
        sources.src.push_back(s);
    }
    if (sources.src.size() == 1)  {
        //Add Direction to Account for Noise
        HarkSourceVal constantS;
        constantS.id = 100;
        constantS.x = 0.5;
        constantS.y = 0;
        constantS.z = 0;
        constantS.azimuth = 360 - sources.src.at(0).azimuth;
        //constantS.azimuth  = 180;
        constantS.elevation = 0;
        constantS.power = 0;
        sources.src.push_back(constantS);
    }

    sources.count = sources.src.size();
    sources.exist_src_num = sources.count;

    source_publisher.publish(sources);

    // Round 2 background subtraction
    double performanceTime = ros::Time::now().toSec();

    stage2_octree->setInputCloud(background_points);
    stage2_octree->addPointsFromInputCloud();
    stage2_octree->switchBuffers();
    stage2_octree->setInputCloud(person_clusters);
    stage2_octree->addPointsFromInputCloud();
    std::vector<int> different_points;
    stage2_octree->getPointIndicesFromNewVoxels(different_points); // Extract points that differ
    int size2 = different_points.size();                           // Store in size variable to avoid calling .size() in loop
    stage2_octree->deleteTree();



    
    // Construct new pointcloud, BUT WITH TRANSFORMED TO APRILTAG COORDINATE FRAME!
    for (int pc_counter = 0; pc_counter < size2; pc_counter++)
    {
        pcl::PointXYZRGB tempPoint = person_clusters->points[different_points[pc_counter]];
        Eigen::Vector3d coordVec(tempPoint.x + translationMatrix(0), tempPoint.y + translationMatrix(1), tempPoint.z + translationMatrix(2));
        coordVec = rotationMatrix * coordVec;
        tempPoint.x = coordVec(0);
        tempPoint.y = coordVec(1);
        tempPoint.z = coordVec(2);
        person_clusters_filtered->points.push_back(tempPoint);
    }

    subtracted_publisher.publish(person_clusters_filtered);
    
    arrOfPeople.clear();
    std::cout << "Frame number " << frameNum << " outputted at " << ros::Time::now().toSec() << "with size " << person_clusters_filtered->points.size()<< std::endl;

    
}

// Initializes the camera by setting reference frame
void initializeCamera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &data)
{
    *origFrame = *data;
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(data);
    filter.setLeafSize(0.03f, 0.03f, 0.03f); // Might be able to adjust this to a larger value since we don't need high resolution for this pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledData(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter.filter(*downsampledData);
    // THIS CODE IS WEIRD BUT IT FIXES AN ISSUE
    stage1_octree->deleteCurrentBuffer();
    stage1_octree->setInputCloud(downsampledData);
    stage1_octree->addPointsFromInputCloud();
    stage1_octree->switchBuffers();
    stage1_octree->setInputCloud(downsampledData);
    stage1_octree->addPointsFromInputCloud();
    stage1_octree->switchBuffers();
    std::cout << "Initializing" << std::endl;
}

// Utilize aruco library to transform to AprilTag reference
void getTransform(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat image)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    cv::Mat imageCopy;
    rotationMatrix << 1, 0, 0,
        0, 0, 1,
        0, -1, 0;

    for (int i = 0; i < ids.size(); i++) {
        std::cout << "Detected marker: " << ids.at(i) << std::endl;
    }
    if (ids.size() > 0)
    {
        image.copyTo(imageCopy);
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.17145, cameraMatrix, distCoeffs, rvecs, tvecs);
        cv::Mat rotationArr;
        cv::Rodrigues(rvecs.at(0), rotationArr);
        Eigen::Matrix4d tempMatrix;
        rotationMatrix << rotationArr.at<double>(0, 0), rotationArr.at<double>(1, 0), rotationArr.at<double>(2, 0),
            rotationArr.at<double>(0, 1), rotationArr.at<double>(1, 1), rotationArr.at<double>(2, 1),
            rotationArr.at<double>(0, 2), rotationArr.at<double>(1, 2), rotationArr.at<double>(2, 2);
        translationMatrix = Eigen::Vector3d(-tvecs.at(0)[0], -tvecs.at(0)[1], -tvecs.at(0)[2]);
        zeroVectorTransformed = rotationMatrix * translationMatrix;
        std::cout << zeroVectorTransformed << std::endl;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera2");
    ros::NodeHandle nh;
    rs2::pointcloud pc;
    rs2::config cfg;
    rs2::context ctx;
    orig_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera2/original", 2);
    subtracted_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera2/clouds/subtracted", 2);
    source_publisher = nh.advertise<HarkSource>("/camera2/sources", 2);
    // Only enable camera and depth, other streams will likely cause more latency
    rs2::points points;
    
    for (auto&& dev : ctx.query_devices())
    {
        std::cout << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }


    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
    // cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    // cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    string currentCam = CAM2_NAME;
    std::cout << "About to enable " << std::endl;
    cfg.enable_device(currentCam);
    
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);
    // Obtain camera constants to use with AprilTag
    auto const color_intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    float fx = color_intrinsics.fx;
    float fy = color_intrinsics.fy;
    float ppx = color_intrinsics.ppx;
    float ppy = color_intrinsics.ppy;
    float camParam[3][3] = {{fx, 0, ppx},
                            {0, fy, ppy},
                            {0, 0, 1}};
    cv::Mat temp(1, 1, CV_32FC1);
    cv::Mat camMatrix(3, 3, CV_32FC1, camParam);
    float tempIntrinsics[5];
    for (int i = 0; i < 5; i++)
    {
        tempIntrinsics[i] = color_intrinsics.coeffs[i];
    }
    cv::Mat distortionMatrix(1, 5, CV_32FC1, tempIntrinsics);
    int currNumFrames = 0;
    bool initializing = true;
    std::cout << std::setprecision(4) << std::fixed;
    //Frame number for timing analysis
    int frameNum = 0;
    while (ros::ok())
    { // ros::ok()??
        // Get relevant frame data
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        std::cout << "Frame number " << ++frameNum << " received at " << ros::Time::now().toSec() << std::endl;
        // map pointcloud to color? (I don't really know what this does)
        pc.map_to(color);
        int colorHeight = color.get_height();
        int colorWidth = color.get_width();
        int colorBytes = color.get_bytes_per_pixel();
        int colorStride = color.get_stride_in_bytes();
        
        // Iterate through the image and add to the ros message
        sensor_msgs::Image color_img;
        uint8_t *colorArr = (uint8_t *)(color.get_data());
        std::vector<uint8_t> vec;
        if (currNumFrames < 51) {
            for (int i = 0; i < colorHeight; i++)
            {
                int i_scaled = i * colorWidth;
                for (int j = 0; j < colorWidth; j++)
                {
                    int baseIndex = 3 * (i_scaled + j);
                    color_img.data.push_back(colorArr[baseIndex]);
                    color_img.data.push_back(colorArr[baseIndex + 1]);
                    color_img.data.push_back(colorArr[baseIndex + 2]);
                }
            }
        }
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth); //9-10 ms
        auto Texture_Coord = points.get_texture_coordinates();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_pointsDown(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto Vertex = points.get_vertices();
        int pointSize = points.size();
        pcl_points->points.reserve(pointSize);
        pcl_pointsDown->points.reserve(pointSize / 21);
        // Build the pointcloud
        //Loop time runs in 5 milliseconds for 640*480
        for (int i = 0; i < pointSize; i++)
        {
            pcl::PointXYZRGB temp;
            auto tempVertexElem = Vertex[i];
            temp.x = tempVertexElem.x;
            temp.y = tempVertexElem.y;
            temp.z = tempVertexElem.z;
            int x_value = min(max(int(Texture_Coord[i].u * colorWidth + .5f), 0), colorWidth - 1);
            int y_value = min(max(int(Texture_Coord[i].v * colorHeight + .5f), 0), colorHeight - 1);
            int Text_Index = x_value * colorBytes + y_value * colorStride;
            // RGB components to save in tuple
            temp.r = colorArr[Text_Index];
            temp.g = colorArr[Text_Index + 1];
            temp.b = colorArr[Text_Index + 2];
            pcl_points->points.push_back(temp);
            if (i % 21 == 0) {
                pcl_pointsDown->points.push_back(temp);
            }
        }
        std::cout << "Frame number " << frameNum << " point cloud generated at " << ros::Time::now().toSec() << std::endl;

        pcl_points->header.frame_id = "PC2";
        orig_cloud_publisher.publish(pcl_points);

        color_img.header.frame_id = "color_img2";
        color_img.width = colorWidth;
        color_img.height = colorHeight;
        color_img.encoding = "rgb8";
        color_img.is_bigendian = true;
        

        // CYCLE THE BUFFERS OF THE OCTREE, I HAVE NO IDEA WHY THIS FIXES IT
        if (initializing && currNumFrames++ <= 50)
        {
            if (currNumFrames == 20)
            {
                getTransform(camMatrix, distortionMatrix, cv::Mat(colorHeight, colorWidth, CV_8UC3, colorArr));
                pipe.stop();
                cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_RGB8, 30);
                cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
                pipe.start(cfg);
                continue;
            }
            initializeCamera(pcl_points);
            continue;
        }
        if (initializing)
        {
            initializeCamera(pcl_points);
            initializing = false;
            std::cout << "Done" << std::endl;
        }
        else
        {
            //13 ms
            processStart = ros::Time::now().toSec();
            std::thread processCloud(process, pcl_points, pcl_pointsDown, frameNum);
            processCloud.detach();
        }
        //std::cout << ros::Time::now().toSec() - startTime << std::endl;
    }
}
