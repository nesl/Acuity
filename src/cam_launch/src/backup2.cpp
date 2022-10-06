#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <vector>
#include <algorithm>
#include <NESLMessages/NeslCoord.h>
#include <NESLMessages/Person.h>
#include <NESLMessages/PersonArr.h>
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
using namespace NESLMessages;
using namespace std;

//TODO: use ros::Time::now() to sync timestamps
//Create octree with resolution 0.15 sized voxels
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.15);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *previousTree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.15);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree2 = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.1);
//Used to keep track of when to swap in new background to reduce drift
double lastUpdate;
//Publisher nodes
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher cloud_publisher;
ros::Publisher color_publisher;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr origFrame(new pcl::PointCloud<pcl::PointXYZRGB>());
PersonArr arrOfPeople;
Eigen::Matrix3d rotationMatrix;
Eigen::Vector3d translationMatrix(0, 0, 0);
const double DIFFERING_PROPORTION_UPPER = 0.1;
const double DIFFERING_PROPORTION_LOWER = 0.05;
int initialDownsampleRate = 2;
int secondDownsampleRate = 11;

//Get absolute difference between nesl coords
double computeAbsDiff(NeslCoord coord1, NeslCoord coord2) {
    double diff = (coord1.x - coord2.x) * (coord1.x - coord2.x) +
                  (coord1.z- coord2.z) * (coord1.z - coord2.z);
    return sqrt(diff);
}

//Probability looks at all people inside the array and expresses how likely for two people to be confused
//If probability is sufficiently low (there are no individuals close to one another), threshold for person matching between 
//frames is lower
bool evaluateDifference(Person person, NeslCoord centroid, double red, double green, double blue, double probability) {
    double centroidDiff = computeAbsDiff(centroid, person.personCoord);
    if (probability < 0.2) {
        return centroidDiff < 0.1;
    }
    double redDiff2 = (red - person.colorArr[0]) * (red - person.colorArr[0]);
    double greenDiff2 = (green - person.colorArr[1]) * (green - person.colorArr[1]);
    double blueDiff2 = (blue - person.colorArr[2]) * (blue - person.colorArr[2]);
    double colorDiff2 = redDiff2 + greenDiff2 + blueDiff2;
    //If people are close together, squared color difference should be sufficiently small
    return colorDiff2 < 300 && centroidDiff < 0.05;
}

int personIDNum = 0;
long frameCount = 900; //what is this??
//process function that takes in a pointcloud and publishes the centroid coordinates as well as the subtracted cloud
void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr data_down(new pcl::PointCloud<pcl::PointXYZRGB>());
    double octreeStartTime = ros::Time::now().toSec();
    for (int i = 0; i < data->points.size(); i += 11) {
        data_down->push_back(data->points[i]);
    }
    //Fill octree with pointcloud data
    octree->setInputCloud(data_down);
    octree->addPointsFromInputCloud();
    std::vector<int> diff_point_vector;
    octree->getPointIndicesFromNewVoxels(diff_point_vector); //Extract points that differ
    int size = diff_point_vector.size();                     //Store in size variable to avoid calling .size() in loop
    //If octree freaks out and somehow deletes the reference buffer
    if (size > 0.8 * data_down->points.size()) {
        octree->deleteTree();
        octree->setInputCloud(origFrame);
        octree->addPointsFromInputCloud();
        octree->switchBuffers();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        *tempCloud = *data_down;
        octree->setInputCloud(tempCloud);
        octree->addPointsFromInputCloud();
        octree->switchBuffers();
        std::cout << "OH NOOOOOOOOOOOOOOOOOOO" << std::endl;
        return;
    }
    else {
        octree->deleteCurrentBuffer(); //Otherwise, delete current buffer and proceed
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //Create new pointcloud to fill with the points that differ

    //Parallelizing doesn't seem to result in significant performance gain
    for (int pc_counter = 0; pc_counter < size; pc_counter++)
    {
        diff_cloud_ptr->points.push_back(data_down->points[diff_point_vector[pc_counter]]);
    }

    //In case of small point cloud, simply return
    if (diff_point_vector.size() < 200)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        temp_cloud->header.frame_id = "backgone3";
        temp_cloud->header.stamp = (long) (ros::Time::now().toSec() * 1e6);
        pub.publish(temp_cloud);
        arrOfPeople.personArr.clear();
        arrOfPeople.header.stamp.sec = (long) (ros::Time::now().toSec());
        arrOfPeople.header.stamp.nsec = (ros::Time::now().toSec() - (long) (ros::Time::now().toSec())) * 1e9;
        pub2.publish(arrOfPeople);
        return;
    }
    
    //Utilize a VoxelGrid filter to downsample pointcloud, makes later data processing much faster

    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(diff_cloud_ptr);
    filter.setLeafSize(0.027, 0.027f, 0.027f); //Might be able to adjust this to a larger value since we don't need high resolution for this pointcloud
    filter.filter(*diff_cloud_ptr);

    //PCL segmentation functions that help extract meaningful features
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.08); //How close it must be to be an inlier, might change this to a smaller value?
    seg.setInputCloud(diff_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    //publish empty pointcloud, no significant inlierse
    if (inliers->indices.size() == 0)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        temp_cloud->header.frame_id = "backgone3";
        temp_cloud->header.stamp = (long) (ros::Time::now().toSec() * 1e6);
        pub.publish(temp_cloud);
        arrOfPeople.personArr.clear();
        arrOfPeople.header.stamp.sec = (long) (ros::Time::now().toSec());
        arrOfPeople.header.stamp.nsec = (ros::Time::now().toSec() - (long) (ros::Time::now().toSec())) * 1e9;
        pub2.publish(arrOfPeople);
        return;
    }
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(diff_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);

    //Search for meaningful clusters
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(diff_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.1); // 10cm
    ec.setMinClusterSize(200);   //Adjust this to detect people only, depends on how camera is oriented
    ec.setMaxClusterSize(70000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(diff_cloud_ptr);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    //Add all meaningful clusters to the pointcloud object
    int loopCount3 = 0;
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        loopCount3++;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::CentroidPoint<pcl::PointXYZRGB> centroid; //Used to compute centroid of pointcloud
        double redAvg = 0;
        double blueAvg = 0;
        double greenAvg = 0;
        int totalCount = 1;
        double minX;
        double minY;
        double minZ;
        double maxX;
        double maxY;
        double maxZ;
        
        bool firstTime = true;
        //Computes min/max x, y, z and color averages
        for (const auto &idx : it->indices)
        {
            pcl::PointXYZRGB tempPoint = (*diff_cloud_ptr)[idx];
            if (firstTime) {
                minX = maxX = tempPoint.x;
                minY = maxY = tempPoint.y;
                minZ = maxZ = tempPoint.z;
                firstTime = false;
            }
            if (tempPoint.x < minX) {
                minX = tempPoint.x;
            }
            else if(tempPoint.x > maxX) {
                maxX = tempPoint.x;
            }
            if (tempPoint.y < minY) {
                minY = tempPoint.y;
            }
            else if(tempPoint.y > maxY) {
                maxY = tempPoint.y;
            }
            if (tempPoint.z < minZ) {
                minZ = tempPoint.z;
            }
            else if(tempPoint.z > maxZ) {
                maxZ = tempPoint.z;
            }
            redAvg += tempPoint.r;
            greenAvg += tempPoint.g;
            blueAvg += tempPoint.b;
            totalCount++;
            centroid.add(tempPoint);
        }
        redAvg /= totalCount;
        blueAvg /= totalCount;
        greenAvg /= totalCount;
        //Add to NeslCoord object to send as custom ros message
        pcl::PointXYZ c1;
        centroid.get(c1);
        pcl::CropBox<pcl::PointXYZRGB> boxFilter;        //Cropbox filter on original pointcloud
        boxFilter.setMin(Eigen::Vector4f(c1.x - ((maxX - minX) / 2 + 0.05), c1.y - ((maxY - minY) / 2 + 0.05), c1.z - ((maxZ - minZ) / 2 + 0.05), 1.0));
        boxFilter.setMax(Eigen::Vector4f(c1.x + ((maxX - minX) / 2 + 0.05), c1.y + ((maxY - minY) / 2 + 0.05), c1.z + ((maxZ - minZ) / 2 + 0.05), 1.0));
        boxFilter.setInputCloud(data);
        boxFilter.filter(*temp_cloud);
        *person_clusters += *temp_cloud; //Issues with modifying object? Prob not
        boxFilter.setInputCloud(origFrame);
        boxFilter.filter(*temp_cloud);
        *background_points += *temp_cloud;
        NeslCoord c;
        Eigen::Vector3d camVector(c1.x + translationMatrix(0), c1.y + translationMatrix(1),
         c1.z + translationMatrix(2));
        camVector = rotationMatrix * camVector;
        c.x = camVector(0);
        c.y = camVector(1);
        c.z = camVector(2);
        double probability = 0;
        std::cout << "Old: "  << c1.x << " " << c1.y << " " << c1.z << " New: ";
        std::cout << c.x << " " << c.y << " " << c.z << std::endl;
        //Compute probability for person being misidentified
        for (int i = 0; i < arrOfPeople.personArr.size(); i++) {
            if (c.x != arrOfPeople.personArr.at(i).personCoord.x && c.z != arrOfPeople.personArr.at(i).personCoord.z) {
                probability += 0.5 / computeAbsDiff(arrOfPeople.personArr.at(i).personCoord, c);
            }
        }
        //Update characteristics of person if identified, if not, add to vector
        bool identified = false;
        for (int i = 0; i < arrOfPeople.personArr.size(); i++) {
            if (evaluateDifference(arrOfPeople.personArr.at(i), c, redAvg, greenAvg, blueAvg, probability)) {
                std::vector<double> tempVec; 
                tempVec.push_back(redAvg);
                tempVec.push_back(greenAvg);
                tempVec.push_back(blueAvg);
                arrOfPeople.personArr.at(i).personCoord.x = c.x;
                arrOfPeople.personArr.at(i).personCoord.y = c.y;
                arrOfPeople.personArr.at(i).personCoord.z = c.z;
                arrOfPeople.personArr.at(i).colorArr = tempVec;
                Eigen::Vector3d lengthVector(maxX - minX + translationMatrix(0), maxY - minY + translationMatrix(1),
                    maxZ - minZ + translationMatrix(2));
                lengthVector = rotationMatrix * lengthVector;
                arrOfPeople.personArr.at(i).bbx = lengthVector(0);
                arrOfPeople.personArr.at(i).bby = lengthVector(1);
                arrOfPeople.personArr.at(i).bbz = lengthVector(2);
                arrOfPeople.personArr.at(i).accountedFor = true;
                identified = true;
            }
        }
        
        //Add to person vector if not identified
        if (!identified) {
            std::vector<double> tempVec; 
            tempVec.push_back(redAvg);
            tempVec.push_back(greenAvg);
            tempVec.push_back(blueAvg);
            Person tempPerson;
            tempPerson.colorArr = tempVec;
            tempPerson.personID = personIDNum++;
            tempPerson.personCoord = c;
            tempPerson.accountedFor = true;
            tempPerson.bbx = maxX - minX;
            tempPerson.bby = maxY - minY;
            tempPerson.bbz = maxZ - minZ;
            tempPerson.talking = false;
            arrOfPeople.personArr.push_back(tempPerson);
        }
    }
    //Nothing, publish empty pointcloud
    if (loopCount3 == 0) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        temp_cloud->header.frame_id = "backgone3";
        temp_cloud->header.stamp = (long) (ros::Time::now().toSec() * 1e6);
        pub.publish(temp_cloud);
        arrOfPeople.personArr.clear();
        arrOfPeople.header.stamp.sec = (long) (ros::Time::now().toSec());
        arrOfPeople.header.stamp.nsec = (ros::Time::now().toSec() - (long) (ros::Time::now().toSec())) * 1e9;
        pub2.publish(arrOfPeople);
        return;
    }
    //Remove all past people that did not show up in current frame
    for (int i = 0; i < arrOfPeople.personArr.size(); i++) {
        if (!arrOfPeople.personArr.at(i).accountedFor) {
            arrOfPeople.personArr.erase(arrOfPeople.personArr.begin() + i);
            i--;
        }
        else {
            arrOfPeople.personArr.at(i).accountedFor = false;
        }
    }
    //Round 2 background subtraction
    octree2->setInputCloud(background_points);
    octree2->addPointsFromInputCloud();
    octree2->switchBuffers();
    octree2->setInputCloud(person_clusters);
    octree2->addPointsFromInputCloud();
    std::vector<int> different_points;
    octree2->getPointIndicesFromNewVoxels(different_points); //Extract points that differ
    int size2 = different_points.size();                     //Store in size variable to avoid calling .size() in loop
    octree2->deleteTree();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_clusters_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); //Create new pointcloud to fill with the points that differ
    //Parallelizing doesn't seem to result in significant performance gain
    //Construct new pointcloud, BUT WITH TRANSFORMED TO APRILTAG COORDINATE FRAME!
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
    previousTree->setInputCloud(person_clusters_filtered);
    previousTree->addPointsFromInputCloud();
    std::vector<int> newPoints;
    previousTree->getPointIndicesFromNewVoxels(newPoints);
    previousTree->switchBuffers();
    /*
    std::cout << newPoints.size() / ((double) person_clusters_filtered->points.size()) << std::endl;
    if (newPoints.size() / ((double) person_clusters_filtered->points.size()) > DIFFERING_PROPORTION_UPPER) {
        initialDownsampleRate = 2;
        secondDownsampleRate = 11;
    }
    else if (newPoints.size() / ((double) person_clusters_filtered->points.size()) < DIFFERING_PROPORTION_LOWER) {
        initialDownsampleRate = 1;
        secondDownsampleRate = 23;
    }
    else {
        initialDownsampleRate = 2;
        secondDownsampleRate = 9;
    }
    */
    //Timestamp and publish
    person_clusters_filtered->header.stamp = (long) (ros::Time::now().toSec() * 1e6);
    arrOfPeople.header.stamp.sec = (long) (ros::Time::now().toSec());
    arrOfPeople.header.stamp.nsec = (ros::Time::now().toSec() - (long) (ros::Time::now().toSec())) * 1e9;
    person_clusters_filtered->header.frame_id = "backgone3";
    pub.publish(person_clusters_filtered);
    pub2.publish(arrOfPeople);
    
}

//Initializes the camera by setting reference frame
void initializeCamera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &data)
{
    *origFrame = *data;
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(data);
    filter.setLeafSize(0.027, 0.027f, 0.027f); //Might be able to adjust this to a larger value since we don't need high resolution for this pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledData(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter.filter(*downsampledData);
    //THIS CODE IS WEIRD BUT IT FIXES AN ISSUE
    octree->deleteCurrentBuffer();
    octree->setInputCloud(downsampledData);
    octree->addPointsFromInputCloud();
    octree->switchBuffers();
    octree->setInputCloud(downsampledData);
    octree->addPointsFromInputCloud();
    octree->switchBuffers();
    std::cout << "Initializing" << std::endl;
}


//Utilize aruco library to transform to AprilTag reference
void getTransform(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat image) {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    cv::Mat imageCopy;
    rotationMatrix << 1, 0, 0,
                      0, 1, 0,
                      0, 0, 0;
    std::cout << "Marker number is:" << ids.size() << std::endl;
    if (ids.size() > 0) {
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
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera3");
    ros::NodeHandle nh;
    rs2::pointcloud pc;
    rs2::config cfg;
    cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera3/clouds/original_cloud", 2);
    color_publisher = nh.advertise<sensor_msgs::Image>("/camera3/color", 2);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera3/clouds/subtracted", 2);
    pub2 = nh.advertise<PersonArr>("/camera3/people", 2);
    //Only enable camera and depth, other streams will likely cause more latency
    rs2::points points;
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);
    //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
   // cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);
    //Obtain camera constants to use with AprilTag
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
    for (int i = 0; i < 5; i++) {
        tempIntrinsics[i] = color_intrinsics.coeffs[i];
    }
    cv::Mat distortionMatrix(1, 5, CV_32FC1, tempIntrinsics);
    int currNumFrames = 0;
    bool initializing = true;
    while (ros::ok())
    { //ros::ok()??
        //Get relevant frame data
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        double startTime = ros::Time::now().toSec();
        //map pointcloud to color? (I don't really know what this does)
        pc.map_to(color);
        int colorHeight = color.get_height();
        int colorWidth = color.get_width();
        int colorBytes = color.get_bytes_per_pixel();
        int colorStride = color.get_stride_in_bytes();

        //Iterate through the image and add to the ros message
        sensor_msgs::Image color_img;
        uint8_t *colorArr = (uint8_t *)(color.get_data());
        std::vector<uint8_t> vec;
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
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto Texture_Coord = points.get_texture_coordinates();
        auto Vertex = points.get_vertices();

        int pointSize = points.size();
        //Only utilizes every third point to downsample higher up the pipeline and improve performance
        //On Intel NUC, 30fps is achieved such that the limiting factor is the framerate of camera!!
        
        double startTime2 = ros::Time::now().toSec();
        for (int i = 0; i < pointSize; i+=2)
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
        }
        //std::cout << ros::Time::now().toSec() - startTime2 << std::endl;
        pcl_points->header.frame_id = "PC3";
        color_img.header.frame_id = "color_img3";
        color_img.width = colorWidth;
        color_img.height = colorHeight;
        color_img.encoding = "rgb8";
        color_img.is_bigendian = true;
        

        
        //CYCLE THE BUFFERS OF THE OCTREE, I HAVE NO IDEA WHY THIS FIXES IT
        if (initializing && currNumFrames++ <= 300)
        {
            if (currNumFrames == 200) {
                getTransform(camMatrix, distortionMatrix, cv::Mat(colorHeight, colorWidth, CV_8UC3, colorArr));
                pipe.stop();
                cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
                cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
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
            process(pcl_points);
        }
        pcl_points->header.stamp = (long) (ros::Time::now().toSec() * 1e6);
        color_img.header.stamp.sec = (int) (ros::Time::now().toSec());
        color_img.header.stamp.nsec = (ros::Time::now().toSec() - (int) (ros::Time::now().toSec())) * 1e9;
        cloud_publisher.publish(pcl_points);
        color_publisher.publish(color_img);
        //std::cout << ros::Time::now().toSec() - startTime << std::endl;
    }
}
