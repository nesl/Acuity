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

using namespace NESLMessages;
using namespace std;
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.15);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree2 = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.25);
//Used to keep track of when to swap in new background to reduce drift
double lastUpdate;
//Publisher nodes
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher cloud_publisher;
ros::Publisher color_publisher;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr origFrame(new pcl::PointCloud<pcl::PointXYZRGB>());
PersonArr arrOfPeople;

bool evaluateDifference(Person person, pcl::PointXYZ centroid, double red, double green, double blue) {
    double redDiff2 = (red - person.colorArr[0]) * (red - person.colorArr[0]);
    double greenDiff2 = (green - person.colorArr[1]) * (green - person.colorArr[1]);
    double blueDiff2 = (blue - person.colorArr[2]) * (blue - person.colorArr[2]);
    double colorDiff2 = redDiff2 + greenDiff2 + blueDiff2;
    double centroidDiff2 = (person.personCoord.x - centroid.x) * (person.personCoord.x - centroid.x) 
                            + (person.personCoord.y - centroid.y) * (person.personCoord.y - centroid.y) 
                            + (person.personCoord.z - centroid.z) * (person.personCoord.z - centroid.z);
    return colorDiff2 < 150 && centroidDiff2 < 0.12;
}

int personIDNum = 0;
//process function that takes in a pointcloud and publishes the centroid coordinates as well as the subtracted cloud
void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &data)
{
    //Fill octree with pointcloud data
    octree->setInputCloud(data);
    octree->addPointsFromInputCloud();
    std::vector<int> diff_point_vector;
    octree->getPointIndicesFromNewVoxels(diff_point_vector); //Extract points that differ
    int size = diff_point_vector.size();                     //Store in size variable to avoid calling .size() in loop
    octree->deleteCurrentBuffer();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //Create new pointcloud to fill with the points that differ

    //Parallelizing doesn't seem to result in significant performance gain
    for (int pc_counter = 0; pc_counter < size; pc_counter++)
    {
        diff_cloud_ptr->points.push_back(data->points[diff_point_vector[pc_counter]]);
    }

    //In case of small point cloud, simply return
    if (diff_point_vector.size() < 800)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        temp_cloud->header.frame_id = "backgone3";
        pub.publish(temp_cloud);
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
    seg.setDistanceThreshold(0.08); //How close it must be to be an inlier
    seg.setInputCloud(diff_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        temp_cloud->header.frame_id = "backgone3";
        pub.publish(temp_cloud);
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
    ec.setMinClusterSize(500);   //Adjust this to detect people only, depends on how camera is oriented
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
        //Find the min and max points
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
        pcl::CropBox<pcl::PointXYZRGB> boxFilter;
        c1.y = -c1.y; //TODO: THIS IS PURELY FOR ARENA
        c1.z = c1.z - 1.4;
        c1.x = -c1.x;
        std::cout << c1.x << " " << c1.y << " " << c1.z << std::endl;
        boxFilter.setMin(Eigen::Vector4f(c1.x - ((maxX - minX) / 2 + 0.05), c1.y - ((maxY - minY) / 2 + 0.05), c1.z - ((maxZ - minZ) / 2 + 0.05), 1.0));
        boxFilter.setMax(Eigen::Vector4f(c1.x + ((maxX - minX) / 2 + 0.05), c1.y + ((maxY - minY) / 2 + 0.05), c1.z + ((maxZ - minZ) / 2 + 0.05), 1.0));
        boxFilter.setInputCloud(data);
        boxFilter.filter(*temp_cloud);
        *person_clusters += *temp_cloud; //Issues with modifying object? Prob not
        boxFilter.setInputCloud(origFrame);
        boxFilter.filter(*temp_cloud);
        *background_points += *temp_cloud;
        NeslCoord c;
        c.x = c1.x;
        c.y = c1.y;
        c.z = c1.z;
        //Check within the person vector to see if person present
        bool identified = false;
        for (int i = 0; i < arrOfPeople.personArr.size(); i++) {
            if (evaluateDifference(arrOfPeople.personArr.at(i), c1, redAvg, greenAvg, blueAvg)) {
                std::vector<double> tempVec; 
                tempVec.push_back(redAvg);
                tempVec.push_back(greenAvg);
                tempVec.push_back(blueAvg);
                arrOfPeople.personArr.at(i).personCoord.x = c1.x;
                arrOfPeople.personArr.at(i).personCoord.y = c1.y;
                arrOfPeople.personArr.at(i).personCoord.z = c1.z;
                arrOfPeople.personArr.at(i).colorArr = tempVec;
                arrOfPeople.personArr.at(i).bbx = maxX - minX;
                arrOfPeople.personArr.at(i).bby = 3 * (-c.y - minY) + 0.5;
                arrOfPeople.personArr.at(i).bbz = maxZ - minZ;
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
            tempPerson.bby = 3 * (-c.y - minY) + 0.5;
            tempPerson.bbz = maxZ - minZ;
            tempPerson.talking = false;
            arrOfPeople.personArr.push_back(tempPerson);
        }
    }
    if (loopCount3 == 0) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        temp_cloud->header.frame_id = "backgone3";
        pub.publish(temp_cloud);
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
    //Background subtraction v2
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
    for (int pc_counter = 0; pc_counter < size2; pc_counter++)
    {
        person_clusters_filtered->points.push_back(person_clusters->points[different_points[pc_counter]]);
    }

    person_clusters_filtered->header.frame_id = "backgone3";
    pub.publish(person_clusters_filtered);
    pub2.publish(arrOfPeople);
}

//Initializes the camera by setting reference frame
void initializeCamera(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &data)
{
    //THIS CODE IS WEIRD BUT IT FIXES AN ISSUE
    octree->deleteCurrentBuffer();
    octree->setInputCloud(data);
    octree->addPointsFromInputCloud();
    octree->switchBuffers();
    *origFrame = *data;
    octree->setInputCloud(origFrame);
    octree->addPointsFromInputCloud();
    octree->switchBuffers();
    std::cout << "Initializing" << std::endl;
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
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    int currNumFrames = 0;
    bool initializing = true;
    while (ros::ok())
    { //ros::ok()??
        //Get relevant frame data
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        //map pointcloud to color? (I don't really know what this does)
        pc.map_to(color);
        double startTime = ros::Time::now().toSec();
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
        for (int i = 0; i < pointSize; i += 3)
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

        pcl_points->header.frame_id = "PC3";
        color_img.header.frame_id = "color_img3";
        color_img.width = colorWidth;
        color_img.height = colorHeight;
        color_img.encoding = "rgb8";
        color_img.is_bigendian = true;

       
        
        
        //pcl_points->header.stamp = (int) (ros::Time::now().toSec());
        color_img.header.stamp.sec = (int) (ros::Time::now().toSec());
        color_img.header.stamp.nsec = (ros::Time::now().toSec() - (int) (ros::Time::now().toSec())) * 1e9;

        //CYCLE THE BUFFERS OF THE OCTREE, I HAVE NO IDEA WHY THIS FIXES IT
        if (initializing && currNumFrames++ <= 300)
        {
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
        cloud_publisher.publish(pcl_points);
        color_publisher.publish(color_img);
        double timeDiff = ros::Time::now().toSec() - startTime;
        std::cout << timeDiff << std::endl;
    }
}