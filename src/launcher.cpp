#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <queue>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
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
#include <cmath>
#include <thread>
#include <string>
#include <mutex>
#include <chrono>
using namespace std;
using namespace std::chrono;

// Create octree used for initial background subtraction with 0.15 voxel size
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *stage1_octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.15);
// Create octree used for the second round of background subtraction
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *stage2_octree = new pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB>(0.1);

// Keep track of original frame for background subtraction purposes
pcl::PointCloud<pcl::PointXYZRGB>::Ptr origFrame(new pcl::PointCloud<pcl::PointXYZRGB>());


// Respeaker will point in the +y direction in AprilTag Frame
const double respeakerX = 0;
const double respeakerY = 0;
const double respeakerZ = 0;


//LiDAR camera serial numbers
const int MIN_PERSON_POINTS = 300;

//Timestamp for latency
double processStart = 0;
std::mutex mtx;

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



//Extract clusters from given point cloud
std::vector<pcl::PointIndices> getClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_ptr)
{
    // Search for meaningful clusters
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(diff_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.10); // 15cm, increase for speed?
    ec.setMinClusterSize(MIN_PERSON_POINTS);    // Adjust this to detect people only, depends on how camera is oriented
    ec.setMaxClusterSize(300000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(diff_cloud_ptr);
    ec.extract(cluster_indices);
    return cluster_indices;
}


// Takes in a pointcloud and publishes the centroid coordinates as well as the subtracted cloud
void process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr data_down, int frameNum)
{
    // Utilize a VoxelGrid filter to match dimensions of the input w the reference, also reduces pointcloud size further
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(data_down);
    filter.setLeafSize(0.03f, 0.03f, 0.03f); 
    filter.filter(*data_down);

    // Fill octree with downsampled pointcloud data
    stage1_octree->setInputCloud(data_down);
    stage1_octree->addPointsFromInputCloud();
    std::vector<int> diff_point_vector;
    stage1_octree->getPointIndicesFromNewVoxels(diff_point_vector);
    int size = diff_point_vector.size();

 
    // Octree sometimes encounters a bug where it deletes the reference buffer, causing the background subtraction to do nothing
    // Reset reference buffer
    if (size > 0.9 * data_down->points.size())
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
            std::cout << "WARNING: OCTREE RESET" << std::endl;
        }
        mtx.unlock();
        return;
    }
    else
    {
        stage1_octree->deleteCurrentBuffer(); // Otherwise, delete current buffer and proceed
    }

    // In case of few points signalling no subjects, simply return
    if (diff_point_vector.size() < MIN_PERSON_POINTS)
    {
        mtx.unlock();
	std::cout << "No new points found" << std::endl;
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr diff_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); // Create new pointcloud to fill with the points that differ
    // Build point cloud of differing voxels
    for (int pc_counter = 0; pc_counter < size; pc_counter++)
    {
        diff_cloud_ptr->points.push_back(data_down->points[diff_point_vector[pc_counter]]);
    }
    

    //Break point cloud into clusters
    std::vector<pcl::PointIndices> cluster_indices = getClusters(diff_cloud_ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr background_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
        
    // Add all meaningful clusters to a new pointcloud object
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
        //in AprilTag coordinate frame
        
        bool firstTime = true;
        // Computes min/max x, y, z in both frames for a given cluster
        for (const auto &idx : it->indices)
        {
            pcl::PointXYZRGB tempPoint = (*diff_cloud_ptr)[idx];
            //Transform to AprilTag coordinate system, bc we must find minX, Y, and Z in the frame of the AprilTag!
           
            if (firstTime)
            {
                minX = maxX = tempPoint.x;
                minY = maxY = tempPoint.y;
                minZ = maxZ = tempPoint.z;
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
        //Compute transform to APRILTAG frame and assign to Coord object
        clusterIndex++;

        //All coordinates and measurements are in the AprilTag reference frame
    }

    // If no clusters, publish empty pointcloud
    if (clusterIndex == -1)
    {
        mtx.unlock();
	std::cout << "Return, no clusters" << std::endl;
        return;
    }
    
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr person_clusters_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 


    // Round 2 background subtraction
    double performanceTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

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
        person_clusters_filtered->points.push_back(tempPoint);
    }
    mtx.unlock();
    std::cout << "Output point cloud with size " << size2 << endl;
    
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


int main(int argc, char *argv[])
{

    // Only enable camera and depth, other streams will likely cause more latency
    rs2::points points;
    rs2::config cfg;
    rs2::pointcloud pc;

    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1024, 768, RS2_FORMAT_Z16, 30);

  
    
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    int currNumFrames = 0;
    bool initializing = true;
    std::cout << std::setprecision(4) << std::fixed;
    //Frame number for timing analysis
    int frameNum = 0;
    while (true)
    { // ros::ok()??
        // Get relevant frame data
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        auto receivedCloud = std::chrono::high_resolution_clock::now();
        //std::cout << "Frame number " << ++frameNum << " received at " << ros::Time::now().toSec() << std::endl;
        // map pointcloud to color? (I don't really know what this does)
        pc.map_to(color);
        int colorHeight = color.get_height();
        int colorWidth = color.get_width();
        int colorBytes = color.get_bytes_per_pixel();
        int colorStride = color.get_stride_in_bytes();
	uint8_t *colorArr = (uint8_t *)(color.get_data());
        
        // Iterate through the image and add to the ros message
     
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
        //std::cout << "Frame number " << frameNum << " point cloud generated at " << ros::Time::now().toSec() << std::endl;

     

        // CYCLE THE BUFFERS OF THE OCTREE, I HAVE NO IDEA WHY THIS FIXES IT
        if (initializing && currNumFrames++ <= 50)
        {
            if (currNumFrames == 20)
            {
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
            //13 ms
            processStart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            mtx.lock();
            std::thread processCloud(process, pcl_points, pcl_pointsDown, frameNum);
            processCloud.detach();
        }
        std::cout <<std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now()-receivedCloud).count() << std::endl;
    }
}
