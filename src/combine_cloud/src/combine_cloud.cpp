#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// basic file operations
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace message_filters;

ros::Publisher combined_publisher;
std::ofstream myfile;

void combine(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cam1PC, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cam2PC) {
    double start = ros::Time::now().toSec();
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 0.0, 0.0, -0.04;
    transform_2.rotate (Eigen::AngleAxisf (0.05, Eigen::Vector3f::UnitY()));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*cam1PC, *transformed_cloud, transform_2);
    pcl::PointCloud<pcl::PointXYZRGB> complete = *transformed_cloud;
    complete = complete + *cam2PC;
    complete.header.stamp = (long)(ros::Time::now().toSec() * 1e6);
    complete.header.frame_id = "complete";
    //Have adjustments
    combined_publisher.publish(complete);
    std::cout << ros::Time::now().toSec() - start << std::endl;
}

/*
void saveCallBack(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cam2PC) {
    pcl::PointCloud<pcl::PointXYZI> pc;
    for (int i = 0; i < cam2PC->points.size(); i++) {
        pcl::PointXYZRGB p = cam2PC->points.at(i);
        myfile << p.x << " " << p.y << " " << p.z << " 0" << std::endl;
    }
    exit(0);
}
*/


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "combiner");
    ros::NodeHandle nh;
    myfile.open("/home/nesl/Desktop/PointCloud.txt");
    combined_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/combined", 2);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> cam1_sub(nh, "/camera1/clouds/subtracted", 5);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> cam2_sub(nh, "/camera2/clouds/subtracted", 5);
    typedef sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZRGB>, pcl::PointCloud<pcl::PointXYZRGB>> MySyncPolicyPC;
    Synchronizer<MySyncPolicyPC> syncPC(MySyncPolicyPC(10), cam1_sub, cam2_sub);
    //ros::Subscriber sub = nh.subscribe("/camera2/original", 2, saveCallBack);
    syncPC.registerCallback(boost::bind(&combine, _1, _2));  
    ros::spin();

} 