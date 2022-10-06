#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

ros::Publisher combined_publisher;

void combine(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cam1PC, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cam2PC) {
    pcl::PointCloud<pcl::PointXYZRGB> complete = *cam1PC;
    complete = complete + *cam2PC;
    //Have adjustments
    combined_publisher.publish(complete);
}

int main(int argc, char* argv[]) {
    ros::NodeHandle nh;
    ros::init(argc, argv, "combiner");
    combined_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/combined", 2);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> cam1_sub(nh, "/camera1/subtracted", 5);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> cam2_sub(nh, "/camera2/subtracted", 5);
    typedef sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZRGB>, pcl::PointCloud<pcl::PointXYZRGB>> MySyncPolicyPC;
    Synchronizer<MySyncPolicyPC> syncPC(MySyncPolicyPC(10), cam1_sub, cam2_sub);
    syncPC.registerCallback(boost::bind(&combine, _1, _2));  

} 