#include "ros/ros.h"
#include "findeuc.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pc2_pub;
bool notclustered = true;

void pointcloud2CB(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
{   
    ROS_INFO_STREAM("input cloud has " << cloud_in->data.size() << " points");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_in, *cloud);

    FindEuc find(cloud);
    ROS_INFO("not initialiser");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    cloud_filtered = find.getClusters();

    ROS_INFO_STREAM("output cloud has " << cloud_filtered->size() << " points");

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "camera_link";
    // publish clustered data
    pc2_pub.publish(output);

    notclustered = false;

}
int main (int argc, char** argv)
{
    ros::init (argc, argv, "cloud_cluster");
    ros::NodeHandle nh;
    
    // advertise the clusters
    pc2_pub = nh.advertise<sensor_msgs::PointCloud2> ("clustered_cloud", 3, true);
    //subscribe to a pointcloud and save
    ros::Subscriber pc2_sub;
    ROS_INFO_STREAM("is this thing on?");
    pc2_sub = nh.subscribe ("/camera/depth/points", 1, pointcloud2CB);
    ros::Rate limiter(5);
    while( ros::ok() && notclustered)
    {
        ros::spinOnce();
        limiter.sleep();
    }
    
}