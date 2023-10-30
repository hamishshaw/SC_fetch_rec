#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

#include "findeuc.h"
#include "ClusterSearch.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include "findeuc.h"
#include "ClusterSearch.h"

#include "vector"

//declare some global variables
ros::Publisher pc2_pub;
ros::Subscriber pc2_sub;
ros::Publisher pose_pub;
ros::Publisher pc2_clusters_pub;


// to store index of coke cluster
int clust_index;
bool not_clustered;

void pointcloud2CB(const sensor_msgs::PointCloud2::ConstPtr &cloud_in) 
{   
    // load in a pointcloud from sensormsg
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_in, *cloudptr);
    if(cloudptr->size()< 1)
    {   
        ROS_ERROR_STREAM("failed to load pointcloud to pcl type");
        return;
    }

    // perform euclidian clustering on the raw cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    FindEuc find(cloudptr);
    find.getIndividualClusters(clusters);
    ROS_INFO_STREAM("there are "<<clusters.size()<< " clouds in clusters");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = find.getClusters();
    //publish the cloud to see all clustered objects
    pcl::toROSMsg(*cloud, output_cloud);
    output_cloud.header.frame_id = cloud_in->header.frame_id;
    pc2_clusters_pub.publish(output_cloud);

    // now search these clusters for cluster that most looks like coke can
    ClusterSearch search(clusters);
    search.search(clust_index);
    clusters[clust_index]->height = clusters[clust_index]->size();
    clusters[clust_index]->width = 1;
    pcl::toROSMsg(*clusters[clust_index], output);
    output.header.frame_id = cloud_in->header.frame_id;
    pc2_pub.publish(output);
    
    // get the centre of the can and publish the pose 
    geometry_msgs::PoseStamped pose_stamp;
    geometry_msgs::Pose pose;
    search.xyzObject(pose);
    pose_stamp.pose = pose;
    pose_stamp.header.frame_id = cloud_in->header.frame_id;

    pose_pub.publish(pose_stamp);


    not_clustered = false;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "cloud_cluster");
    ros::NodeHandle nh;
    not_clustered = true;
    // advertise the clusters

    pc2_pub = nh.advertise<sensor_msgs::PointCloud2> ("suspect_cloud", 3, true);
    pc2_clusters_pub = nh.advertise<sensor_msgs::PointCloud2> ("clustered_clouds", 3, true);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("can_location",3 , true);
    //subscribe to a pointcloud and save
    ros::Subscriber pc2_sub;

    pc2_sub = nh.subscribe ("/head_camera/depth_downsample/points", 1, pointcloud2CB);
    ros::Rate limiter(5);
    while( ros::ok() && not_clustered)
    {
        ros::spinOnce();
        limiter.sleep();
    }
    
}