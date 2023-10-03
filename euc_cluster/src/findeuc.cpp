#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill

#include "findeuc.h"
#include "ros/ros.h"

#include "iostream"

// default constructor takes PCL pointcloud
FindEuc::FindEuc( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud):
cloud_(cloud), cloud_cluster_(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
cloud_f_(new pcl::PointCloud<pcl::PointXYZ>), inliers_(new pcl::PointIndices),coefficients_(new pcl::ModelCoefficients),
cloud_plane_(new pcl::PointCloud<pcl::PointXYZ>)
{
    voxelgrid();
    // Create the segmentation object for the planar model and set all the parameters
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setMaxIterations (100);
    seg_.setDistanceThreshold (0.02); 
    cluster();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FindEuc::getClusters()
{
    return cloud_cluster_;
}



void FindEuc::voxelgrid()
{
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered_);
    nr_points_ = cloud_filtered_->size();
    ROS_INFO_STREAM("nr_points = " << nr_points_);
    std::cout << "PointCloud after filtering has: " << cloud_filtered_->size ()  << " data points." << std::endl; //*
}


void FindEuc::cluster()
{
    while (cloud_filtered_->size () > 0.3 * nr_points_)
    {
    // Segment the largest planar component from the remaining cloud
        seg_.setInputCloud (cloud_filtered_);
        seg_.segment (*inliers_, *coefficients_);
        if (inliers_->indices.size () == 0)
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered_);
        extract.setIndices (inliers_);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane_);
        std::cout << "PointCloud representing the planar component: " << cloud_plane_->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f_);
        *cloud_filtered_ = *cloud_f_;
    }
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered_);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered_);
    ec.extract (cluster_indices);

    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
        for (const auto& idx : cluster.indices) {
        cloud_cluster_->push_back((*cloud_filtered_)[idx]);
        } //*
        cloud_cluster_->width = cloud_cluster_->size ();
        cloud_cluster_->height = 1;
        cloud_cluster_->is_dense = true;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        //writer_.write<pcl::PointXYZ> ("cloud_cluster_" + ss.str () + ".pcd", *cloud_cluster_, false); //*
        j++;
    }
}


