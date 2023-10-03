#ifndef FINDEUC_H
#define FINDEUC_H

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
#include <iomanip> 
// This class takes an input cloud and estimates a plane that obects are placed on. 
// It then estimates what objects or on this plane and saves them as clusters, essential removing the background featureless space
class FindEuc
{

    public:
    //constructor
    FindEuc( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // return the clustered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr getClusters();

    private:

    // pre-clustering filtering
    void voxelgrid();
    // performs clustering
    void cluster();
    // stored data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_;
    //
    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    pcl::PointIndices::Ptr inliers_;
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_;
    pcl::PCDWriter writer_;
    int nr_points_;
    //

};

#endif