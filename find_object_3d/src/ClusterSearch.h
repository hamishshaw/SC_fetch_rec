#ifndef CLUSTERSEARCH
#define CLUSTERSEARCH

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h> // for compute3DCentroid
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <limits>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp> // for replace_last


#include <geometry_msgs/PoseStamped.h>



typedef std::pair<std::string, std::vector<float> > vfh_model;

class ClusterSearch
{   

    public:
    //Default constructor
    ClusterSearch( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds);

    /** \brief searches the loaded clusters and decides which cluster is most similar to the described model
     */
    bool search(int &cluster_idx);

    bool xyzObject(geometry_msgs::Pose &pose);
    
    
    private:

    /** \brief Loads an n-D histogram file as a VFH signature
     * \param path the input file name
     * \param vfh the resultant VFH model
     */
    bool loadHist (const boost::filesystem::path &path, vfh_model &vfh);
    
     /** \brief Search for the closest k neighbors
     * \param index the tree
     * \param model the query model
     * \param k the number of neighbors to search for
     * \param indices the resultant neighbor indices
     * \param distances the resultant neighbor distances
     */
    inline void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

    /** \brief Load the list of file model names from an ASCII file
     * \param models the resultant list of model name
     * \param filename the input file name
     */
    bool loadFileList (std::vector<vfh_model> &models, const std::string &filename);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
    std::vector<vfh_model> models_test_;
    int idx_;
};

#endif