// load a set of clustered clouds.
// Find which cloud is most similar to can vfh
// find centre of cluster

#include <pcl/features/normal_3d.h>
#include "ClusterSearch.h"
#include <pcl/features/vfh.h>
#include <string>

ClusterSearch::ClusterSearch( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds)

{
    clouds_ = clouds;
    // create a vfh descriptor for each cloud within clouds
    pcl::console::print_info("clouds_ size = %d\n",clouds_.size());
    for(int i = 0; i < clouds.size(); i++)
    {   
    pcl::console::print_info("in constructor loop\n");
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
    // Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (clouds_[i]);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);
		ne.setRadiusSearch(0.015);
		ne.compute(*normals);

		// Create the VFH estimation class, and pass the input dataset+normals to it
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud (clouds_[i]);
		vfh.setInputNormals (normals);
 
		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		vfh.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

		// Compute the features for each cluster that is loaded. create a vfh model that is later used in ksearch
		vfh.compute (*vfhs);
    
    vfh_model model;
    model.first = "clusterNum_"+ std::to_string(i);
    model.second.resize(308);

    for(int n = 0; n < vfhs->at(0).descriptorSize(); n++)
    {
      model.second[n]= vfhs->at(0).histogram[n];
    }
    models_test_.push_back(model);

    vfhs.reset();
    }
}



bool ClusterSearch::loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (const pcl::InvalidConversionException&)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <pcl::PCLPointField> fields;
  pcl::getFieldIndex<pcl::VFHSignature308> ("vfh", fields);

  for (std::size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}

inline void ClusterSearch::nearestKSearch(flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}


bool ClusterSearch::loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  std::ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    std::getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

bool ClusterSearch::search(int &cluster_idx)
{
  // prepare some variables to use for loading data
  int k = 6;

  std::string kdtree_idx_file_name = "kdtree.idx";
  std::string training_data_h5_file_name = "training_data.h5";
  std::string training_data_list_file_name = "training_data.list";

  std::vector<vfh_model> models;
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  flann::Matrix<float> data;
  // load in pretrained data used for ksearch
    // Check if the data has already been saved to disk
  if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
  {
    pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
        training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    return (false);
  }
  else
  {
    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "training_data");
    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
  }

  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_file_name))
  {
    pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
    return (false);
  }

  // finally loading the kdtree in a flann format
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
  index.buildIndex ();


  // now that the data is loaded we can iterate through the clusters and see which has the smallest k distance
  float distance = 1000.00;
  int idx;
  for(int i = 0; i < models_test_.size(); i++)
  {
    nearestKSearch (index, models_test_[0], k, k_indices, k_distances);
    pcl::console::print_info("k_distance = %d\n",k_distances[0][0]);

    // find the index of the cluster with the smallest k_distance / most similar
    if(k_distances[0][0]<distance)
    {
      idx = i;
    }
  }
  cluster_idx = idx;
  
  pcl::console::print_info("the most similar cluster is at %d \n",idx);

  return(true);
}