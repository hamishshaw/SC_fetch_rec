#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/pcd_io.h>
#include <string> 
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <vtkPLYReader.h>

int
main(int argc, char** argv)
{
	// Load the PLY model from a file.
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(argv[1]);
	reader->Update();

	// VTK is not exactly straightforward...
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();

	vtkSmartPointer<vtkPolyData> object = mapper->GetInput();

	// Virtual scanner object.
	pcl::apps::RenderViewsTesselatedSphere render_views;
    
	render_views.addModelFromPolyData(object);
	// Pixel width of the rendering window, it directly affects the snapshot file size.
	render_views.setResolution(150);
	// Horizontal FoV of the virtual camera.
	render_views.setViewAngle(57.0f);
	// If true, the resulting clouds of the snapshots will be organized.
	render_views.setGenOrganized(true);
	// How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
	render_views.setTesselationLevel(1);
	// If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
	// This will affect the number of snapshots produced (if true, less will be made).
	// True: 42 for level 1, 162 for level 2, 642 for level 3...
	// False: 80 for level 1, 320 for level 2, 1280 for level 3...
	render_views.setUseVertices(true);
	// If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
	render_views.setComputeEntropies(true);

	render_views.generateViews();

	// Object for storing the rendered views.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
	// Object for storing the poses, as 4x4 transformation matrices.
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	// Object for storing the entropies (optional).
	std::vector<float> entropies;
	render_views.getViews(views);
	render_views.getPoses(poses);
	render_views.getEntropies(entropies);
	
    // create a collection of edited clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>> views_edited;
	// for each view we are going to calculate normals, and save a vhf descriptor aswell as the view.
    for( int i; i<views.size(); i++)
    {   
		// some ptrs to hold cloud info 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
        
		// delete nans and insert into empty pointcloud
		pcl::Indices indices;
		views[i]->is_dense=false;
		pcl::removeNaNFromPointCloud(*views[i], *cloud, indices);

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ne (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree_ne);
		ne.setRadiusSearch(0.015);
		ne.compute(*normals);

		// Create the VFH estimation class, and pass the input dataset+normals to it
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud (cloud);
		vfh.setInputNormals (normals);
 
		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_vfh (new pcl::search::KdTree<pcl::PointXYZ> ());
		vfh.setSearchMethod (tree_vfh);

		// Output datasets
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

		// Compute the features
		vfh.compute (*vfhs);
		
		// save edited pointcloud as pcd
		std::string filename = "view_";
        filename += std::to_string(i);
        filename += ".pcd";
        pcl::io::savePCDFileASCII (filename, *views[i]);

		// save vhf
		filename = "view_";
        filename += std::to_string(i);
		filename += "_vfh.pcd";
		pcl::io::savePCDFileASCII(filename,*vfhs);



		cloud.reset();
		normals.reset();
    }


}