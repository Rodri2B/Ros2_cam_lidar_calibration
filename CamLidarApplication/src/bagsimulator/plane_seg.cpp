#include <iostream>

#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>

#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>

#include <iomanip> // for setw, setfill

int main ()
{

// Read in the cloud data

pcl::PCDReader reader;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

std::string path = "../resources/clouds/";
reader.read ((path+"table_scene_lms400.pcd"), *cloud);

std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*


// Create the filtering object: downsample the dataset using a leaf size of 1cm

pcl::VoxelGrid<pcl::PointXYZ> vg;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

vg.setInputCloud (cloud);

vg.setLeafSize (0.01f, 0.01f, 0.01f);

vg.filter (*cloud_filtered);

std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

//*cloud_filtered = *cloud;

// Create the segmentation object for the planar model and set all the parameters

pcl::SACSegmentation<pcl::PointXYZ> seg;

pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector;

pcl::PCDWriter writer;

seg.setOptimizeCoefficients (true);

seg.setModelType (pcl::SACMODEL_PLANE);

seg.setMethodType (pcl::SAC_LMEDS);

seg.setMaxIterations (300);

seg.setDistanceThreshold (0.02);


int nr_points = (int) cloud_filtered->size ();


while (cloud_filtered->size () > 0.3 * nr_points)

{

    // Segment the largest planar component from the remaining cloud

    seg.setInputCloud (cloud_filtered);

    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)

    {

      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

      break;

    }


    // Extract the planar inliers from the input cloud

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cloud_filtered);

    extract.setIndices (inliers);

    extract.setNegative (false);


    // Get the points associated with the planar surface

    extract.filter (*cloud_plane);

    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 

                                        << coefficients->values[1] << " "

                                        << coefficients->values[2] << " " 

                                        << coefficients->values[3] << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& idx : inliers->indices) {

        cloud_cluster->push_back((*cloud_plane)[idx]);

    } //*

    cluster_vector.push_back(cloud_cluster);

    // Remove the planar inliers, extract the rest

    extract.setNegative (true);
    //cloud_f.reset(); //release the memory before the pointer goes out of scope
    extract.filter (*cloud_f);
    //cloud_filtered.reset();
    *cloud_filtered = *cloud_f;
    //cloud_f->clear(); //clear the contents of the point cloud without releasing the memory
}

for(int i =0; i < cluster_vector.size(); i++){

    std::stringstream ss;

    ss << std::setw(4) << std::setfill('0') << i;

    writer.write<pcl::PointXYZ> (path + "plane_cluster_" + ss.str () + ".pcd", *cluster_vector[i], false); //*
}

return (0);

}
