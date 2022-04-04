#include "PclStuff.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;


Cloud::Ptr PclStuff::Downsample(const Cloud::ConstPtr &cloud_in,
                                float leaf_size) {
    pcl::console::TicToc tt;
    Cloud::Ptr cloud_result(new Cloud());
//    std::cerr << "Downsampling...\n", tt.tic ();
    pcl::VoxelGrid<Point> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud_in);
    grid.filter(*cloud_result);
//    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_result->size () << " points\n";

    return cloud_result;
}

//Defination of a function
Cloud::Ptr
PclStuff::GroundRemover(Cloud::ConstPtr cloud_in, float treshold) {
//  creates new object to fill point cloud
    Cloud::Ptr cloud_groundless(new Cloud);
    pcl::SACSegmentation<Point> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(1000);
//    seg.setDistanceThreshold(treshold);
    seg.setMaxIterations(2000);
    seg.setDistanceThreshold(treshold);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cout
                << "Could not estimate a planar model for the given dataset."
                << std::endl;
        *cloud_groundless = *cloud_in;
        return cloud_groundless;
    }

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_groundless);
    return cloud_groundless;
}

Cloud::Ptr
PclStuff::GroundFinder(Cloud::ConstPtr cloud_in, float treshold) {
    // Create the filtering object
    Cloud::Ptr cloud_ground(new Cloud);
    pcl::SACSegmentation<Point> seg;
    pcl::PointIndices::Ptr ground(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(treshold);
    seg.setInputCloud(cloud_in);
    seg.segment(*ground, *coefficients);

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(ground);
    extract.filter(*cloud_ground);
    std::cout
            << *cloud_ground
            << std::endl;


    return cloud_ground;


}


Cloud::Ptr
PclStuff::EucledianCluster(Cloud::ConstPtr cloud_in) {
//    clud_in :point cloud includes sub points
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;


    ec.setClusterTolerance(0.2); // 2cm
    ec.setMinClusterSize(18);
    ec.setMaxClusterSize(2500);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {

        for (const auto &idx: it->indices) {
//          every points inside pointcloud
          std::cout << cloud_in->points[idx] << std::endl;
            pcl::PointXYZI p_old = cloud_in->points[idx];
            pcl::PointXYZI p_new;
            p_new.x = p_old.x;
            p_new.y = p_old.y;
            p_new.z = p_old.z;
//            iterates 100 over intensity for points
            p_new.intensity = j;
//          push_back works lıke .append
            cloud_cluster->push_back((p_new)); //*

        }
//        std::cout << "Type of x : " << typeid(cloud_cluster).name() << std::endl;
//        std::cout<<cloud_cluster->points.data()<<std::endl;
//        std::cout<<(*cloud_cluster)[it]<<std::endl;
        j += 100;
        //return cloud_cluster;
    }
//    pointcloud configs before publish
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    return cloud_cluster;


}
