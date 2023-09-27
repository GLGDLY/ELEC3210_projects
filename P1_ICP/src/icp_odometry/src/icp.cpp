/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"
#include <pcl/registration/icp.h>
#include "parameters.h"
#include <pcl/kdtree/kdtree.h>

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    // This is an example of using pcl::IterativeClosestPoint to align two point clouds
    // In your project, you should implement your own ICP algorithm!!!
    // In your implementation, you can use KDTree in PCL library to find nearest neighbors
    // Use chatGPT, google and github to learn how to use PCL library and implement ICP. But do not copy totally. TA will check your code using advanced tools.
    // If you use other's code, you should add a reference in your report. https://registry.hkust.edu.hk/resource-library/academic-integrity

    // example
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(src_cloud);
    // icp.setInputTarget(tar_cloud);
    // icp.setMaximumIterations(params::max_iterations);  // set maximum iteration
    // icp.setTransformationEpsilon(1e-6);  // set transformation epsilon
    // icp.setMaxCorrespondenceDistance(params::max_distance);  // set maximum correspondence distance
    // pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    // icp.align(aligned_cloud, init_guess.cast<float>());

    // Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();

    // 1. https://pointclouds.org/documentation/tutorials/kdtree_search.html

    // create kdtree and populate it with tar
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(tar_cloud);

    // icp need k = 1 kek
    constexpr int K = 1;
    constexpr int MAX_ITER = 100;
    constexpr float THRESHOLD = 10;
    bool converged = false;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    int num_of_iters = 0;
    while (!converged || num_of_iters < MAX_ITER) {
        // loop through each point in the src point cloud and find its nearest neighbor in tar
        for (std::size_t sample = 0; sample < src_cloud->points.size(); sample += 50) {
            // once found just overwrite the value of the point in src
            if (kdtree.nearestKSearch (src_cloud->points[sample], K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
                for (std::size_t i = 0; i < pointIdxKNNSearch.size (); i++) {
                    src_cloud->points[sample].x = tar_cloud->points[pointIdxKNNSearch[i]].x;
                    src_cloud->points[sample].y = tar_cloud->points[pointIdxKNNSearch[i]].y;
                    src_cloud->points[sample].z = tar_cloud->points[pointIdxKNNSearch[i]].z;
                }
            }
            // check converge
            if (std::all_of(pointKNNSquaredDistance.cbegin(), pointKNNSquaredDistance.cend(), [](int i) { return i < THRESHOLD; })) {
                converged = true;
            }
            
        }
        num_of_iters++;   
    }

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    // 2. https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/src/ICP.cpp
        

    return transformation;
}