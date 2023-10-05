/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include "parameters.h"
#include <pcl/kdtree/kdtree.h>


//     /*
//     Here have two point sets:
//     X = {X1, X2, X3, ..., Xn}
//     Y = {Y1, Y2, Y3, ..., Yn}
//     Here, X is tar_cloud and Y is src_cloud

//     Where if we find R and t that fits:
//     X=R*Y+t / X-RY-y = 0, 
//     then the two point set's Correspondence is found
    
//     Change to the Error Function (by minimize this to finish ICP)
//     E(R,t) = SIGMA(n,i=0)[(Xi-R*Yi-t)^2] / Ni
//     Minimize the function output will be good
//     ALSO, Nx = Ny = Ni, or the ICP is not aplicable(here src and tar don't have this problem as odometry will do down sampling）
//     */



//Gary dailo and Linux dailo made this from others repos

// Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
//     // This is an example of using pcl::IterativeClosestPoint to align two point clouds
//     // In your project, you should implement your own ICP algorithm!!!
//     // In your implementation, you can use KDTree in PCL library to find nearest neighbors
//     // Use chatGPT, google and github to learn how to use PCL library and implement ICP. But do not copy totally. TA will check your code using advanced tools.
//     // If you use other's code, you should add a reference in your report. https://registry.hkust.edu.hk/resource-library/academic-integrity

//     // example
//     // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//     // icp.setInputSource(src_cloud);
//     // icp.setInputTarget(tar_cloud);
//     // icp.setMaximumIterations(params::max_iterations);  // set maximum iteration
//     // icp.setTransformationEpsilon(1e-6);  // set transformation epsilon
//     // icp.setMaxCorrespondenceDistance(params::max_distance);  // set maximum correspondence distance
//     // pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
//     // icp.align(aligned_cloud, init_guess.cast<float>());

//     // Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();




//     // 1. https://pointclouds.org/documentation/tutorials/kdtree_search.html

//     // create kdtree and populate it with tar
//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//     kdtree.setInputCloud(tar_cloud);

//     // icp need k = 1 kek
//     constexpr int K = 1;    //only need one neighbor
//     constexpr int MAX_ITER = 100;   //maximun number of iteration
//     constexpr float THRESHOLD = 10; //the convergence threshold
//     bool converged = false;

//     std::vector<int> pointIdxKNNSearch(K);          // the index of neighbor
//     std::vector<float> pointKNNSquaredDistance(K);  // the distance of the neighbor

//     int num_of_iters = 0;
//     while (!converged || num_of_iters < MAX_ITER) {
//         // loop through each point in the src point cloud and find its nearest neighbor in tar
//         for (std::size_t sample = 0; sample < src_cloud->points.size(); sample += 50) {
//             // once found just overwrite the value of the point in src
//             if (kdtree.nearestKSearch (src_cloud->points[sample], K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
//                 for (std::size_t i = 0; i < pointIdxKNNSearch.size (); i++) {
//                     // TODO: HERE NEED CHANGE, need to find the transformation matrix and translation vector
//                     src_cloud->points[sample].x = tar_cloud->points[pointIdxKNNSearch[i]].x;
//                     src_cloud->points[sample].y = tar_cloud->points[pointIdxKNNSearch[i]].y;
//                     src_cloud->points[sample].z = tar_cloud->points[pointIdxKNNSearch[i]].z;
//                 }
//             }
//             // check converge
//             if (std::all_of(pointKNNSquaredDistance.cbegin(), pointKNNSquaredDistance.cend(), [](int i) { return i < THRESHOLD; })) {
//                 converged = true;
//             }
            
//         }
//         num_of_iters++;   
//     }

//     Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

//     // 2. https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/src/ICP.cpp
        

//     return transformation;
// }




//Hartin's shit from himself with help of chatGPTs, and Hartin himself can't understand this shit but looked working(hopefully)

// Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    
//     // Create a kd-tree for the target point cloud
//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//     kdtree.setInputCloud(tar_cloud);

//     // Maximum number of iterations and convergence criteria
//     int max_iterations = 100;
//     double transformation_epsilon = 1e-8;   //Can modify this

//     Eigen::Matrix4d transformation = init_guess;

//     for (int i = 0; i < max_iterations; ++i) {

//         // Transform the source point cloud using the current estimated transformation
//         //Create a new pointCLoud called transformed_src_cloud
//         pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         //Perform a transformation of the src_cloud and store it into transformed_src_cloud according to current transformation matrix
//         pcl::transformPointCloud(*src_cloud, *transformed_src_cloud, transformation);

//         // Find correspondences using kd-tree
//         pcl::Correspondences correspondences;   //create a correspondence vector called correspondences to store the association between a point in the transformed source cloud and its correesponding point in the tar_cloud
//         for (const auto& point : transformed_src_cloud->points) {
//             std::vector<int> indices(1);    //store index of neighboring in the tar_cloud
//             std::vector<float> squared_distances(1); // store the squared distance between the cur point and neighor

//             //find the nearest neighor, 1 means number of neighbors needed and indices and squared_distance is the data from the nearest neighbor
//             kdtree.nearestKSearch(point, 1, indices, squared_distances);

//             //Create a Correspondence object called correspondence. init with index of nearest neighbor and size of the correspondences vector with squared distance from nearest neighbor
//             pcl::Correspondence correspondence(indices[0], static_cast<int>(correspondences.size()), squared_distances[0]); 
//             //add this correspondence object to the correspondence vector.
//             correspondences.push_back(correspondence);
//         }

//         // Estimate the transformation using Singular Value Decomposition (SVD)
//         Eigen::Matrix4d previous_transformation = transformation; //Store previouse transformation
//         transformation = pcl::estimateRigidTransformationSVD(*transformed_src_cloud, *tar_cloud, correspondences);  //Using SVD to calculate the new transformation matrix

//         // Check convergence
//         double delta = (transformation - previous_transformation).norm();   //Calculate the Euclidean orm of the two matrixs, represents the magnitude of the difference between the two transgormation matrices
//         if (delta < transformation_epsilon) {   //if the difference is too small between two genereations, then it is almost aligned, stop here
//             break;
//         }
//     }

//     return transformation;
// }




//try to use TransformationEstimationSVD
Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {

// Fill in the source and target point clouds
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    Eigen::Matrix4f transformation_matrix;
    svd.estimateRigidTransformation(*src_cloud, *tar_cloud, transformation_matrix);
    return transformation_matrix;
}
