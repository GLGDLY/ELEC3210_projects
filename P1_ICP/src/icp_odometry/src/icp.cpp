/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"
#include <pcl/registration/icp.h>
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

//https://blog.csdn.net/taifyang/article/details/113898308 Can see see this

//Gary dailo and Linux dailo made this from others repos

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {

    Eigen::Matrix4d transformation = init_guess;
    Eigen::Vector3f translation = Eighen::Vector3f::Zero();

    // 1. https://pointclouds.org/documentation/tutorials/kdtree_search.html

    // create kdtree and populate it with tar
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(tar_cloud);

    // icp need k = 1 kek
    constexpr int K = 1;    //only need one neighbor
    constexpr int MAX_ITER = 100;   //maximun number of iteration
    constexpr float THRESHOLD = 10; //the convergence threshold
    bool converged = false;

    //Create two temp. for finding nabours
    std::vector<int> pointIdxKNNSearch(K);          // the index of neighbor
    std::vector<float> pointKNNSquaredDistance(K);  // the distance of the neighbor

    //Create array to store the information for the neighbours
    std::vector<int> corrIndex(rc_cloud->points.size());
    std::vector<float> corrDistance(rc_cloud->points.size());

    int num_of_iters = 0;
    //Start iteration
    while (!converged || num_of_iters < MAX_ITER) {
        // Transform the src_cloud using current transformation matrix
        pcl::PointCloud<pcl::PointXYZ> curr_transformedcloud;
        pcl::transformPointCloud(*src_cloud,curr_transformedcloud,transformation,translation);

        // loop through each point in the src point cloud and find its nearest neighbor in tar and store it's information
        for (std::size_t sample = 0; sample < src_cloud->points.size(); sample += 50) {
            // once found just overwrite the value of the point in src
            if (kdtree.nearestKSearch (src_cloud->points[sample], K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 ) {
                    //The new closer point found, store that into the correspondance array
                    corrIndex(sample) = pointIdxK5NNSearch(0);
                    corrDistance(sample) = pointKNNSquaredDistance(0);
            }
        }

        Eigem::JacobiSVD<Eigen::MatrixXd> svd(demean,)

        // check converge
        if (std::all_of(pointKNNSquaredDistance.cbegin(), pointKNNSquaredDistance.cend(), [](int i) { return i < THRESHOLD; })) {
            converged = true;
        }
            
        
        num_of_iters++;   
    }


    // 2. https://github.com/Gregjksmith/Iterative-Closest-Point/blob/master/src/ICP.cpp
        

    return transformation;
}




