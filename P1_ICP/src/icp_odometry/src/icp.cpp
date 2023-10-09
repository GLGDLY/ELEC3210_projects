/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"

#include "parameters.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>


Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
								 pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(tar_cloud);

	Eigen::Matrix4d transformation = init_guess;
	Eigen::Matrix4d prev_transformation = Eigen::Matrix4d::Identity();

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*src_cloud, *transformed_cloud, transformation);

	uint32_t iter = 0;

	while (iter < params::max_iterations) {
		try {
			pcl::PointCloud<pcl::PointXYZ>::Ptr correspondences(new pcl::PointCloud<pcl::PointXYZ>);
			std::vector<int> correspondences_indices;
			std::vector<float> correspondences_distances;

			for (int i = 0; i < transformed_cloud->points.size(); i++) {
				pcl::PointXYZ searchPoint = transformed_cloud->points[i];
				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);
				kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
				correspondences->points.push_back(tar_cloud->points[pointIdxNKNSearch[0]]);
				correspondences_indices.push_back(pointIdxNKNSearch[0]);
				correspondences_distances.push_back(pointNKNSquaredDistance[0]);
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr correspondences_filtered(new pcl::PointCloud<pcl::PointXYZ>);
			std::vector<int> correspondences_filtered_indices;
			std::vector<float> correspondences_filtered_distances;

			for (int i = 0; i < correspondences->points.size(); i++) {
				if (correspondences_distances[i] < params::max_distance) {
					correspondences_filtered->points.push_back(correspondences->points[i]);
					correspondences_filtered_indices.push_back(correspondences_indices[i]);
					correspondences_filtered_distances.push_back(correspondences_distances[i]);
				}
			}

			Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, correspondences_filtered->points.size());
			Eigen::Matrix<double, 3, Eigen::Dynamic> tar(3, correspondences_filtered->points.size());

			for (int i = 0; i < correspondences_filtered->points.size(); i++) {
				src(0, i) = transformed_cloud->points[i].x;
				src(1, i) = transformed_cloud->points[i].y;
				src(2, i) = transformed_cloud->points[i].z;
				tar(0, i) = correspondences_filtered->points[i].x;
				tar(1, i) = correspondences_filtered->points[i].y;
				tar(2, i) = correspondences_filtered->points[i].z;
			}

			Eigen::Matrix<double, 3, 1> src_mean = src.rowwise().mean();
			Eigen::Matrix<double, 3, 1> tar_mean = tar.rowwise().mean();

			for (int i = 0; i < correspondences_filtered->points.size(); i++) {
				src.col(i) -= src_mean;
				tar.col(i) -= tar_mean;
			}

			Eigen::Matrix3d H = src * tar.transpose();

			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3d U = svd.matrixU();
			Eigen::Matrix3d V = svd.matrixV();

			Eigen::Matrix3d Rotation = V * U.transpose();

			if (Rotation.determinant() < 0) {
				for (int i = 0; i < 3; i++) {
					U(i, 2) *= -1;
				}
				Rotation = V * U.transpose();
			}

			Eigen::Vector3d Translation = tar_mean - Rotation * src_mean;

			Eigen::Matrix4d update_transformation = Eigen::Matrix4d::Identity();
			update_transformation.block<3, 3>(0, 0) = Rotation;
			update_transformation.block<3, 1>(0, 3) = Translation;

			transformation = update_transformation * transformation;

			if ((transformation - prev_transformation).norm() < params::max_distance) {
				break;
			} else {
				prev_transformation = transformation;
			}

			pcl::transformPointCloud(*src_cloud, *transformed_cloud, transformation);

			iter++;
		} catch (...) {
			break;
		}
	}

	return transformation;
}
