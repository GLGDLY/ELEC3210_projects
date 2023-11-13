#include "ekf_slam.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;


EKFSLAM::~EKFSLAM() {}

EKFSLAM::EKFSLAM(ros::NodeHandle &nh):
        nh_(nh) {

//    initialize ros publisher
    lidar_sub = nh_.subscribe("/velodyne_points", 1, &EKFSLAM::cloudHandler, this);
    odom_sub = nh_.subscribe("/odom", 1, &EKFSLAM::odomHandler, this);
    map_cylinder_pub = nh_.advertise<visualization_msgs::MarkerArray>("/map_cylinder", 1);
    obs_cylinder_pub = nh_.advertise<visualization_msgs::MarkerArray>("/obs_cylinder", 1);
    odom_pub = nh_.advertise<nav_msgs::Odometry>("ekf_odom", 1000);
    path_pub = nh_.advertise<nav_msgs::Path>("ekf_path", 1000);
    scan_pub = nh_.advertise<sensor_msgs::PointCloud2>("current_scan", 1);
    map_pub = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);
    laserCloudIn = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    mapCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    extractCylinder = std::make_shared<ExtractCylinder>(nh_);

    globalId = -1;
	/**
	 * TODO: initialize the state vector and covariance matrix
	 */
    // set the initial state and covariance to zero
    mState = Eigen::VectorXd::Zero(3); // x, y, yaw
    mCov = Eigen::MatrixXd::Zero(3, 3); // covariance matrix
    R = Eigen::MatrixXd::Zero(2, 2); // process noise: Rn = cov(n), which n = [v, w]
    R(0, 0) = 0.1; // tune this
    R(1, 1) = 0.2; // tune this
    Q = Eigen::MatrixXd::Zero(2, 2); // measurement noise: Qn = cov(n), which n = [r, phi]

    std::cout << "EKF SLAM initialized" << std::endl;
}

void EKFSLAM::run() {
    ros::Rate rate(1000);
    while (ros::ok()){
        if (cloudQueue.empty() || odomQueue.empty()){
            rate.sleep();
            continue;
        }

        cloudQueueMutex.lock();
        cloudHeader = cloudQueue.front().first;
        laserCloudIn = parseCloud(cloudQueue.front().second);
        cloudQueue.pop();
        cloudQueueMutex.unlock();

        // find the cloest odometry message
        odomMutex.lock();
        auto odomIter = odomQueue.front();
        auto odomPrevIter = odomQueue.front();
        while (!odomQueue.empty() && odomIter != odomQueue.back() && odomIter.first.stamp < cloudHeader.stamp){
            odomPrevIter = odomIter;
            odomIter = odomQueue.front();
            odomQueue.pop();
        }
        odomMutex.unlock();

        if (firstFrame){
            firstFrame = false;
            Twb = Eigen::Matrix4d::Identity();
            cloudHeaderLast = cloudHeader;
            continue;
        }

        auto odomMsg = odomIter == odomQueue.back() ? odomPrevIter : odomIter;
        Eigen::Vector2d ut = Eigen::Vector2d(odomMsg.second->twist.twist.linear.x, odomMsg.second->twist.twist.angular.z);
        double dt = (cloudHeader.stamp - cloudHeaderLast.stamp).toSec();

        timer.tic();
		// Extended Kalman Filter
		// 1. predict
        predictState(mState, mCov, ut, dt);
		// 2. update
        updateMeasurement();
		timer.toc();

		// publish odometry and map
		map_pub_timer.tic();
        accumulateMap();
        publishMsg();
		cloudHeaderLast = cloudHeader;

        rate.sleep();
    }
}

double EKFSLAM::normalizeAngle(double angle){
	if (angle > M_PI){
		angle -= 2 * M_PI;
	} else if (angle < -M_PI){
		angle += 2 * M_PI;
	}
	return angle;
}

Eigen::MatrixXd EKFSLAM::jacobGt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt){
	int num_state = state.rows();
	Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(num_state, num_state);
	/**
	 * TODO: implement the Jacobian Gt
	 */
    // Note: ut = [v, w] -> ut(0) = v
    Eigen::Matrix3d Gt_x = Eigen::Matrix3d::Identity();
    Gt_x(0, 2) = -ut(0) * dt * sin(state(2));
    Gt_x(1, 2) = ut(0) * dt * cos(state(2));
    Gt.block(0, 0, 3, 3) = Gt_x; 

	return Gt;
}

Eigen::MatrixXd EKFSLAM::jacobFt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt){
	int num_state = state.rows();
	Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(num_state, 2);
	/**
	 * TODO: implement the Jacobian Ft
	 */
    Ft(0, 0) = dt * cos(state(2));
    Ft(1, 0) = dt * sin(state(2));
    Ft(2, 1) = dt;

	return Ft;
}

Eigen::MatrixXd EKFSLAM::jacobB(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt){
	int num_state = state.rows();
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_state, 2);
	B(0, 0) = dt * cos(state(2));
	B(1, 0) = dt * sin(state(2));
	B(2, 1) = dt;
	return B;
}

void EKFSLAM::predictState(Eigen::VectorXd& state, Eigen::MatrixXd& cov, Eigen::Vector2d ut, double dt){
	// Note: ut = [v, w]
	state = state + jacobB(state, ut, dt) * ut; // update state
	Eigen::MatrixXd Gt = jacobGt(state, ut, dt);
	Eigen::MatrixXd Ft = jacobFt(state, ut, dt);
	cov = Gt * cov * Gt.transpose() + Ft * R * Ft.transpose(); // update covariance
}

Eigen::Vector2d EKFSLAM::transform(const Eigen::Vector2d& p, const Eigen::Vector3d& x){
	Eigen::Vector2d p_t;
	p_t(0) = p(0) * cos(x(2)) - p(1) * sin(x(2)) + x(0);
	p_t(1) = p(0) * sin(x(2)) + p(1) * cos(x(2)) + x(1);
	return p_t;
}

void EKFSLAM::addNewLandmark(const Eigen::Vector2d& lm, const Eigen::MatrixXd& InitCov){
	// add new landmark to mState and mCov
	/**
	 * TODO: implement the function
	 */
    int num_landmarks = (mState.rows() - 3) / 2;
    Eigen::VectorXd state_new = Eigen::VectorXd::Zero(mState.rows() + 2);
    Eigen::MatrixXd cov_new = Eigen::MatrixXd::Zero(mCov.rows() + 2, mCov.cols() + 2);
    state_new.segment(0, mState.rows()) = mState;
    state_new.segment(mState.rows(), 2) = lm;
    cov_new.block(0, 0, mCov.rows(), mCov.cols()) = mCov;
    cov_new.block(mCov.rows(), mCov.cols(), 2, 2) = InitCov;
    mState = state_new;
    mCov = cov_new;
}

void EKFSLAM::accumulateMap(){

    Eigen::Matrix4d Twb = Pose3DTo6D(mState.segment(0, 3));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laserCloudIn, *transformedCloud, Twb);
    *mapCloud += *transformedCloud;

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(mapCloud);
    voxelSampler.setLeafSize(0.5, 0.5, 0.5);
    voxelSampler.filter(*mapCloud);
}

void EKFSLAM::updateMeasurement(){

    cylinderPoints = extractCylinder->extract(laserCloudIn, cloudHeader); // 2D pole centers in the laser/body frame
    Eigen::Vector3d xwb = mState.block<3, 1>(0, 0); // pose in the world frame
    int num_landmarks = (mState.rows() - 3) / 2; // number of landmarks in the state vector
    int num_obs = cylinderPoints.rows(); // number of observations
    Eigen::VectorXi indices = Eigen::VectorXi::Ones(num_obs) * -1; // indices of landmarks in the state vector
    for (int i = 0; i < num_obs; ++i) {
        Eigen::Vector2d pt_transformed = transform(cylinderPoints.row(i), xwb); // 2D pole center in the world frame
		// Implement the data association here, i.e., find the corresponding landmark for each observation
		/**
		 * TODO: data association
		 *
		 * **/

        // update the indices if the landmark already exists in the state vector
        for (int j = 0; j < num_landmarks; ++j) {
            const Eigen::Vector2d& landmark = mState.block<2, 1>(3 + j * 2, 0);
            if ((pt_transformed - landmark).norm() < 3){
                indices(i) = j;
                break;
            }
        }

        // update the indices if the landmark is new
        // if (indices(i) == -1){
        //     indices(i) = ++globalId;
        //     addNewLandmark(pt_transformed, Q);
        // }
    }
    // std::cout << globalId << std::endl;
    // simulating bearing model
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2 * num_obs);
    for (int i = 0; i < num_obs; ++i) {
        const Eigen::Vector2d& pt = cylinderPoints.row(i);
        z(2 * i, 0) = pt.norm();
        z(2 * i + 1, 0) = atan2(pt(1), pt(0));
    }
    // update the measurement vector
    num_landmarks = (mState.rows() - 3) / 2;
    Eigen::VectorXd _mState = mState;
    for (int i = 0; i < num_obs; ++i) {
        int idx = indices(i);
        if (idx == -1 || idx + 1 > num_landmarks) continue;
        const Eigen::Vector2d& landmark = _mState.block<2, 1>(3 + idx * 2, 0);
		// Implement the measurement update here, i.e., update the state vector and covariance matrix
		/**
		 * TODO: measurement update
		 */
        // FIXME: debug this logic -- update mState and mCov is not working (which is commented now)
        // double q = (landmark(0) - xwb(0)) * (landmark(0) - xwb(0)) + (landmark(1) - xwb(1)) * (landmark(1) - xwb(1));
        // Eigen::Vector2d z_hat = Eigen::Vector2d::Zero(2);
        // z_hat(0) = sqrt(q);
        // z_hat(1) = atan2(landmark(1) - xwb(1), landmark(0) - xwb(0)) - xwb(2);

        // Eigen::Vector2d z_hat = transform(landmark, xwb);
        // Eigen::MatrixXd Kt = mCov * Ht.transpose() * (Ht * mCov * Ht.transpose() + Q).inverse();
        // mState = mState + Kt * (z.segment(2 * i, 2) - z_hat);
        // mState(2) = normalizeAngle(mState(2));
        // mCov = (Eigen::MatrixXd::Identity(mState.rows(), mState.rows()) - Kt * Ht) * mCov;




        // Eigen::Vector2d z_hat = transform(landmark, xwb);
        // Eigen::Vector2d z_diff = z.segment<2>(2 * i) - z_hat;
        // z_diff(1) = normalizeAngle(z_diff(1));
        // Eigen::MatrixXd Ht = Eigen::MatrixXd::Zero(2, mState.rows());
        // Ht.block<2, 2>(0, 0) = -Eigen::Matrix2d::Identity();
        // Ht.block<2, 2>(0, 3 + idx * 2) = Eigen::Matrix2d::Identity();
        // Eigen::MatrixXd Kt = mCov * Ht.transpose() * (Ht * mCov * Ht.transpose() + Q).inverse();
        // mState = mState + Kt * z_diff;
        // mState(2) = normalizeAngle(mState(2));
        // mCov = (Eigen::MatrixXd::Identity(mState.rows(), mState.rows()) - Kt * Ht) * mCov;
               

        //update the state vector
        Eigen::Vector2d z_hat = transform(landmark, xwb);
        Eigen::Vector2d z_diff = z.segment<2>(2 * i) - z_hat;
        z_diff(1) = normalizeAngle(z_diff(1));
        _mState.segment<2>(3 + idx * 2) += mCov.block<2, 2>(3 + idx * 2, 3 + idx * 2).inverse() * mCov.block<2, 2>(3 + idx * 2, 3 + idx * 2) * z_diff;
        _mState(2) = normalizeAngle(_mState(2));
        // std::cout << (_mState.rows() - 3) / 2 << " " << _mState.size() << " ";

        //update the covariance matrix
        //Eigen::MatrixXd Ht = Eigen::MatrixXd::Zero(2, _mState.rows());
        //Ht.block<2, 2>(0, 0) = -Eigen::Matrix2d::Identity();
        //Ht.block<2, 2>(0, 3 + idx * 2) = Eigen::Matrix2d::Identity();
        
        double q = (landmark(0) - xwb(0)) * (landmark(0) - xwb(0)) + (landmark(1) - xwb(1)) * (landmark(1) - xwb(1));
        double Ht_0 = 1/q;
        Eigen::MatrixXd Ht_1 = Eigen::MatrixXd::Zero(2, 5);
        Ht_1(0, 0) = -sqrt(q) * (landmark(0) - xwb(0));
        Ht_1(0, 1) = -sqrt(q) * (landmark(1) - xwb(1));
        Ht_1(0, 3) = sqrt(q) * (landmark(0) - xwb(0));
        Ht_1(0, 4) = sqrt(q) * (landmark(1) - xwb(1));
        Ht_1(1, 0) = (landmark(1) - xwb(1));
        Ht_1(1, 1) = -(landmark(0) - xwb(0));
        Ht_1(1, 2) = -q;
        Ht_1(1, 3) = -(landmark(1) - xwb(1));
        Ht_1(1, 4) = (landmark(0) - xwb(0));
        Eigen::MatrixXd Ht_2 = Eigen::MatrixXd::Zero(5, 3 + 2 * num_landmarks);
        Ht_2.block(0, 0, 2, 3) = Eigen::MatrixXd::Identity(2, 3);
        Ht_2.block(0, 3 + 2 * idx, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
        Eigen::MatrixXd Ht = Ht_0 * Ht_1 * Ht_2;


        Eigen::MatrixXd Kt = mCov * Ht.transpose() * (Ht * mCov * Ht.transpose() + Q).inverse();
        mCov = (Eigen::MatrixXd::Identity(_mState.rows(), _mState.rows()) - Kt * Ht) * mCov;
        // std::cout << mCov.size() << std::endl;

    }

    xwb = mState.block<3, 1>(0, 0); // pose in the world frame
    for (int i = 0; i < num_obs; ++i) {
        Eigen::Vector2d pt_transformed = transform(cylinderPoints.row(i), xwb); // 2D pole center in the world frame
        if (indices(i) == -1){
            indices(i) = ++globalId;
            addNewLandmark(pt_transformed, Q);
        }
    }
} 

void EKFSLAM::publishMsg(){
    // publish map cylinder
    visualization_msgs::MarkerArray markerArray;
    int num_landmarks = (mState.rows() - 3) / 2;
    for (int i = 0; i < num_landmarks; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = cloudHeader.stamp;
        marker.ns = "map_cylinder";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mState(3 + i * 2, 0);
        marker.pose.position.y = mState(3 + i * 2 + 1, 0);
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }
    map_cylinder_pub.publish(markerArray);

    int num_obs = cylinderPoints.rows();
    markerArray.markers.clear();
    for (int i = 0; i < num_obs; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = cloudHeader.stamp;
        marker.ns = "obs_cylinder";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector2d pt = transform(cylinderPoints.row(i).transpose(), mState.segment(0, 3));
        marker.pose.position.x = pt(0);
        marker.pose.position.y = pt(1);
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }
    obs_cylinder_pub.publish(markerArray);

//    publish odom
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.header.stamp = cloudHeader.stamp;
    odom.pose.pose.position.x = mState(0,0);
    odom.pose.pose.position.y = mState(1,0);
    odom.pose.pose.position.z = 0;
    Eigen::Quaterniond q(Eigen::AngleAxisd(mState(2,0), Eigen::Vector3d::UnitZ()));
    q.normalize();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub.publish(odom);

//    publish path
    path.header.frame_id = "map";
    path.header.stamp = cloudHeader.stamp;
    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path.poses.push_back(pose);
    path_pub.publish(path);

////    publish map
    sensor_msgs::PointCloud2 mapMsg;
    pcl::toROSMsg(*mapCloud, mapMsg);
    mapMsg.header.frame_id = "map";
    mapMsg.header.stamp = cloudHeader.stamp;
    map_pub.publish(mapMsg);

////    publish laser
    sensor_msgs::PointCloud2 laserMsg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laserCloudIn, *laserTransformed, Pose3DTo6D(mState.segment(0, 3)).cast<float>());
    pcl::toROSMsg(*laserTransformed, laserMsg);
    laserMsg.header.frame_id = "map";
    laserMsg.header.stamp = cloudHeader.stamp;
    scan_pub.publish(laserMsg);

	map_pub_timer.toc();
    std::cout << "x: " << mState(0,0) << " y: " << mState(1,0) << " theta: " << mState(2,0) * 180 / M_PI << ", time ekf: " << timer.duration_ms() << " ms"
						  << ", map_pub: " << map_pub_timer.duration_ms() << " ms" << std::endl;
}

void EKFSLAM::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    cloudQueueMutex.lock();
    std_msgs::Header cloudHeader = laserCloudMsg->header;
    cloudQueue.push(std::make_pair(cloudHeader, laserCloudMsg));
    cloudQueueMutex.unlock();
}

void EKFSLAM::odomHandler(const nav_msgs::OdometryConstPtr& odomMsg){
    odomMutex.lock();
    std_msgs::Header odomHeader = odomMsg->header;
    odomQueue.push(std::make_pair(odomHeader, odomMsg));
    odomMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EKFSLAM::parseCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *cloudTmp);
    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudTmp, *cloudTmp, indices);
    return cloudTmp;
}

