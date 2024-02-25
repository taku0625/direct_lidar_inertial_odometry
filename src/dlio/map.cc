/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/map.h"

dlio::MapNode::MapNode(ros::NodeHandle node_handle) : nh(node_handle)
{

	this->getParams();

	this->publish_timer = this->nh.createTimer(ros::Duration(this->publish_freq_), &dlio::MapNode::publishTimer, this);

	this->keyframe_sub = this->nh.subscribe("keyframes", 10,
											&dlio::MapNode::callbackKeyframe, this, ros::TransportHints().tcpNoDelay());
	this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 100);
	this->save_pcd_srv = this->nh.advertiseService("save_pcd", &dlio::MapNode::savePcd, this);

	this->dlio_map = pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>());

	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams()
{

	ros::param::param<std::string>("~dlio/odom/odom_frame", this->odom_frame, "odom");
	ros::param::param<double>("~dlio/map/sparse/frequency", this->publish_freq_, 1.0);
	ros::param::param<double>("~dlio/map/sparse/leafSize", this->leaf_size_, 0.5);
	ros::param::param<int>("~dlio/map/preprocessing/feature/cov_points_num", this->cov_points_num_, 10);
	ros::param::param<double>("~dlio/map/preprocessing/feature/min_dist_between_points", this->min_dist_between_points_, 1.5);
	ros::param::param<double>("~dlio/map/preprocessing/feature/min_eigenvalue_ration", this->min_eigenvalue_ration_, 10);
	ros::param::param<double>("~dlio/map/preprocessing/feature/min_neighbor", this->min_neighbor_, 10);


	// Get Node NS and Remove Leading Character
	std::string ns = ros::this_node::getNamespace();
	ns.erase(0, 1);

	// Concatenate Frame Name Strings
	this->odom_frame = ns + "/" + this->odom_frame;
}

void dlio::MapNode::start()
{
}

void dlio::MapNode::publishTimer(const ros::TimerEvent &e)
{

	if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height)
	{
		sensor_msgs::PointCloud2 map_ros;
		pcl::toROSMsg(*this->dlio_map, map_ros);
		map_ros.header.stamp = ros::Time::now();
		map_ros.header.frame_id = this->odom_frame;
		this->map_pub.publish(map_ros);
	}
}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr &keyframe)
{

	// convert scan to pcl format
	pcl::PointCloud<PointType>::Ptr keyframe_pcl =
		pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>());
	pcl::fromROSMsg(*keyframe, *keyframe_pcl);

	// voxel filter
	this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
	this->voxelgrid.setInputCloud(keyframe_pcl);
	this->voxelgrid.filter(*keyframe_pcl);

	// save filtered keyframe to map for rviz
	*this->dlio_map += *keyframe_pcl;

	auto output_cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
	this->filter(this->dlio_map, output_cloud);
	this->dlio_map = output_cloud;
}

bool dlio::MapNode::savePcd(direct_lidar_inertial_odometry::save_pcd::Request &req,
							direct_lidar_inertial_odometry::save_pcd::Response &res)
{

	pcl::PointCloud<PointType>::Ptr m =
		pcl::PointCloud<PointType>::Ptr(boost::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

	float leaf_size = req.leaf_size;
	std::string p = req.save_path;

	std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
			  << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... ";
	std::cout.flush();

	// voxelize map
	pcl::VoxelGrid<PointType> vg;
	vg.setLeafSize(leaf_size, leaf_size, leaf_size);
	vg.setInputCloud(m);
	vg.filter(*m);

	// save map
	int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
	res.success = ret == 0;

	if (res.success)
	{
		std::cout << "done" << std::endl;
	}
	else
	{
		std::cout << "failed" << std::endl;
	}

	return res.success;
}


void dlio::MapNode::filter(const pcl::PointCloud<PointType>::Ptr &input_cloud,
						  pcl::PointCloud<PointType>::Ptr &output_cloud)
{
    auto input_filtered_cloud_1 = pcl::make_shared<pcl::PointCloud<PointType>>();
    auto input_filtered_cloud_2 = pcl::make_shared<pcl::PointCloud<PointType>>();
    auto covs_1 = std::make_shared<nano_gicp::CovarianceList>();
    auto covs_2 = std::make_shared<nano_gicp::CovarianceList>();

	*input_filtered_cloud_1 = *input_cloud;
    calculateCovs(input_filtered_cloud_1, covs_1);

    // filterOnce(input_filtered_cloud_1, output_cloud, covs_1, output_covs, 1);

    while (true)
    {
        filterOnce(input_filtered_cloud_1, input_filtered_cloud_2, covs_1, covs_2);

        if (input_filtered_cloud_1->size() == input_filtered_cloud_2->size())
        {
            filterOnce(input_filtered_cloud_2, input_filtered_cloud_1, covs_2, covs_1);
            // filterOnce(input_filtered_cloud_2, input_filtered_cloud_1, covs_2, covs_1, 1);

            output_cloud = input_filtered_cloud_1;

            return;
        }

        filterOnce(input_filtered_cloud_2, input_filtered_cloud_1, covs_2, covs_1);

        if (input_filtered_cloud_1->size() == input_filtered_cloud_2->size())
        {
            filterOnce(input_filtered_cloud_1, input_filtered_cloud_2, covs_1, covs_2);
            // filterOnce(input_filtered_cloud_1, input_filtered_cloud_2, covs_1, covs_2, 1);

            output_cloud = input_filtered_cloud_2;

            return;
        }
    }
}

void dlio::MapNode::calculateCovs(const pcl::PointCloud<PointType>::Ptr &input_cloud,
                                 std::shared_ptr<nano_gicp::CovarianceList> &output_covs,
                                 const int search_mode)
{
    output_covs.reset(new nano_gicp::CovarianceList(input_cloud->size()));
    kdtree_.setInputCloud(input_cloud);

#pragma omp parallel for num_threads(this->num_threads_) schedule(guided, 8)
    for (int i = 0; i < input_cloud->size(); ++i)
    {
        output_covs->at(i) = calculateCov(input_cloud->at(i), search_mode);
    }
}

Eigen::Matrix4d dlio::MapNode::calculateCov(const PointType &point, const int search_mode)
{
    std::vector<int> k_indices;
    std::vector<float> k_sq_distances;
    if (search_mode == 0)
    {
        kdtree_.nearestKSearch(point, cov_points_num_, k_indices, k_sq_distances);
    }
    else
    {
        // kdtree_.radiusSearch(point, COV_POINTS_RADIUS_, k_indices, k_sq_distances);
    }

    int num_neighbors = k_indices.size();

    if (num_neighbors < cov_points_num_ || k_sq_distances.back() > min_dist_between_points_ || k_sq_distances[1] > min_neighbor_)
    {
        return Eigen::Matrix4d::Zero();
    }

    Eigen::Matrix<double, 4, -1> neighbors(4, num_neighbors);
    for (int j = 0; j < num_neighbors; ++j)
    {
        neighbors.col(j) = kdtree_.getInputCloud()->at(k_indices[j]).getVector4fMap().template cast<double>();
    }

    neighbors.colwise() -= neighbors.rowwise().mean().eval();
    Eigen::Matrix4d cov = neighbors * neighbors.transpose() / num_neighbors;

    return cov;
}

void dlio::MapNode::filterOnce(const pcl::PointCloud<PointType>::Ptr &input_cloud,
                              pcl::PointCloud<PointType>::Ptr &output_cloud,
                              const std::shared_ptr<nano_gicp::CovarianceList> &input_covs,
                              std::shared_ptr<nano_gicp::CovarianceList> &output_covs,
                              const int search_mode)
{
    output_cloud.reset(new pcl::PointCloud<PointType>());
    output_covs.reset(new nano_gicp::CovarianceList());
    output_cloud->points.reserve(input_cloud->size());
    output_covs->reserve(input_covs->size());

    kdtree_.setInputCloud(input_cloud);

    std::vector<bool> valid_point_flags(input_cloud->size());
    auto tmp_output_covs = std::make_shared<nano_gicp::CovarianceList>(input_covs->size());

#pragma omp parallel for num_threads(this->num_threads_) schedule(guided, 8)
    for (int i = 0; i < input_cloud->size(); ++i)
    {
        Eigen::Matrix4d cov = calculateCov(input_cloud->at(i), search_mode);
        tmp_output_covs->at(i) = cov;

        if (!isValidPoint(cov.block<3, 3>(0, 0), search_mode))
        {
            valid_point_flags[i] = false;
        }
        else
        {
            valid_point_flags[i] = true;
        }
    }

    // output_cloudとoutput_covsで並びの同期をはかるため
    for (int i = 0; i < valid_point_flags.size(); ++i)
    {
        if (valid_point_flags[i])
        {
            output_cloud->push_back(input_cloud->at(i));
            // output_covs->push_back(tmp_output_covs->at(i));
            output_covs->push_back(input_covs->at(i));
        }
    }

    output_cloud->points.shrink_to_fit();
    output_covs->shrink_to_fit();
}

Eigen::Matrix4d dlio::MapNode::regularizateCov(const Eigen::Matrix4d &cov)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d values(1, 1, 1e-3);

    Eigen::Matrix4d cov_regularizated = Eigen::Matrix4d::Zero();
    cov_regularizated.template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();

    return cov_regularizated;
}

bool dlio::MapNode::isValidPoint(const Eigen::Matrix3d &cov, const int search_mode)
{
    if (cov.isZero())
    {
        return false;
    }

    if (search_mode == 0)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        if (solver.info() != Eigen::Success)
        {
            abort();
        }

        double x = solver.eigenvalues().x();
        double y = solver.eigenvalues().y();
        double z = solver.eigenvalues().z();

        if ((z / x) < min_eigenvalue_ration_)
        {
            return false;
        }
    }

    return true;
}