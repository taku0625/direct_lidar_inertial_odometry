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

#include "dlio/dlio.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

class dlio::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  void start();

private:

  void getParams();

  void publishTimer(const ros::TimerEvent& e);
  void callbackCurrentCloud(const sensor_msgs::PointCloud2ConstPtr& current);

  bool savePcd(direct_lidar_inertial_odometry::save_pcd::Request& req,
               direct_lidar_inertial_odometry::save_pcd::Response& res);

  ros::NodeHandle nh;
  ros::Timer publish_timer;

  ros::Subscriber current_sub;
  ros::Publisher current_map_pub;
  ros::Publisher feature_map_pub;

  ros::ServiceServer save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr current_map;
  pcl::PointCloud<PointType>::Ptr all_feature_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  double publish_freq_;
  double leaf_size_;


  void filter(const pcl::PointCloud<PointType>::Ptr &input_cloud,
				pcl::PointCloud<PointType>::Ptr &output_cloud);

	void calculateCovs(const pcl::PointCloud<PointType>::Ptr &input_cloud,
                       std::shared_ptr<nano_gicp::CovarianceList> &output_covs,
                       const int search_mode = 0);

	Eigen::Matrix4d regularizateCov(const Eigen::Matrix4d &cov);

	Eigen::Matrix4d calculateCov(const PointType &point, const int search_mode = 0);

	void filterOnce(const pcl::PointCloud<PointType>::Ptr &input_cloud,
						pcl::PointCloud<PointType>::Ptr &output_cloud,
						const std::shared_ptr<nano_gicp::CovarianceList> &input_covs,
						std::shared_ptr<nano_gicp::CovarianceList> &output_covs,
						const int search_mode = 0);

	bool isValidPoint(const Eigen::Matrix3d &cov, const int search_mode = 0);

	nanoflann::KdTreeFLANN<PointType> kdtree_;
	const double COV_POINTS_RADIUS_ = 0.5;
	int cov_points_num_;
	double min_dist_between_points_;
	double min_eigenvalue_ration_;
	double min_neighbor_;

	int num_threads_ = 12;
  int cloud_num_ = 0;

  double extraction_rate_ = 0.0;
  int extraction_num_ = 0;

  geometry_msgs::PoseArray::Ptr kf_traj_ = nullptr;
  bool detected_loop_ = false;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> kf_clouds_;
  std::vector<Eigen::Matrix4f> kf_transform_;
  
};
