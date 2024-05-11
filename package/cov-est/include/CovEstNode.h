// Headers
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <err.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <ctime>
#include <cmath>

class CovEstNode
{

private:
	ros::NodeHandle nh_;

	message_filters::Subscriber<nav_msgs::Odometry> odom1_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom2_sub_;
	ros::Publisher cov_x_pub_; // publisher for x position covariance
	ros::Publisher cov_y_pub_; // publisher for y position covariance
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> sync_policy;
    typedef message_filters::Synchronizer<sync_policy> Sync;
    boost::shared_ptr<Sync> sync_;
	
	std::ofstream output_sync_file_;

public:
	CovEstNode(const ros::NodeHandle &nh);
	~CovEstNode();

	void callback( const nav_msgs::OdometryConstPtr& odom_1_msg , const nav_msgs::OdometryConstPtr& odom_2_msg );

private:
	int n_synchro_{0};
	std::string output_folder_;
	std::string odom1_sub_str_;
	std::string odom2_sub_str_;
	std::string cov_x_pub_str_{"/cov_est/covariance_x"};
	std::string cov_y_pub_str_{"/cov_est/covariance_y"};

	// computation data
	float sigma_x_n_;  // sample covariance estimator - x position
	float sigma_y_n_;  // sample covariance estimator - y position
	float mu_x_n_;     // sample mean estimator - x position
	float mu_y_n_;     // sample mean estimator - y position
	float sigma_0_{0}; // init value of sample covariance estimator
	float mu_0_{0};    // init value of sample mean estimator
	float x_gt_0_{0};  // initial ground truth x position
	float y_gt_0_{0};  // initial ground truth y position

	bool paramValidation(std::string &errmsg);

};
