// Headers
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

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

class OdoSyncNode
{

private:
	ros::NodeHandle nh_;

	message_filters::Subscriber<nav_msgs::Odometry> odom1_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> odom2_sub_;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> sync_policy;
    typedef message_filters::Synchronizer<sync_policy> Sync;
    boost::shared_ptr<Sync> sync_;
	
	std::ofstream output_sync_file_;

public:
	OdoSyncNode(const ros::NodeHandle &nh);
	~OdoSyncNode();

	void callback( const nav_msgs::OdometryConstPtr& msg_rgb , const nav_msgs::OdometryConstPtr& msg_depth );

private:
	int n_synchro_;
	std::string output_folder_;
	std::string odom1_sub_str_;
	std::string odom2_sub_str_;

	bool paramValidation(std::string &errmsg);

};
