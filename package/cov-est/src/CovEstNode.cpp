#include "CovEstNode.h"

CovEstNode::CovEstNode(const ros::NodeHandle &nh) : nh_{nh}, n_synchro_{0}
{

	ROS_INFO("Starting odometry synchronizer node");

	// check parameters are valid
	std::string errmsg{""};
	if (!paramValidation(errmsg))
	{
		ROS_ERROR("%s", errmsg.c_str());
		ros::shutdown();
		exit(1);
	}

	ROS_INFO("output folder: %s", output_folder_.c_str());
	ROS_INFO("first odom topic: [%s]", odom1_sub_str_.c_str());
	ROS_INFO("second topic: [%s]", odom2_sub_str_.c_str());

    // covariance publishers
	cov_x_pub_ = nh_.advertise<std_msgs::Float64>(cov_x_pub_str_, 10);
	cov_y_pub_ = nh_.advertise<std_msgs::Float64>(cov_y_pub_str_, 10);

	// topic subscribed to
	odom1_sub_.subscribe(nh_, odom1_sub_str_, 5);
	odom2_sub_.subscribe(nh_, odom2_sub_str_, 5);

	sync_.reset(new Sync(sync_policy(20), odom1_sub_, odom2_sub_));
	sync_->registerCallback(boost::bind(&CovEstNode::callback, this, _1, _2));

	// file stream to log synchronized data
	std::string filename{output_folder_ + "cov_est.txt"};
	output_sync_file_.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
}

CovEstNode::~CovEstNode()
{
	output_sync_file_.close();
	ROS_INFO("Synchronized [%d] odometry messages in folder: [%s]", n_synchro_, output_folder_.c_str());
}

void CovEstNode::callback(const nav_msgs::OdometryConstPtr &odom_1_msg, const nav_msgs::OdometryConstPtr &odom_2_msg)
{
	// write the header if first line
	if(!n_synchro_)
	{
		output_sync_file_ << "time,";
		output_sync_file_ << odom1_sub_str_ << ".x," << odom1_sub_str_ << ".y," << odom1_sub_str_ << ".z,"; 
		output_sync_file_ << odom2_sub_str_ << ".x," << odom2_sub_str_ << ".y," << odom2_sub_str_ << ".z,"; 
		// also record sample covariance and mean estimation
		output_sync_file_ << "sigma_x_n,sigma_y_n,mu_x_n,mu_y_n\n";
	}

    // retrieve positions
	float x_gt_n, y_gt_n, x_lo_n, y_lo_n;
	x_gt_n = odom_1_msg->pose.pose.position.x; // ground truth - x position
	y_gt_n = odom_1_msg->pose.pose.position.y; // ground truth - y position
	x_lo_n = odom_2_msg->pose.pose.position.x; // odometry - x position
	y_lo_n = odom_2_msg->pose.pose.position.y; // odometry - y position

	// positionning error (caution!! supposing both frame are aligned at beginning)
	float err_x, err_y;
	err_x = x_lo_n - (x_gt_n - x_gt_0_);
	err_y = y_lo_n - (y_gt_n - y_gt_0_);

	// update synchro iteration
	++n_synchro_;

	// compute covariance and mean sample estimation
	// see detail implementation there: https://math.stackexchange.com/questions/374881/recursive-formula-for-variance/4741037#4741037
	if(n_synchro_ == 1) // covariance initialisation
	{
		x_gt_0_ = x_gt_n;
		y_gt_0_ = y_gt_n;
		mu_x_n_ = err_x;
		mu_y_n_ = err_y;
		sigma_x_n_ = sigma_0_;
		sigma_y_n_ = sigma_0_;
	}
	else
	{
		// cast iteration integer to float to avoid bad surprise!
		float n_f = static_cast<float>(n_synchro_);

		// update sample covariance estimator
		sigma_x_n_ = ( (n_f-2.0)/(n_f-1.0) )*sigma_x_n_ + (1.0/n_f)*(err_x - mu_x_n_)*(err_x - mu_x_n_);
		sigma_y_n_ = ( (n_f-2.0)/(n_f-1.0) )*sigma_y_n_ + (1.0/n_f)*(err_y - mu_x_n_)*(err_y - mu_x_n_);

		// update sample mean estimator
		mu_x_n_ = (1.0/n_f)*(err_x) + ( (n_f-1.0)/n_f )*(mu_x_n_);
		mu_y_n_ = (1.0/n_f)*(err_y) + ( (n_f-1.0)/n_f )*(mu_y_n_);
	}

	output_sync_file_ << odom_1_msg->header.stamp << ",";
	output_sync_file_ << odom_1_msg->pose.pose.position.x << "," << odom_1_msg->pose.pose.position.y << "," << odom_1_msg->pose.pose.position.z << ","; 
	output_sync_file_ << odom_2_msg->pose.pose.position.x << "," << odom_2_msg->pose.pose.position.y << "," << odom_2_msg->pose.pose.position.z << ","; 
	output_sync_file_ << sigma_x_n_ << "," << sigma_y_n_ << "," << mu_x_n_ << "," << mu_y_n_ << "\n"; 

    // publish covariances
	std_msgs::Float64 cov_x_msg;
	cov_x_msg.data = sigma_x_n_;
    cov_x_pub_.publish(cov_x_msg);

	std_msgs::Float64 cov_y_msg;
	cov_y_msg.data = sigma_y_n_;
    cov_y_pub_.publish(cov_y_msg);
}

bool CovEstNode::paramValidation(std::string &errmsg)
{
	bool status = true;

	// check parameters output_folder
	if (!nh_.getParam("output_folder", output_folder_) || output_folder_.empty())
	{
		errmsg += "Could not find valid parameter for [output_folder]\n";
		status = false;
	}
	// check parameters odom1_sub_topic
	if (!nh_.getParam("odom1_sub_topic", odom1_sub_str_))
	{
		errmsg += "Could not find parameter [odom1_sub_topic]\n";
		status = false;
	}
	// check parameters odom2_sub_topic
	if (!nh_.getParam("odom2_sub_topic", odom2_sub_str_))
	{
		errmsg += "Could not find parameter [odom2_sub_topic]\n";
		status = false;
	}

	// complete error message if required
	if (!status)
	{
		errmsg = "Some parameters are missing, odometry synchronzer node will stop!\n" + errmsg;
	}
	return status;
}
