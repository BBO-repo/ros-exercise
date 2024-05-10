#include "OdoSyncNode.h"

OdoSyncNode::OdoSyncNode(const ros::NodeHandle &nh) : nh_{nh}, n_synchro_{0}
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

	// topic subscribed to
	odom1_sub_.subscribe(nh_, odom1_sub_str_, 5);
	odom2_sub_.subscribe(nh_, odom2_sub_str_, 5);

	sync_.reset(new Sync(sync_policy(20), odom1_sub_, odom2_sub_));
	sync_->registerCallback(boost::bind(&OdoSyncNode::callback, this, _1, _2));

	// file stream to log synchronized data
	std::string filename{output_folder_ + "odo_sync.txt"};
	output_sync_file_.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
}

OdoSyncNode::~OdoSyncNode()
{
	output_sync_file_.close();
	ROS_INFO("Synchronized [%d] odometry messages in folder: [%s]", n_synchro_, output_folder_.c_str());
}

void OdoSyncNode::callback(const nav_msgs::OdometryConstPtr &msg_rgb, const nav_msgs::OdometryConstPtr &msg_depth)
{
	++n_synchro_;
	output_sync_file_ << "Print something:" << n_synchro_ << std::endl;
}

bool OdoSyncNode::paramValidation(std::string &errmsg)
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
