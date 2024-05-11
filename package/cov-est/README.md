# odo-sync package
Quick package to perform to estimator error covariance between two odometry topics within ROS framework.<br>
This package synchronizes two odometry topics, one of them is considered as being the ground truth. The error covariance between the two odometries is computed using the sample covariance estimator and is published