#include "utility.h"
#include "lio_sam_lo/cloud_info.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
* A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;


class mapOptimization : public ParamServer
{

public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOdomAftMappedROS;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;

    ros::Subscriber subLaserCloudInfo;
    ros::Subscriber subGPS;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam_lo::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    //bb: addition
    std::mutex mtxWin;
    std::vector<PointType> win_cloudKeyPoses3D;
    std::vector<PointTypePose> win_cloudKeyPoses6D;

    std::vector<pcl::PointCloud<PointType>::Ptr> win_cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> win_surfCloudKeyFrames;
    //bb: addition

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    //pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    //pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::PointCloud<PointType>::Ptr latestKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryKeyFrameCloud;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLaserInfoStamp;
    double timeLaserCloudInfoLast;

    float transformTobeMapped[6];

    std::mutex mtx;

    double timeLastProcessing = -1;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    int imuPreintegrationResetId = 0;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;

    //bb: addition
    size_t winSize = 30;
    pcl::PointCloud<PointType>::Ptr globalMapCloud;         // point cloud of the global map
    pcl::PointCloud<PointType>::Ptr globalMapCloudDS;       // down sampled point cloud of the global map
    pcl::PointCloud<PointType>::Ptr cloudScanForInitialize; // point cloud for initialization

    ros::Publisher pubLaserCloudInWorld;
    ros::Publisher pubGlobalMapWorld;

    float transformInTheWorld[6] = {}; // the pose in the prebuilt map
    float tranformOdomToWorld[6] = {};
    int frameNum = 1;
    std::mutex mtxtranformOdomToWorld;
    std::mutex mtx_general;

    enum InitializedFlag
    {
        NonInitialized,
        Initializing,
        Initialized
    };
    InitializedFlag initializedFlag;

    geometry_msgs::PoseStamped poseOdomToMap;
    ros::Publisher pubOdomToMapPose;
    //bb: addition

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/map_global", 1);
        pubOdomAftMappedROS = nh.advertise<nav_msgs::Odometry> ("lio_sam_lo/mapping/odometry", 1);
        pubPath = nh.advertise<nav_msgs::Path>("lio_sam_lo/mapping/path", 1);

        subLaserCloudInfo = nh.subscribe<lio_sam_lo::cloud_info>("lio_sam_lo/feature/cloud_info", 10, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        //bb: addition
        pubGlobalMapWorld = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/cloud_map_map",1);
        pubLaserCloudInWorld = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/lasercloud_in_world", 1);
        pubOdomToMapPose = nh.advertise<geometry_msgs::PoseStamped>("lio_sam_lo/mapping/pose_odomTo_map", 1);
        //bb: addition

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/icp_loop_closure_corrected_cloud", 1);

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_lo/mapping/cloud_registered_raw", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        allocateMemory();
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        //kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        //kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        latestKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistoryKeyFrameCloud.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP.setZero();

        //bb: addition
        globalMapCloud.reset(new pcl::PointCloud<PointType>());//bb: addition
	    globalMapCloudDS.reset(new pcl::PointCloud<PointType>());//bb: addition
        cloudScanForInitialize.reset(new pcl::PointCloud<PointType>());
        initializedFlag = NonInitialized;
        loadGlobalMapCloud();
        //bb: addition
    }

    void laserCloudInfoHandler(const lio_sam_lo::cloud_infoConstPtr& msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserCloudInfoLast = msgIn->header.stamp.toSec();

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        //bb: addition
        // use the first scan to localize robot in the generated map then let LIO-SAM go on
         if(initializedFlag == NonInitialized || initializedFlag == Initializing)
         {
            if(cloudScanForInitialize->points.size() == 0)
            {
                downsampleCurrentScan();
                {
                    std::lock_guard<std::mutex> mLock(mtx_general);
                    *cloudScanForInitialize += *laserCloudCornerLastDS;
                    *cloudScanForInitialize += *laserCloudSurfLastDS;
                }
		        laserCloudCornerLastDS->clear();
		        laserCloudSurfLastDS->clear();
		        laserCloudCornerLastDSNum = 0;
		        laserCloudSurfLastDSNum = 0;
                
                transformTobeMapped[0] = cloudInfo.imuRollInit;
                transformTobeMapped[1] = cloudInfo.imuPitchInit;
                transformTobeMapped[2] = cloudInfo.imuYawInit;
                if (!useImuHeadingInitialization)
                {
                    //if not use the heading of init_IMU as Initialization
                    transformTobeMapped[2] = 0;
                }
             }
		
             return;
         }

        frameNum++;
        //bb: addition

        std::lock_guard<std::mutex> lock(mtx);

        if (timeLaserCloudInfoLast - timeLastProcessing >= mappingProcessInterval) {//bb:control the rate of mapping process

            timeLastProcessing = timeLaserCloudInfoLast;

            updateInitialGuess();//bb: update initial value for states

            extractSurroundingKeyFrames();//bb:

            downsampleCurrentScan();//bb:down sample the current corner points and surface points

            scan2MapOptimization();//bb: calculate the tranformtion using lidar measurement with the Imu preintegration as initial values
                                    //and then interpolate roll and pitch angle using IMU measurement and above measurement

            saveKeyFramesAndFactor();//bb: save corner cloud and surface cloud of this scan, and add odom and GPS factors

            //correctPoses();

            publishOdometry();

            publishFrames();
        }
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        for (int i = 0; i < cloudSize; ++i){

            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    void updateInitialGuess()
    {
        static Eigen::Affine3f lastImuTransformation;//bb: note that this is static type
        // initialization
        if (cloudKeyPoses3D->points.empty())//bb: there is no key pose
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)//bb: if not use the heading of init_IMU as Initialization
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        if (cloudInfo.odomAvailable == true && cloudInfo.imuPreintegrationResetId == imuPreintegrationResetId)
        { 
            transformTobeMapped[0] = cloudInfo.initialGuessRoll;
            transformTobeMapped[1] = cloudInfo.initialGuessPitch;
            transformTobeMapped[2] = cloudInfo.initialGuessYaw;

            transformTobeMapped[3] = cloudInfo.initialGuessX;
            transformTobeMapped[4] = cloudInfo.initialGuessY;
            transformTobeMapped[5] = cloudInfo.initialGuessZ;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }



        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;//bb: the transform of IMU between two scans

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    void extractForLoopClosure()
    {
        //bb: addition
        //in this place the maximum of numPoses is winSize
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = win_cloudKeyPoses3D.size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            cloudToExtract->push_back(win_cloudKeyPoses3D[i]);
        }
        extractCloud(cloudToExtract);
        //bb: addition
    }

    //bb: extract the nearby Map points
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        std::vector<pcl::PointCloud<PointType>> laserCloudCornerSurroundingVec;
        std::vector<pcl::PointCloud<PointType>> laserCloudSurfSurroundingVec;

        laserCloudCornerSurroundingVec.resize(cloudToExtract->size());
        laserCloudSurfSurroundingVec.resize(cloudToExtract->size());

        // extract surrounding map
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            // bb: addition
            PointTypePose thisPose6D;
            thisPose6D = win_cloudKeyPoses6D[i];
            laserCloudCornerSurroundingVec[i]  = *transformPointCloud(win_cornerCloudKeyFrames[i],  &thisPose6D);
            laserCloudSurfSurroundingVec[i]    = *transformPointCloud(win_surfCloudKeyFrames[i],    &thisPose6D);
            // bb: addition
        }

        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            *laserCloudCornerFromMap += laserCloudCornerSurroundingVec[i];
            *laserCloudSurfFromMap   += laserCloudSurfSurroundingVec[i];
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
        {
            return;
        }
        // bb: addition
        extractForLoopClosure();
        // bb: addition
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        //bb: for every corner point
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            //bb: calculate its location in the map using the prediction pose
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                //bb: the average coordinate of the most nearest points
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();
                //bb: calculate some coeff and judge whether tho point is valid corner point
                cornerOptimization();
                //bb: calculate some coeff and judge whether tho point is valid surface point
                surfOptimization();

                combineOptimizationCoeffs();

                //bb: the true iteration steps, calculate the transform
                if (LMOptimization(iterCount) == true)
                    break;              
            }
            //bb: interpolate the roll and pitch angle using the IMU measurement and Lidar calculation
            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }
    //bb: interpolate the roll and pitch angle using the IMU measurement and Lidar calculation
    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = 0.01;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                //bb: interpolate between Imu roll measurement and angle from lidar calculation
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                //bb: interpolate between Imu roll measurement and angle from lidar calculation
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        //bb: the first key pose
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            //bb: add constraint between current pose and previous pose
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }


    void saveKeyFramesAndFactor()
    {
        //bb: check if should generate key pose
        if (saveFrame() == false)
            return;

        // odom factor
        //bb: add odom factor in the graph
        addOdomFactor();

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

        //bb:cloudKeyPoses3D can be used to calculate the nearest key frames
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserCloudInfoLast;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        //bb:
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        //bb: addition
        {
            std::lock_guard<std::mutex> mLock(mtxWin);
            win_cloudKeyPoses3D.push_back(thisPose3D);
            win_cloudKeyPoses6D.push_back(thisPose6D);
            if(win_cloudKeyPoses3D.size() > winSize)
            {
                win_cloudKeyPoses3D.erase(win_cloudKeyPoses3D.begin());
                win_cloudKeyPoses6D.erase(win_cloudKeyPoses6D.begin());
            }
            win_cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
            win_surfCloudKeyFrames.push_back(thisSurfKeyFrame);
            if(win_cornerCloudKeyFrames.size() > winSize)
            {
                win_cornerCloudKeyFrames.erase(win_cornerCloudKeyFrames.begin());
                win_surfCloudKeyFrames.erase(win_surfCloudKeyFrames.begin());
            }
        }
        //bb: addition

        // save path for visualization
        updatePath(thisPose6D);
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = "odom";
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        laserOdometryROS.pose.covariance[0] = double(imuPreintegrationResetId);
        pubOdomAftMappedROS.publish(laserOdometryROS);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, "odom");
        // Publish surrounding key frames
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, "odom");
        // publish registered key frame
        //bb: feature points
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, "odom");
        }
        //bb: addition
        if(pubLaserCloudInWorld.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudInBase(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cloudOutInWorld(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
            Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
            PointTypePose pose_Odom_Map;
            {
                std::lock_guard<std::mutex> mLock(mtxtranformOdomToWorld);
                pose_Odom_Map = trans2PointTypePose(tranformOdomToWorld);
            }
            Eigen::Affine3f T_pose_Odom_Map = pclPointToAffine3f(pose_Odom_Map);

            Eigen::Affine3f T_poseInMap = T_pose_Odom_Map * T_thisPose6DInOdom;
            *cloudInBase += *laserCloudCornerLastDS;
            *cloudInBase += *laserCloudSurfLastDS;
            pcl::transformPointCloud(*cloudInBase, *cloudOutInWorld, T_poseInMap.matrix());
            publishCloud(&pubLaserCloudInWorld, cloudOutInWorld, timeLaserInfoStamp, "map");
        }
        //bb: addition
        // publish registered high-res raw cloud
        //bb: whole point_cloud of the scan
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, "odom");
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = "odom";
            pubPath.publish(globalPath);
        }
    }

    //bb: addition
    void loadGlobalMapCloud()
    {
        pcl::io::loadPCDFile("/workspace/data/map/GlobalMap.pcd", *globalMapCloud);

        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(globalMapCloud);
        downSizeFilterICP.filter(*cloud_temp);
        *globalMapCloudDS = *cloud_temp;

        std::cout << "global map cloud size: " << globalMapCloud->points.size() << std::endl;
        std::cout << "down sampled global map cloud size: " << globalMapCloudDS->points.size() << std::endl;
    }

    void localizeOnlyThread()
    {
        while (ros::ok())
        {
            //avoid ICP using the same initial guess for many times
            if(initializedFlag == NonInitialized)
            {
                InitialICPLocalize();
            }
            else if(initializedFlag == Initializing)
            {
                std::cout << "Offer A New Guess Please " << std::endl;//do nothing, wait for a new initial guess
                ros::Duration(1.0).sleep();
            }
            else
            {
		        ros::Duration(10.0).sleep();
                GlobalICPScanMatch();
            }
        }
    }

    void InitialICPLocalize()
    {
        pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());

        {
            std::lock_guard<std::mutex> mLock(mtx_general);
            *laserCloudIn += *cloudScanForInitialize;
        }

        if(laserCloudIn->points.size() == 0)
        {
            return;
        }
        std::cout << "the size of incoming lasercloud: " << laserCloudIn->points.size() << std::endl;

        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);


        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        ndt.setInputSource(laserCloudIn);
        ndt.setInputTarget(globalMapCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());

        PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
        Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
        ndt.align(*unused_result_0, T_thisPose6DInWorld.matrix());


        //use the outcome of ndt as the initial guess for ICP
        icp.setInputSource(laserCloudIn);
        icp.setInputTarget(globalMapCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result, ndt.getFinalTransformation());
        std::cout << "the pose before initializing is: x" << transformInTheWorld[3] << " y" << transformInTheWorld[4]
                  << " z" << transformInTheWorld[5] <<std::endl;
        std::cout << "the pose in odom before initializing is: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                  << " z" << tranformOdomToWorld[5] <<std::endl;
        std::cout << "the icp score in initializing process is: " << icp.getFitnessScore() << std::endl;
        std::cout << "the pose after initializing process is: "<< icp.getFinalTransformation() << std::endl;

        PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
        std::cout<< "transformTobeMapped X_Y_Z: " << transformTobeMapped[3] << " " << transformTobeMapped[4] << " " << transformTobeMapped[5] << std::endl;
        Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

        Eigen::Affine3f T_thisPose6DInMap;
        T_thisPose6DInMap = icp.getFinalTransformation();
        float x_g, y_g, z_g, R_g, P_g, Y_g;
        pcl::getTranslationAndEulerAngles (T_thisPose6DInMap, x_g, y_g, z_g, R_g, P_g, Y_g);
        transformInTheWorld[0] = R_g;
        transformInTheWorld[1] = P_g;
        transformInTheWorld[2] = Y_g;
        transformInTheWorld[3] = x_g;
        transformInTheWorld[4] = y_g;
        transformInTheWorld[5] = z_g;


        Eigen::Affine3f transOdomToMap = T_thisPose6DInMap * T_thisPose6DInOdom.inverse();
        float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
        pcl::getTranslationAndEulerAngles (transOdomToMap, deltax, deltay, deltaz, deltaR, deltaP, deltaY);

        {
            std::lock_guard<std::mutex> mLock(mtxtranformOdomToWorld);
            //renew tranformOdomToWorld
            tranformOdomToWorld[0] = deltaR;
            tranformOdomToWorld[1] = deltaP;
            tranformOdomToWorld[2] = deltaY;
            tranformOdomToWorld[3] = deltax;
            tranformOdomToWorld[4] = deltay;
            tranformOdomToWorld[5] = deltaz;
        }
        std::cout << "the pose of odom relative to Map: x" << tranformOdomToWorld[3] << " y" << tranformOdomToWorld[4]
                  << " z" << tranformOdomToWorld[5] <<std::endl;
        publishCloud(&pubLaserCloudInWorld, unused_result, timeLaserInfoStamp, "map");
        publishCloud(&pubGlobalMapWorld, globalMapCloudDS, timeLaserInfoStamp, "map");

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        {
            initializedFlag = Initializing;
            std::cout << "Initializing Fail" << std::endl;
            return;
        }
        else
        {
            initializedFlag = Initialized;
            std::cout << "Initializing Succeed" << std::endl;
            geometry_msgs::PoseStamped pose_odomTo_map;
            tf::Quaternion q_odomTo_map = tf::createQuaternionFromRPY(deltaR, deltaP, deltaY);

            pose_odomTo_map.header.stamp = timeLaserInfoStamp;
            pose_odomTo_map.header.frame_id = "map";
            pose_odomTo_map.pose.position.x = deltax; pose_odomTo_map.pose.position.y = deltay; pose_odomTo_map.pose.position.z = deltaz;
            pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
            pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
            pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
            pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
            pubOdomToMapPose.publish(pose_odomTo_map);

        }
    }

    void GlobalICPScanMatch()
    {
	    if (cloudKeyPoses3D->points.empty() == true)
        {
            return;
        }

        int latestFrameIDlocalizeOnly;
        latestFrameIDlocalizeOnly = win_cloudKeyPoses3D.size() - 1;

        pcl::PointCloud<PointType>::Ptr latestCloudIn(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> mLock(mtxWin);
            *latestCloudIn += *transformPointCloud(win_cornerCloudKeyFrames[latestFrameIDlocalizeOnly], &win_cloudKeyPoses6D[latestFrameIDlocalizeOnly]);
            *latestCloudIn += *transformPointCloud(win_surfCloudKeyFrames[latestFrameIDlocalizeOnly],   &win_cloudKeyPoses6D[latestFrameIDlocalizeOnly]);
        }
        std::cout << "the size of input cloud: " << latestCloudIn->points.size() <<std::endl;

        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.01);
        ndt.setResolution(1.0);


        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align cloud
        // Calculate the tranform of odom relative to world
        Eigen::Affine3f transodomToWorld_init;
        {
            std::lock_guard<std::mutex> mLock(mtxtranformOdomToWorld);
            transodomToWorld_init = pcl::getTransformation(tranformOdomToWorld[3], tranformOdomToWorld[4],tranformOdomToWorld[5],tranformOdomToWorld[0],tranformOdomToWorld[1],tranformOdomToWorld[2]);
        }
        Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();

        //Firstly perform ndt in coarse resolution
        ndt.setInputSource(latestCloudIn);
        ndt.setInputTarget(globalMapCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
        ndt.align(*unused_result_0, matricInitGuess);
        //use the outcome of ndt as the initial guess for ICP
        icp.setInputSource(latestCloudIn);
        icp.setInputTarget(globalMapCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result, ndt.getFinalTransformation());

        std::cout << "ICP converg flag:" << icp.hasConverged() << ". Fitness score: " << icp.getFitnessScore() << std::endl << std::endl;

        Eigen::Affine3f transodomToWorld_New;
        transodomToWorld_New = icp.getFinalTransformation();
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles (transodomToWorld_New, x, y, z, roll, pitch, yaw);

        {
            //renew tranformOdomToWorld
            std::lock_guard<std::mutex> mLock(mtxtranformOdomToWorld);
            tranformOdomToWorld[0] = roll;
            tranformOdomToWorld[1] = pitch;
            tranformOdomToWorld[2] = yaw;
            tranformOdomToWorld[3] = x;
            tranformOdomToWorld[4] = y;
            tranformOdomToWorld[5] = z;
        }

        //publish global map
        publishCloud(&pubGlobalMapWorld, globalMapCloudDS, timeLaserInfoStamp, "map");

        if (icp.hasConverged() == true && icp.getFitnessScore() < historyKeyframeFitnessScore)
        {
            geometry_msgs::PoseStamped pose_odomTo_map;
            tf::Quaternion q_odomTo_map = tf::createQuaternionFromRPY(roll, pitch, yaw);

            pose_odomTo_map.header.stamp = timeLaserInfoStamp;
            pose_odomTo_map.header.frame_id = "map";
            pose_odomTo_map.pose.position.x = x; pose_odomTo_map.pose.position.y = y; pose_odomTo_map.pose.position.z = z;
            pose_odomTo_map.pose.orientation.x = q_odomTo_map.x();
            pose_odomTo_map.pose.orientation.y = q_odomTo_map.y();
            pose_odomTo_map.pose.orientation.z = q_odomTo_map.z();
            pose_odomTo_map.pose.orientation.w = q_odomTo_map.w();
            pubOdomToMapPose.publish(pose_odomTo_map);
        }
    }

    void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
    {
        //first calculate global pose
        //x-y-z
        if(initializedFlag == Initialized)
            return;

        float x = pose_msg->pose.pose.position.x;
        float y = pose_msg->pose.pose.position.y;
        float z = pose_msg->pose.pose.position.z;

        //roll-pitch-yaw
        tf::Quaternion q_global;
        double roll_global; double pitch_global; double yaw_global;

        q_global.setX(pose_msg->pose.pose.orientation.x);
        q_global.setY(pose_msg->pose.pose.orientation.y);
        q_global.setZ(pose_msg->pose.pose.orientation.z);
        q_global.setW(pose_msg->pose.pose.orientation.w);

        tf::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);
        //global transformation
        transformInTheWorld[0] = roll_global;
        transformInTheWorld[1] = pitch_global;
        transformInTheWorld[2] = yaw_global;
        transformInTheWorld[3] = x;
        transformInTheWorld[4] = y;
        transformInTheWorld[5] = z;
        PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
        Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);
        //Odom transformation
        PointTypePose thisPose6DInOdom = trans2PointTypePose(transformTobeMapped);
        Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);
        //transformation: Odom to Map
        Eigen::Affine3f T_OdomToMap = T_thisPose6DInWorld * T_thisPose6DInOdom.inverse();
        float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
        pcl::getTranslationAndEulerAngles (T_OdomToMap, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        {
            std::lock_guard<std::mutex> mLock(mtxtranformOdomToWorld);
            tranformOdomToWorld[0] = delta_roll;
            tranformOdomToWorld[1] = delta_pitch;
            tranformOdomToWorld[2] = delta_yaw;
            tranformOdomToWorld[3] = delta_x;
            tranformOdomToWorld[4] = delta_y;
            tranformOdomToWorld[5] = delta_z;
        }
        initializedFlag = NonInitialized;
    }
    //bb: addition

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam_lo");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread localizeInWorldThread(&mapOptimization::localizeOnlyThread, &MO);

    ros::spin();

    localizeInWorldThread.join();

    return 0;
}