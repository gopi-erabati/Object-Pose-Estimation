#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

// **********************                      Class for                           *****************************
// ********************** ROS Interface to Subscribe to RGB and Depth Image topics *****************************
// ********************** Use segmentation class to cluster objects                *****************************
// ********************** Use PoseEstimator class to estimate pose and             *****************************
// ********************** Display in viewer and publish pose on /tf topic          *****************************

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//Users
#include <global.h>
#include <datagrabber.h>
#include <objectsegmentationplane.h>
#include <poseestimator.h>
#include <processingpcd.h>
#include <geometry.h>

class RosInterface
{
    //ROS Interface Variables
    cv::Mat depthMat, rgbMat;
    cv_bridge::CvImagePtr imgPtrDepth, imgPtrRgb;
    bool euclidSensor, kinectSensor, astraSensor;
    ros::Publisher pubPose;

    // Other variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSource, cloudTarget,cloudFromRgbDepth, cloudTargetSeg, cloudPlane, cloudSourceOriginal, cloudTargetFiltered;
    Eigen::Matrix4f pose;
    bool newFrame,firstFrame;
    std::vector<pcl::PointCloud<PointTObj>::Ptr> cloudClusterVector;
    int prevCloudClusterSize;
    std::vector<std::vector <float> > clusterColorTable;
    pcl::visualization::PCLVisualizer::Ptr viewer;


    //Objects
    ObjectSegmentationPlane objSegPlane;
    PoseEstimator poseEstimator;
    ProcessingPcd procPcd;
    DataGrabber *dataGrabber;



public:
    //Constructor to set the sensor type
    RosInterface( bool p_euclidSensor, bool p_kinectSensor, bool p_astraSensor );

    // Function to start the interface and subscribe Ros topics and estimate pose
    void startRosInterface( int argc, char** argv, std::string objName, int scenario, std::vector<float> limits, std::string pathToObj );

    // Callback to subscribe RGB and Depth Images from Kinect and convert to point cloud
    void callbackRgbDepth( const sensor_msgs::ImageConstPtr& msgRgb , const sensor_msgs::ImageConstPtr& msgDepth );

    // Function to publish pose in TransformStamped Type
    // frame_id : model_frame
    // child_frame_id : object_frame
    void publishPose();

    // Callback to subscribe Depth Images from Astra sensor and Euclid sensor and convert to point cloud
    void callbackDepth(const sensor_msgs::ImageConstPtr &msgDepth );

    // Function to create viewer
    pcl::visualization::PCLVisualizer::Ptr createViewer();
};

#endif // ROSINTERFACE_H
