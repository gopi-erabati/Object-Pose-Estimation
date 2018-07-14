#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

// **********************                      Class for                           *****************************
// ********************** ROS Interface to Subscribe to RGB and Depth Image topics *****************************
// ********************** convert to point clouds                                  *****************************
// ********************** PRESS c - capture ; f- finish                            *****************************

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
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//Users
#include <global.h>
#include <datagrabber.h>
#include <objectsegmentationplane.h>
#include <processingpcd.h>
#include <geometry.h>


class RosInterface
{

    //ROS Interface Variables
    cv_bridge::CvImagePtr imgPtrRgb, imgPtrDepth;
    cv::Mat depthMat, rgbMat;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTarget, cloudFromRgbDepth;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool euclidSensor, kinectSensor, astraSensor;
    bool capture, finish;

    //Objects
    DataGrabber *dataGrabber;

public:
    //Constructor to set the sensor type
    RosInterface( bool p_euclidSensor, bool p_kinectSensor, bool p_astraSensor );

    // Function to start the interface and subscribe Ros topics
    bool startScan(int argc, char **argv, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &cloudVector);

    // Callback to subscribe RGB and Depth Images from Kinect and convert to point cloud
    void callbackRgbDepth( const sensor_msgs::ImageConstPtr& msgRgb , const sensor_msgs::ImageConstPtr& msgDepth );

    // Callback to subscribe Depth Images from Astra sensor and Euclid sensor and convert to point cloud
    void callbackDepth(const sensor_msgs::ImageConstPtr &msgDepth );

    // Function to create viewer
    pcl::visualization::PCLVisualizer::Ptr createViewer();

    // Function for keyboard callback to press 'c' and 'f'
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                                void* viewer_void);
};

#endif // ROSINTERFACE_H
