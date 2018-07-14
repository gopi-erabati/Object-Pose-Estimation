#ifndef DATAGRABBER_H
#define DATAGRABBER_H

#include <global.h>

// ***************** Class which deals with the conversion of RGB and Depth Images to PointClouds ****************

class DataGrabber
{

    bool euclidSensor, kinectSensor, astraSensor;

public:
    //Constructor to set the Sensor
    DataGrabber( bool p_euclidSensor, bool p_kinectSensor, bool p_astraSensor );

    //function to convert RGB and Depth Maps to Point Clouds
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > rgbd2Pcl( cv::Mat p_imageRgb, cv::Mat p_imageDepth );

    //function to convert Depth Maps to Point Clouds
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > rgbd2Pcl( cv::Mat p_imageDepth );

    //function to convert the depth values from depth image to meters
    void depthToMeter(const float p_FeatX, const float p_FeatY, const float p_RawDisparity,
                      float &p_X, float &p_Y, float &p_Z);
};

#endif // DATAGRABBER_H
