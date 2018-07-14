#ifndef KINECTGRABBER_H
#define KINECTGRABBER_H


#include <pcl/io/openni_grabber.h>
#include "global.h"


//Union structure for RGB information in PCD file
union PCD_BGRA
{
    struct
    {
        uchar B; // LSB
        uchar G; // ---
        uchar R; // MSB
        uchar A; //
    };
    float RGB_float;
    uint  RGB_uint;
};

class KinectGrabber
{

    boost::shared_ptr<pcl::visualization::CloudViewer> viewer; // Point cloud viewer object.
    pcl::OpenNIGrabber* openniGrabber; // OpenNI grabber that takes data from the device.
    unsigned int filesSaved;  // For the numbering of the clouds saved to disk.
    bool saveCloud;

//    const float badPoint;


public:
    KinectGrabber();

    // This function is called every time the device has new data for point cloud capture.
    void grabberCallbackPc(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

    //this function is called every time the device has new data for RGB and Depth Map
    void grabberCallbackRgbDm(const boost::shared_ptr<openni_wrapper::Image>& rgbImageNi,
                              const boost::shared_ptr<openni_wrapper::DepthImage>& depthImageNi, float constantFocal);

    // For detecting when SPACE is pressed.
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                          void* nothing);

    // Creates, initializes and returns a new viewer.
    boost::shared_ptr<pcl::visualization::CloudViewer> createViewer();

    //function to strat the scan process to grab rgb and depth frames and also pointclouds
    void startScan();

    //function to convert RGB and Depth Images to Pointclouds
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > getPcd( cv::Mat &rgbImage, cv::Mat &depthImage);

    //function to convert the depth values from depth image to meters
    void depthToMeter( const int p_FeatX, const int p_FeatY, const int p_RawDisparity,
                                   float &p_X, float &p_Y, float &p_Z );
};

#endif // KINECTGRABBER_H
