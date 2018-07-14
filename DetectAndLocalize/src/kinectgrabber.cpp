#include "../include/kinectgrabber.h"

KinectGrabber::KinectGrabber()
{

    filesSaved = 0;
    saveCloud = false;
//    badPoint = std::numeric_limits<float>::quiet_NaN();

}

// This function is called every time the device has new data.
void KinectGrabber::grabberCallbackPc( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud )
{
    if (! viewer->wasStopped()){
        viewer->showCloud(cloud);
    }

    //if (saveCloud)
    {
        std::stringstream stream;
        stream << "../DataSets/Kinect/Box1/RawPointClouds/pc" << filesSaved << ".pcd";
        std::string filename = stream.str();
        if (pcl::io::savePCDFile(filename, *cloud, true) == 0)
        {
            filesSaved++;
            std::cout << "Saved " << filename << "." << endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());

        saveCloud = false;
    }
}

////this function is called every time the device has new data for RGB and Depth Map
//void KinectGrabber::grabberCallbackRgbDm( const boost::shared_ptr<openni_wrapper::Image>& rgbImageNi,
//                                          const boost::shared_ptr<openni_wrapper::DepthImage>& depthImageNi, float constantFocal ){

//    //convert openni_wrapper rgb image to opencv rgb image
//    cv::Mat rgbImage = cv::Mat( rgbImageNi->getHeight(), rgbImageNi->getWidth(), CV_8UC3 );
//    rgbImageNi->fillRGB( rgbImage.cols, rgbImage.rows, rgbImage.data, rgbImage.step );
//    cv::cvtColor(rgbImage, rgbImage, CV_BGR2RGB);

//    //convert openni_wrapper depth image to opencv depth image
//    cv::Mat depthImage = cv::Mat( depthImageNi->getHeight(), depthImageNi->getWidth(), CV_32FC1);
//    depthImageNi->fillDepthImage( depthImage.cols, depthImage.rows, (float *)depthImage.data, depthImage.step );
//    //depthImage.convertTo(depthImage, CV_32FC1, 0.125/2.0, 0);

//    //show rgb and depth images frames
//    cv::imshow("RGB Image", rgbImage );
//    cv::imshow("Depth Image", depthImage );
//    cv::waitKey(30);

//    //save RGB and Depth Frames
//    if ( saveCloud ){

//        std::stringstream streamRgb;
//        streamRgb << "../DataSets/test/RGBImages/RGB" << filesSaved << ".jpg";
//        std::string fileName = streamRgb.str();
//        if (cv::imwrite(fileName, rgbImage) == true){
//            std::cout << "Saved" << fileName << "." << std::endl;
//        }

//        std::stringstream streamDep;
//        streamDep << "../DataSets/test/DepthImages/Depth" << filesSaved << ".jpg";
//        fileName = streamDep.str();
//        std::vector<int> compression_params;
//        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
//        compression_params.push_back(100);
//        if (cv::imwrite( fileName, depthImage, compression_params) == true){
//            filesSaved++;
//            std::cout << "Saved" << fileName << "." << std::endl;
//        }

//        saveCloud = false;
//    }

//}

// For detecting when SPACE is pressed.
void KinectGrabber::keyboardEventOccurred( const pcl::visualization::KeyboardEvent& event,
                                           void* nothing )
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<pcl::visualization::CloudViewer> KinectGrabber::createViewer()
{
    boost::shared_ptr<pcl::visualization::CloudViewer> v
            (new pcl::visualization::CloudViewer("OpenNI viewer"));
    v->registerKeyboardCallback( &KinectGrabber::keyboardEventOccurred, *this );


    return (v);
}

//for scanning the objects and starting grabber
void KinectGrabber::startScan(){

    // intialise the grabber
    openniGrabber = new pcl::OpenNIGrabber();


    //template to grab point clouds
    boost::function<void (const  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & )> f =
            boost::bind( &KinectGrabber::grabberCallbackPc, this, _1 );
    openniGrabber->registerCallback(f);

    viewer = createViewer(); //to create cloudviewer to view Pointcloud


//    //template to grab RGB and Depth map
//    boost::function<void ( const boost::shared_ptr<openni_wrapper::Image>&,
//                           const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant )> f1 =
//            boost::bind( &KinectGrabber::grabberCallbackRgbDm, this, _1, _2, _3 );
//    openniGrabber->registerCallback(f1);

    //start the grabber
    openniGrabber->start();

    //Main loop.
    while (! viewer->wasStopped())
        boost::this_thread::sleep(boost::posix_time::seconds(1));

    //stop the grabber
    openniGrabber->stop();
}



//function to convert RGB and Depth Images to Pointclouds
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > KinectGrabber::getPcd(cv::Mat &rgbImage, cv::Mat &depthImage){

    float x, y, z; //to store 3D points

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->width = rgbImage.cols;
    cloud->height = rgbImage.rows;
    cloud->is_dense = true;
    cloud->points.resize(rgbImage.cols * rgbImage.rows);


    //run two loops for rows and columns
    for( int j = 0; j < depthImage.rows ; j++ ){
        for ( int i = 0; i < depthImage.cols; i++ ){

            // store color information in the union defined
            PCD_BGRA pcd_bgra;
            pcd_bgra.B = rgbImage.at<cv::Vec3b>(j, i)[0];
            pcd_bgra.G = rgbImage.at<cv::Vec3b>(j, i)[1];
            pcd_bgra.R = rgbImage.at<cv::Vec3b>(j, i)[2];
            pcd_bgra.A = 0;

            pcl::PointXYZRGBA vertex; //define vertex point

            //get the depth value
            int depthValue = (int) depthImage.at<unsigned short>(j, i);

            //find world cordinates
            depthToMeter(i, j, depthValue, x, y, z);

            // the point is created with depth and color data
            if ( z >= 0 ){
                vertex.x   = x;
                vertex.y   = y;
                vertex.z   = z;
            } else {
                // if the data is outside the boundaries
//                vertex.x   = badPoint;
//                vertex.y   = badPoint;
//                vertex.z   = badPoint;
                continue;
            }
            vertex.rgb = pcd_bgra.RGB_float;

            // the point is pushed back in the cloud
            cloud->points.push_back( vertex );


        }
    }
    return cloud;
}

//function to convert the depth values from depth image to meters
void KinectGrabber::depthToMeter(const int p_FeatX, const int p_FeatY, const int p_RawDisparity,
                                 float &p_X, float &p_Y, float &p_Z ){


    // reject invalid points
    if( p_RawDisparity <= 0 )
    {
        p_X = 0; p_Y = 0; p_Z = 0; return;
    }

    float fx = 525.0; // focal length x
    float fy = 525.0; // focal length y
    float cx = 319.5; // optical center x
    float cy = 239.5; // optical center y
    float sclFactor = 1000.0;

    // Recall the camera projective projection model
    p_Z = p_RawDisparity / sclFactor;
    p_X = (p_FeatX - cx) * p_Z / fx;
    p_Y = (p_FeatY - cy) * p_Z / fy;

}
