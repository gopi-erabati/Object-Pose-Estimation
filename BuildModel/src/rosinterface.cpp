#include "../include/rosinterface.h"

//Constructor to set the sensor type
RosInterface::RosInterface(bool p_euclidSensor, bool p_kinectSensor, bool p_astraSensor):euclidSensor( p_euclidSensor ), kinectSensor ( p_kinectSensor ), astraSensor ( p_astraSensor )
{

    dataGrabber = new DataGrabber(euclidSensor, kinectSensor, astraSensor);

    cloudFromRgbDepth = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    capture = false; finish = false;

    // Start Viewer
    viewer = createViewer();
    viewer->resetCamera();


}

// Function to start the interface and subscribe Ros topics
bool RosInterface::startScan(int argc, char **argv, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &cloudVector){

    // Initialize the ROS system and start a node.
    ros::init(argc, argv , "scanNode");
    boost::shared_ptr<ros::NodeHandle> nodeHandle;
    nodeHandle.reset( new ros::NodeHandle () );

    std::cout << "Node started" << std::endl;

    // Subscribers and Synchronizers
    //Kinect
    //Initilaise Subscriber for RGB and Depth topics
    message_filters::Subscriber<sensor_msgs::Image> subscriberDepth;
    message_filters::Subscriber<sensor_msgs::Image> subscriberRgb;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mySyncPolicy;
    //ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
    boost::shared_ptr<message_filters::Synchronizer<mySyncPolicy> > sync;


    //Astra
    image_transport::ImageTransport it( *nodeHandle );
    image_transport::Subscriber sub;

    //Euclid
    ros::Subscriber subRos;

    if ( kinectSensor ){

        subscriberDepth.subscribe( *nodeHandle , "/camera/depth_registered/image_raw" , 10 );
        subscriberRgb.subscribe( *nodeHandle , "/camera/rgb/image_color" , 10 );

        sync.reset( new message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), subscriberRgb, subscriberDepth ) );
        sync->registerCallback(boost::bind(&RosInterface::callbackRgbDepth, this, _1, _2));
    }
    else if ( astraSensor ){
        //for only depth
        sub = it.subscribe( "/rgbd/depth/image_raw", 100, &RosInterface::callbackDepth, this, image_transport::TransportHints("raw") );
    }
    else if ( euclidSensor ){

        subRos  = nodeHandle->subscribe( "/camera/depth/image_raw", 100, &RosInterface::callbackDepth, this );

    }

    while(ros::ok())
    {
        ros::spinOnce();
        viewer->spinOnce(100);

        // remove esxisting point clouds
        viewer->removePointCloud("CloudTarget");
        viewer->removePointCloud("cloudPlane");
        viewer->removeText3D("label");

        // copy cloud from callback to a new variable
        cloudTarget = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
        pcl::copyPointCloud(*cloudFromRgbDepth, *cloudTarget);

        //show cloud
        if(!cloudTarget->empty()){
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTarget( cloudTarget );
            viewer->addPointCloud<pcl::PointXYZRGB> (cloudTarget, rgbTarget, "CloudTarget");
            viewer->resetCameraViewpoint("cloudTarget");
        }

        viewer->addText(" PRESS -> c - capture ; f - finish ", 20, 20, "label");

        // if 'c' is pressed save cloud in vector
        if(capture){
            cloudVector.push_back(cloudTarget);
            capture = false;
        }
        // if 'f' is pressed stop scanning
        if(finish){
            viewer->close();
            ros::shutdown();
            return true;
        }

    }
}

// Callback to subscribe RGB and Depth Images from Kinect and convert to point cloud
void RosInterface::callbackRgbDepth( const sensor_msgs::ImageConstPtr& msgRgb , const sensor_msgs::ImageConstPtr& msgDepth )
{

    //Convert Depth rosmsg to openCV
    try{
        imgPtrDepth = cv_bridge::toCvCopy( *msgDepth ); //, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    //Convert RGB rosmsg to openCV
    try{
        imgPtrRgb = cv_bridge::toCvCopy( *msgRgb ); //, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }


    depthMat = imgPtrDepth->image;
    rgbMat = imgPtrRgb->image;

    // convert to pointclouds
    cloudFromRgbDepth = dataGrabber->rgbd2Pcl( rgbMat, depthMat );

}

// Callback to subscribe Depth Images from Astra sensor and Euclid sensor and convert to point cloud
void RosInterface::callbackDepth( const sensor_msgs::ImageConstPtr& msgDepth )
{
    std::cout << "Callback" << std::endl;
    viewer->removePointCloud("CloudTarget");
    viewer->removePointCloud("CloudSource");

    //ROS_INFO_STREAM("callback\n");
    cv_bridge::CvImagePtr imgPtrDepth;

    //    compressed_depth_image_transport compDepth;
    //    const sensor_msgs::Image::Ptr msgDepthNew =  compDepth.decodeCompressedDepthImage( *msgDepth );


    //Convert Depth rosmsg to openCV
    try{
        imgPtrDepth = cv_bridge::toCvCopy( *msgDepth);//, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    cv::Mat& depthMat = imgPtrDepth->image;

    // convert to pointclouds
    cloudFromRgbDepth = dataGrabber->rgbd2Pcl( depthMat );

}

// Function to create viewer and add keyboard callback
pcl::visualization::PCLVisualizer::Ptr RosInterface::createViewer(){

    pcl::visualization::PCLVisualizer::Ptr v ( new pcl::visualization::PCLVisualizer ("Scanning Window"));
    v->setBackgroundColor( 0.0, 0.0, 0.0);
    v->addCoordinateSystem( 0.3 );
    v->initCameraParameters();
    v->registerKeyboardCallback(&RosInterface::keyboardEventOccurred, *this);

    return v;
}

// Function for keyboard callback to press 'c' and 'f'
void RosInterface::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void){

    if (event.getKeySym () == "c" && event.keyDown ()){
        capture = true;
    }
    else if ( event.getKeySym() == "f" && event.keyDown()){
        finish = true;
    }

}
