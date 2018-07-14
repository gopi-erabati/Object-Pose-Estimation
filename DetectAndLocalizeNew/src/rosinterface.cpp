#include "../include/rosinterface.h"

//Constructor to set the sensor type and intialise varibles and start the viewer
RosInterface::RosInterface(bool p_euclidSensor, bool p_kinectSensor, bool p_astraSensor):euclidSensor( p_euclidSensor ), kinectSensor ( p_kinectSensor ), astraSensor ( p_astraSensor )
{
    newFrame = false; firstFrame = true;
    pose << Eigen::Matrix4f::Identity();
    prevCloudClusterSize = 0;

    //for cluster colors
    clusterColorTable = std::vector< std::vector <float> >(15, std::vector<float>(3));
    clusterColorTable[0][0] = 0; clusterColorTable[0][1] = 0; clusterColorTable[0][2] = 1; //Blue
    clusterColorTable[1][0] = 1; clusterColorTable[1][1] = 1; clusterColorTable[1][2] = 0; //Yellow
    clusterColorTable[2][0] = 0; clusterColorTable[2][1] = 1; clusterColorTable[2][2] = 1; //Cyan
    clusterColorTable[3][0] = 1; clusterColorTable[3][1] = 0; clusterColorTable[3][2] = 1; //magenta
    clusterColorTable[4][0] = 0.5; clusterColorTable[4][1] = 0.5; clusterColorTable[4][2] = 0; //Olive
    clusterColorTable[5][0] = 0; clusterColorTable[5][1] = 0.5; clusterColorTable[5][2] = 0.5; //teal
    clusterColorTable[6][0] = 1; clusterColorTable[6][1] = 0.65; clusterColorTable[6][2] = 0; //orange
    clusterColorTable[7][0] = 0.85; clusterColorTable[7][1] = 0.65; clusterColorTable[7][2] = 0.125; //golden rod
    clusterColorTable[8][0] = 0.541; clusterColorTable[8][1] = 0.168; clusterColorTable[8][2] = 0.89; //blue viloet
    clusterColorTable[9][0] = 0.3; clusterColorTable[9][1] = 0.51; clusterColorTable[9][2] = 0.51; //indigo
    clusterColorTable[10][0] = 0.87; clusterColorTable[10][1] = 0.63; clusterColorTable[10][2] = 0.87; //plum
    clusterColorTable[11][0] = 0.96; clusterColorTable[11][1] = 0.87; clusterColorTable[11][2] = 0.7; //wheat
    clusterColorTable[12][0] = 0.545; clusterColorTable[12][1] = 0.27; clusterColorTable[12][2] = 0.075; //saddle brown
    clusterColorTable[13][0] = 0.74; clusterColorTable[13][1] = 0.56; clusterColorTable[13][2] = 0.56; //rosy brown
    clusterColorTable[14][0] = 0.44; clusterColorTable[14][1] = 0.5; clusterColorTable[14][2] = 0.56; //slate grey

    // set sensor for datagrabber class
    dataGrabber = new DataGrabber(euclidSensor, kinectSensor, astraSensor);

    // pointclouds
    cloudSource = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudSourceOriginal = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudFromRgbDepth = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudTarget = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudTargetSeg = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudPlane = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudTargetFiltered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB> );

    // start viewer
    viewer = createViewer();
    viewer->resetCamera();

}

// Function to start the interface and subscribe Ros topics and estimate pose
void RosInterface::startRosInterface(int argc, char **argv, std::string objName, int scenario, std::vector<float> limits, std::string pathToObj){

    // load the model cloud
    // If there is no path given load the previously built models (brick, drill, cylinder)
    // otherwise load the model according to path
    if (pathToObj == "empty"){
       //load the objects already built
        std::string fileName;
        if ( objName == "brick"){

            //BRICK
            //std::string fileName = "/home/gkerabat/Documents/Project/DataSets/Kinect/Brick/Registration_17-27-08/GoodIcp/cloudIcpOutRem.pcd";
            //fileName = "../3DModel/brick.pcd";
            //fileName = "../3DModel/brickModelAxisAligned.pcd";
            fileName = "../3DModel/brickModelAxisAlignedOrigin.pcd";

        }
        else if (objName == "drill"){
            fileName = "../3DModel/drillNewModelOrigin.pcd";
        }
        else if (objName == "yellow"){

            //RANDOM
            //std::string fileName = "/home/gkerabat/Documents/Project/DataSets/Kinect/Random/Icp/cloudIcpOutRem.pcd";
            //fileName = "../3DModel/random.pcd";
            //fileName = "../3DModel/randomModelAxisAligned.pcd";
            fileName = "../3DModel/randomModelAxisAlignedOrigin.pcd";
        }
        else{
            std::cout << "Unknown object name" << std::endl;
        }

        //load the cloud
        if ( pcl::io::loadPCDFile( fileName, *cloudSource ) == -1 ){
            PCL_ERROR (" could not load pcd file \n");
        }
        else{
            std::cout << "loaded cloud of (brick(0), drill(1), yellow(2)) " << objName << std::endl;
        }
        pcl::copyPointCloud( *cloudSource, *cloudSourceOriginal);
    }
    else{

        boost::filesystem::path pathToObjBoost(pathToObj);
        if( pathToObjBoost.extension() == ".pcd"){

            //load the cloud
            if ( pcl::io::loadPCDFile( pathToObj, *cloudSource ) == -1 ){
                PCL_ERROR (" could not load pcd file \n");
            }
            else{
                std::cout << "loaded cloud of new object "  << std::endl;
            }
            pcl::copyPointCloud( *cloudSource, *cloudSourceOriginal);
        }
        else if (pathToObjBoost.extension() == ".ply"){

            //load the cloud
            if ( pcl::io::loadPLYFile( pathToObj, *cloudSource ) == -1 ){
                PCL_ERROR (" could not load pcd file \n");
            }
            else{
                std::cout << "loaded cloud of new object "  << std::endl;
            }
            pcl::copyPointCloud( *cloudSource, *cloudSourceOriginal);
        }

    }

    // Initialize the ROS system and start the node.
    ros::init(argc, argv, "Object_Localization");
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
        //compressed_depth_image_transport::CompressedDepthSubscriber sub; sub.subscribe( *nodeHandle, "/rgbd/depth/image_raw", 100, &RosInterface::callbackDepthCompressed, this, image_transport::TransportHints("compressed")) ;
        //for only depth
        sub = it.subscribe( "/rgbd/depth/image_raw", 10, &RosInterface::callbackDepth, this, image_transport::TransportHints("raw") );

        //for depth and color
        //Initilaise Subscriber for RGB and Depth topics
        //    message_filters::Subscriber<sensor_msgs::Image> subscriberDepth;
        //    message_filters::Subscriber<sensor_msgs::Image> subscriberRgb;

        //    subscriberDepth.subscribe( *nodeHandle , "/rgbd/depth_registered/image_raw" , 100 );
        //    subscriberRgb.subscribe( *nodeHandle , "/rgbd/rgb/image_color" , 100 );

        //    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mySyncPolicy;

        //    //ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
        //    boost::shared_ptr<message_filters::Synchronizer<mySyncPolicy> > sync;
        //    sync.reset( new message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(100), subscriberRgb, subscriberDepth ) );
        //    sync->registerCallback(boost::bind(&RosInterface::callbackRgbDepthAstra, this, _1, _2));
    }
    else if ( euclidSensor ){
        subRos = nodeHandle->subscribe( "/camera/depth/image_raw", 100, &RosInterface::callbackDepth, this );

    }

    pubPose = nodeHandle->advertise<tf2_msgs::TFMessage>("/tf", 1000);


    while(ros::ok())
    {
        ros::spinOnce();
        viewer->spinOnce(100);

        //For every new frame estimate pose
        if(newFrame){

            //cv::Mat depMat = depthMat;
            //cloudTarget = dataGrabber->rgbd2Pcl( depMat );

            // Store the cloud from callback to a other variable to process it
            pcl::copyPointCloud(*cloudFromRgbDepth, *cloudTarget);

            // Remove existing pointclouds and texts on viewer
            viewer->removePointCloud("CloudTarget");
            viewer->removePointCloud("CloudSource");
            viewer->removePointCloud("cloudPlane");
            viewer->removeText3D("viewerText");

            double fitnessScore = 10.0;double alignedStrength = 0.0;

            //For table
            if ( scenario == 0){

                //show cloud
                if(!cloudTarget->empty()){
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTarget( cloudTarget );
                    viewer->addPointCloud<pcl::PointXYZRGB> (cloudTarget, rgbTarget, "CloudTarget");
                    viewer->resetCameraViewpoint("cloudTarget");
                }

                //apply tabletop
                {
                    pcl::ScopeTime t("TableTop");
                    cloudClusterVector.clear();
                    cloudTargetFiltered = procPcd.getPassThrough(cloudTarget, limits.at(0), limits.at(1), limits.at(2), limits.at(3), limits.at(4), limits.at(5) );
                    bool isPlane = objSegPlane.getSegmentedObjectsOnPlane( cloudTargetFiltered, cloudClusterVector, cloudPlane );
                }
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcloudPlane( cloudPlane );
                viewer->addPointCloud<pcl::PointXYZRGB>( cloudPlane, rgbcloudPlane, "cloudPlane" );
                viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloudPlane");

                //check cluster empty and proceed
                if(!cloudClusterVector.empty()){

                    //remove previous clusters
                    for(int j = 0; j < prevCloudClusterSize; ++j){
                        std::stringstream stream1; stream1 << "Cloud" << j;
                        std::string cloudName1 = stream1.str(); //name of cloud
                        viewer->removePointCloud(cloudName1);
                    }

                    //show all clusters
                    for(int i=0; i < cloudClusterVector.size(); ++i){
                        std::stringstream stream; stream << "Cloud" << i;
                        std::string cloudName = stream.str(); //name of cloud
                        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcloudCluster( cloudClusterVector.at(i) );
                        viewer->addPointCloud<pcl::PointXYZRGB>( cloudClusterVector.at(i), rgbcloudCluster, cloudName );
                        double value1 = 0.1*(i+1); double value2 = 0.15*(i+1); double value3 = 0.4*(i+1);
                        viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, clusterColorTable[i][0], clusterColorTable[i][1], clusterColorTable[i][2], cloudName);

                    }

                    //for first frame , check alignment with all clsuters and
                    //for next frame check centroid distance between clusters and
                    // previously aligned model and align to the object whose distance is less than threshold
                    if(firstFrame){
                        for(int i = 0; i < cloudClusterVector.size(); ++i){

                            cloudTargetSeg = cloudClusterVector.at(i);

                            pcl::copyPointCloud(*cloudSourceOriginal, *cloudSource);
                            if (!cloudTargetSeg->empty()){
                                pose = poseEstimator.estimateFinalPose( cloudSource, cloudTargetSeg, fitnessScore, alignedStrength);
                            }

                            //                            if(fitnessScore < 0.0001){
                            //                                break;
                            //                            }
                            if(fitnessScore < 0.0001 || alignedStrength > 0.4){
                                break;
                            }
                        }
                        firstFrame = false;
                    }
                    else{
                        double distance = 10.0;
                        for(int i = 0; i < cloudClusterVector.size(); ++i){
                            cloudTargetSeg = cloudClusterVector.at(i);

                            //compute centroid of cluster cloud and model cloud
                            Eigen::Vector4f centroidClusterTemp (Eigen::Vector4f::Zero());
                            pcl::compute3DCentroid(*cloudTargetSeg, centroidClusterTemp);
                            Eigen::Vector3f centroidCluster = centroidClusterTemp.head<3>();

                            Eigen::Vector4f centroidModelTemp (Eigen::Vector4f::Zero());
                            pcl::compute3DCentroid(*cloudSource, centroidModelTemp);
                            Eigen::Vector3f centroidModel = centroidModelTemp.head<3>();

                            distance = (centroidCluster - centroidModel).lpNorm<2>();

                            if(distance < 0.05){
                                if (!cloudTargetSeg->empty()){
                                    pose = poseEstimator.estimateFinalPose( cloudSource, cloudTargetSeg, fitnessScore, alignedStrength );
                                    break;
//                                    if(fitnessScore < 0.0001 || alignedStrength > 0.4){
//                                        break;
//                                    }
//                                    else{
//                                        for(int i = 0; i < cloudClusterVector.size(); ++i){

//                                            cloudTargetSeg = cloudClusterVector.at(i);

//                                            pcl::copyPointCloud(*cloudSourceOriginal, *cloudSource);
//                                            if (!cloudTargetSeg->empty()){
//                                                pose = poseEstimator.estimateFinalPose( cloudSource, cloudTargetSeg, fitnessScore, alignedStrength );
//                                            }

//                                            //                                if(fitnessScore < 0.0001){
//                                            //                                    break;
//                                            //                                }
//                                            if(fitnessScore < 0.0001 || alignedStrength > 0.4){
//                                                break;
//                                            }
//                                        }
//                                        break;
//                                    }
                                }
                                //break;
                            }
                        }
                        // if the objects are moved with more than 5cm distance, then again check alignmnet with all clusters
                        if(distance > 0.05){
                            for(int i = 0; i < cloudClusterVector.size(); ++i){

                                cloudTargetSeg = cloudClusterVector.at(i);

                                pcl::copyPointCloud(*cloudSourceOriginal, *cloudSource);
                                if (!cloudTargetSeg->empty()){
                                    pose = poseEstimator.estimateFinalPose( cloudSource, cloudTargetSeg, fitnessScore, alignedStrength );
                                }

                                //                                if(fitnessScore < 0.0001){
                                //                                    break;
                                //                                }
                                if(fitnessScore < 0.0001 || alignedStrength > 0.4){
                                    break;
                                }
                            }
                        }
                    }
                }

                // show aligned model in Red color
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbSource( cloudSource );
                viewer->addPointCloud<pcl::PointXYZRGB>( cloudSource, rgbSource, "CloudSource" );
                viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "CloudSource");

                // update clustersize
                prevCloudClusterSize = cloudClusterVector.size();

                // Convert rotation angles from Quaternion to Euler and show on Viewer
                Eigen::Quaternionf orientQuatAlgo( pose.topLeftCorner<3,3>());
                double roll = 0; double pitch = 0; double yaw = 0;
                quaternion2EulerAngle( orientQuatAlgo, roll, pitch, yaw);
                std::stringstream streamText; streamText << "No. of Clusters : " << prevCloudClusterSize << " || " << "Alignment Strength : " << alignedStrength << " || " << "Fittness Score : " << fitnessScore << " || " << std::endl <<
                                                            "Pose :: Translation : " << pose(0,3) << ", " << pose(1,3) << ", " << pose(2,3) << " | " << "Rotation : " << roll << ", " << pitch << ", " << yaw << " || ";
                std::string textViewer = streamText.str();
                viewer->addText(textViewer, 10, 10, "viewerText");
            }
            // if object is not on table ; held in hand; need to develop more good segmentation algortihms!
            else if (scenario == 1){

                if(!cloudTarget->empty()){
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbTarget( cloudTarget );
                    viewer->addPointCloud<pcl::PointXYZRGB> (cloudTarget, rgbTarget, "CloudTarget");
                    viewer->resetCameraViewpoint("cloudTarget");
                }

                //only when there is no plane detected
                cloudTargetSeg = procPcd.getPassThrough( cloudTarget,std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8])); //1.20 Random 1.1 Other

                if (!cloudTargetSeg->empty()){
                    pose = poseEstimator.estimateFinalPose( cloudSource, cloudTargetSeg, fitnessScore, alignedStrength );
                }

                // show aligned model in Red Color
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbSource( cloudSource );
                viewer->addPointCloud<pcl::PointXYZRGB>( cloudSource, rgbSource, "CloudSource" );
                viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "CloudSource");

                // Convert rotation angles from Quaternion to Euler and show on Viewer
                Eigen::Quaternionf orientQuatAlgo( pose.topLeftCorner<3,3>());
                double roll = 0; double pitch = 0; double yaw = 0;
                quaternion2EulerAngle( orientQuatAlgo, roll, pitch, yaw);
                std::stringstream streamText; streamText << "Alignment Strength : " << alignedStrength << " || " << "Fittness Score : " << fitnessScore << " || " << std::endl <<
                                                            "Pose :: Translation : " << pose(0,3) << ", " << pose(1,3) << ", " << pose(2,3) << " | " << "Rotation : " << roll << ", " << pitch << ", " << yaw << " || ";
                std::string textViewer = streamText.str();
                viewer->addText(textViewer, 10, 10, "viewerText");
            }

            // publish pose on /tf topic
            publishPose();
            newFrame = false;
        }
    }
}


// Callback to subscribe Depth Images from Astra sensor and Euclid sensor and convert to point cloud
void RosInterface::callbackDepth( const sensor_msgs::ImageConstPtr& msgDepth )
{

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

    depthMat = imgPtrDepth->image;

    //cv::Mat outputImg = cv::imdecode(msgDepth->data, CV_LOAD_IMAGE_UNCHANGED);

    // To save RGBD Data
    //    char file_rgb[100];
    //    char file_depth[100];

    //    sprintf( file_rgb, "/home/gkerabat/Documents/Project/testtracking/rgbd_sync/colour/rgb_%04d.png", countImages );
    //    sprintf( file_depth, "/home/gopikrishna/Documents/Thesis/Datasets/Astra/Brick/DepthData/depth_%04d.png", countImages );

    //    cv::imwrite( file_rgb , mat_rgb);
    //    cv::imwrite( file_depth, depthMat);
    //    std::cout << depthMat.type() << std::endl;

    //ROS_INFO_STREAM(countImages << "\n");
    //ROS_INFO_STREAM("Images saved\n");

    cloudFromRgbDepth = dataGrabber->rgbd2Pcl( depthMat );
    newFrame = true;


}

// Function to publish pose in TransformStamped Type
// frame_id : model_frame
// child_frame_id : object_frame
void RosInterface::publishPose(){

    //publish Pose
    geometry_msgs::TransformStamped msgPose;
    msgPose.transform.translation.x = pose(0,3);
    msgPose.transform.translation.y = pose(1,3);
    msgPose.transform.translation.z = pose(2,3);

    Eigen::Quaternionf quatPose =  Eigen::Quaternionf( pose.topLeftCorner<3,3>() );
    msgPose.transform.rotation.w = quatPose.w();
    msgPose.transform.rotation.x = quatPose.x();
    msgPose.transform.rotation.y = quatPose.y();
    msgPose.transform.rotation.z = quatPose.z();
    msgPose.header.stamp.sec = ros::Time::now().sec;
    msgPose.header.stamp.nsec = ros::Time::now().nsec;
    msgPose.header.frame_id = "model_frame";
    msgPose.child_frame_id = "object_frame";

    tf2_msgs::TFMessage msg; msg.transforms.push_back(msgPose);
    pubPose.publish(msg);

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

    // To save RGBD Data
    //    char file_rgb[100];
    //    char file_depth[100];

    //    sprintf( file_rgb, "/home/gkerabat/Documents/Project/testtracking/rgbd_sync/colour/rgb_%04d.png", countImages );
    //    sprintf( file_depth, "/home/gkerabat/Documents/Project/testtracking/rgbd_sync/depth/depth_%04d.png", countImages );

    //    cv::imwrite( file_rgb , mat_rgb);
    //    cv::imwrite( file_depth, mat_depth);

    //ROS_INFO_STREAM(countImages << "\n");
    //ROS_INFO_STREAM("Images saved\n");

    // convert to pointclouds
    cloudFromRgbDepth = dataGrabber->rgbd2Pcl( rgbMat, depthMat );

    newFrame = true;


}

// Function to create viewer
pcl::visualization::PCLVisualizer::Ptr RosInterface::createViewer(){

    pcl::visualization::PCLVisualizer::Ptr v ( new pcl::visualization::PCLVisualizer ("Cloud Viewer"));
    v->setBackgroundColor( 0.0, 0.0, 0.0);
    v->addCoordinateSystem( 0.3 );
    v->initCameraParameters();

    return v;
}
