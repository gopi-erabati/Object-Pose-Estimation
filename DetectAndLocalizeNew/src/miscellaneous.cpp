#include "miscellaneous.h"


void tableTop(){
    QStringList fileNamesPcd = QFileDialog::getOpenFileNames( 0, "Select Pointclouds", "/home/gkerabat/Documents/Project/DataSets", "PointClouds (*.pcd)", 0, QFileDialog::DontUseNativeDialog);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudVector;

    //load all pcd files and stack them in a vector
    //loop to read point clouds and store in vector
    for (int i = 0; i < fileNamesPcd.size(); i++){

        QString fileName = fileNamesPcd.at(i);
        std::cout << fileName.toStdString() << std::endl ;
        //load the cloud
        if ( pcl::io::loadPCDFile( fileName.toStdString(), *cloud ) == -1 ){
            PCL_ERROR (" could not lad pcd file \n");
        }

        //store clouds in vector
        cloudVector.push_back( cloud );
        cloud.reset( new pcl::PointCloud<pcl::PointXYZRGB> );
    }

    // apply tabletop to detect objects
    ObjectSegmentationPlane objSegPlane;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudVectorObj;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusterVector;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < cloudVector.size(); ++i){

        bool isPlane = objSegPlane.getSegmentedObjectsOnPlane( cloudVector.at(i), cloudClusterVector, cloudPlane );
        cloudVectorObj.push_back( cloudClusterVector.at(0) );

        //save segmented cloud
        std::stringstream stream;
        stream << "/home/gkerabat/Documents/Project/DataSets/Kinect/DrillNew/Registration1/SegPointClouds/cloudCluster_" << i << ".pcd";
        std::string saveFileName = stream.str();
        if (pcl::io::savePCDFile( saveFileName, *cloudClusterVector.at(0), true) == 0)
        {
            std::cout << "Saved Segmented cloud : " << i << endl;
        }
        else PCL_ERROR("Problem saving ICP cloud.\n");

        cloud.reset( new pcl::PointCloud<pcl::PointXYZRGB> );
        cloudClusterVector.clear();
    }
}


void convertToPointCloud(){

        //load the RGB Images
        QStringList fileNamesRgb = QFileDialog::getOpenFileNames( 0, "Select RGB Images", "/home/gkerabat/Documents/Project/DataSets", "Images (*.png)", 0, QFileDialog::DontUseNativeDialog);
        std::vector<cv::Mat> imageRgbVector; //vector to store rgb images
        cv::Mat imageTemp;
        //loop to read images and store in vector
        for (int i = 0; i < fileNamesRgb.size(); i+=30){

            QString fileName = fileNamesRgb.at(i);
            imageTemp = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_UNCHANGED); //read images
            imageRgbVector.push_back( imageTemp ); //store in vector
            //std::stringstream stream;
            //stream << "/home/gkerabat/Documents/Project/DataSets/Kinect/Dril/SfM/image_" << i << ".jpg";
            //cv::imwrite(stream.str(), imageTemp);
            //std::cout << "Saved cloud : " << i << endl;
        }

        int j= 0;
        //load the Depth Images
        QStringList fileNamesDepth = QFileDialog::getOpenFileNames( 0, "Select Depth Images", "/home/gkerabat/Documents/Project/DataSets", "Images (*.png)", 0, QFileDialog::DontUseNativeDialog);
        std::vector<cv::Mat> imageDepthVector; //vector to store rgb images
        //loop to read images and store in vector
        for (int i = 0; i < fileNamesDepth.size(); i+=30){

            QString fileName = fileNamesDepth.at(i);
            imageTemp = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_UNCHANGED ); //read images
            imageDepthVector.push_back( imageTemp ); //store in vector
            j++;
        }


        std::cout << j << std::endl;
        DataGrabber dataGrabber(false, true, false);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
        //for all images of rgb and depth generate point clouds
        for (int i = 0; i < imageDepthVector.size(); i++){

            cloud = dataGrabber.rgbd2Pcl(imageRgbVector.at(i), imageDepthVector.at(i));

            std::stringstream stream;
            stream << "/home/gkerabat/Documents/Project/DataSets/Kinect/DrillNew/Registration1/RawPointClouds/cloud_" << i << ".pcd";
            std::string saveFileName = stream.str();

            if (pcl::io::savePCDFile( saveFileName, *cloud, true) == 0)
            {
                std::cout << "Saved cloud : " << i << endl;
            }
            else PCL_ERROR("Problem saving cloud.\n");
            cloud.reset( new pcl::PointCloud<pcl::PointXYZRGB> );
        }
}

void getIcp(){

    //get the pointclouds to align
    QStringList fileNames = QFileDialog::getOpenFileNames( 0, "Select point cloud", "/home/gkerabat/Documents/Project/DataSets/body/", "PointClouds (*.pcd)", 0, QFileDialog::DontUseNativeDialog);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > filteredCloudVector;

    //loop to read point clouds and store in vector
    for (int i = 0; i < fileNames.size(); i++){

        QString fileName = fileNames.at(i);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgbaFiltered ( new pcl::PointCloud<pcl::PointXYZRGB> );
        std::cout << fileName.toStdString() << std::endl ;
        //load the cloud
        if ( pcl::io::loadPCDFile( fileName.toStdString(), *cloudRgbaFiltered ) == -1 ){
            PCL_ERROR (" could not lad pcd file \n");
        }

        //store clouds in vector
        filteredCloudVector.push_back( cloudRgbaFiltered );
    }

    std::reverse(filteredCloudVector.begin(), filteredCloudVector.end());

    //object for RegMeshPcd class to perform ICP and Meshing
    RegMeshPcd regMeshPcd;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIcp ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudIcp = regMeshPcd.registerPointClouds( filteredCloudVector );

    if (pcl::io::savePCDFile( "/home/gkerabat/Documents/Project/DataSets/Kinect/DrillNew/Registration1/cloudIcp.pcd", *cloudIcp, true) == 0)
    {
        std::cout << "Saved ICP cloud" << std::endl;
    }
    else PCL_ERROR("Problem saving ICP cloud.\n");
}

void getVfh(){

    QStringList fileNamesPcd = QFileDialog::getOpenFileNames( 0, "Select Pointclouds", "/home/gkerabat/Documents/Project/DataSets", "PointClouds (*.pcd)", 0, QFileDialog::DontUseNativeDialog);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudVector;

    //load all pcd files and stack them in a vector
    //loop to read point clouds and store in vector
    for (int i = 0; i < fileNamesPcd.size(); i++){

        QString fileName = fileNamesPcd.at(i);
        std::cout << fileName.toStdString() << std::endl ;
        //load the cloud
        if ( pcl::io::loadPCDFile( fileName.toStdString(), *cloud ) == -1 ){
            PCL_ERROR (" could not lad pcd file \n");
        }

        //store clouds in vector
        cloudVector.push_back( cloud );
        cloud.reset( new pcl::PointCloud<pcl::PointXYZRGB> );
    }

    // apply tabletop to detect objects
    ObjectDetection objDet;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloudVfh ( new pcl::PointCloud<pcl::VFHSignature308> );

    for (int i = 0; i < cloudVector.size(); ++i){

        objDet.getVfhFeature( cloudVector.at(i), cloudVfh );

        //save segmented cloud
        std::stringstream stream;
        stream << "/home/gkerabat/Documents/Project/DataSets/Kinect/Random/VfhPointClouds/yellow_Vfh_" << i << ".pcd";
        std::string saveFileName = stream.str();
        if (pcl::io::savePCDFile( saveFileName, *cloudVfh, true) == 0)
        {
            std::cout << "Saved VFH cloud : " << i << endl;
        }
        else PCL_ERROR("Problem saving ICP cloud.\n");

        cloudVfh.reset( new pcl::PointCloud<pcl::VFHSignature308> );
    }
}

void transAndRotPcd( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn2 ){

    // *********************************** For translation and rotation of pointclouds ***************************

    std::cout << "start" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZRGB> ), cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud1AtOrigin(new pcl::PointCloud<pcl::PointXYZRGB>), cloud2AtOrigin(new pcl::PointCloud<pcl::PointXYZRGB>), cloud2AtOriginRotated ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::copyPointCloud(*cloudIn1, *cloud1);
    pcl::copyPointCloud(*cloudIn2, *cloud2);

    //bring both clouds to origin
    Eigen::Vector4f centroid1 (Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*cloud1, centroid1);
    Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
    Eigen::Vector3f origin(0, 0, 0);
    transform1.translation() = origin - centroid1.head<3>();
    pcl::transformPointCloud( *cloud1, *cloud1AtOrigin, transform1);
//    //save
//    if (pcl::io::savePCDFile( "/home/gkerabat/Documents/Project/DataSets/Kinect/DrillNew/Registration1/ICP/cloudIcpAtOrigin.pcd", *cloud1AtOrigin, true) == 0)
//    {
//        std::cout << "Saved rotated cloud" << "." << endl;
//    }
//    else PCL_ERROR("Problem saving ICP cloud.\n");

    Eigen::Vector4f centroid2 (Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*cloud2, centroid2);
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translation() = origin - centroid2.head<3>();
    pcl::transformPointCloud( *cloud2, *cloud2AtOrigin, transform2);

    //transform the second cloud
    Eigen::Vector3f eulerAngles(roll, pitch, yaw);
    cv::Mat rotMat = eulerAnglesToRotationMatrix(eulerAngles);
    Eigen::Matrix3f rotEig;
    rotEig << rotMat.at<float>(0,0), rotMat.at<float>(0,1), rotMat.at<float>(0,2), rotMat.at<float>(1,0), rotMat.at<float>(1,1), rotMat.at<float>(1,2), rotMat.at<float>(2,0), rotMat.at<float>(2,1), rotMat.at<float>(2,2);
    Eigen::Vector3f cloudMove(tX, tY, tZ);

    Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();
    transform3.translate(cloudMove);
    transform3.rotate(rotEig);

    pcl::transformPointCloud( *cloud2AtOrigin, *cloud2AtOriginRotated, transform3 );

    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("PCL Visualizer") );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0 );
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbCloud1( cloud1 );
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud1, rgbCloud1, "cloud1");
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcloud2( cloud2AtOrigin );
//    viewer->addPointCloud<pcl::PointXYZRGB> (cloud2AtOrigin, rgbcloud2, "cloud2");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcloud2Trans( cloud2AtOriginRotated );
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud2AtOriginRotated, rgbcloud2Trans, "cloud2trans");

    //viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1" );
    //viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud2" );
    //viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud2trans" );

    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get ());
    viewer->addCoordinateSystem( 0.3 );
    viewer->initCameraParameters();
    viewer->resetCamera();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        if(eventStatus){
            //transform the second cloud
            Eigen::Vector3f eulerAngles(roll, pitch, yaw);
            cv::Mat rotMat = eulerAnglesToRotationMatrix(eulerAngles);
            Eigen::Matrix3f rotEig;
            rotEig << rotMat.at<float>(0,0), rotMat.at<float>(0,1), rotMat.at<float>(0,2), rotMat.at<float>(1,0), rotMat.at<float>(1,1), rotMat.at<float>(1,2), rotMat.at<float>(2,0), rotMat.at<float>(2,1), rotMat.at<float>(2,2);
            Eigen::Vector3f cloudMove(tX, tY, tZ);

            Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();
            transform3.translate(cloudMove);
            transform3.rotate(rotEig);

            pcl::transformPointCloud( *cloud2AtOrigin, *cloud2AtOriginRotated, transform3 );

            if (pcl::io::savePCDFile( "/home/gkerabat/Documents/Project/DataSets/Kinect/DrillNew/Registration1/ICP/drillNewAxisAlignedOrigin.pcd", *cloud2AtOriginRotated, true) == 0)
            {
                std::cout << "Saved rotated cloud" << "." << endl;
            }
            else PCL_ERROR("Problem saving ICP cloud.\n");

            viewer->updatePointCloud<pcl::PointXYZRGB> (cloud1, rgbCloud1, "cloud1");
            //viewer->updatePointCloud<pcl::PointXYZRGB> (cloud2AtOrigin, rgbcloud2, "cloud2");
            viewer->updatePointCloud<pcl::PointXYZRGB> (cloud2AtOriginRotated, rgbcloud2Trans, "cloud2trans");
            //viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1" );
            //viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud2" );
            //viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud2trans" );

            eventStatus = false;
        }

    }
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void){

    if (event.getKeySym () == "a" && event.keyDown ()){
        roll += 2;
    }
    else if(event.getKeySym () == "b" && event.keyDown ()){
        roll -= 2;
    }
    else if(event.getKeySym () == "d" && event.keyDown ()){
        pitch += 2;
    }
    else if(event.getKeySym () == "h" && event.keyDown ()){
        pitch -= 2;
    }
    else if(event.getKeySym () == "i" && event.keyDown ()){
        yaw += 2;
    }
    else if(event.getKeySym () == "k" && event.keyDown ()){
        yaw -= 2;
    }
    else if(event.getKeySym () == "m" && event.keyDown ()){
        tX += 0.005;
    }
    else if(event.getKeySym () == "n" && event.keyDown ()){
        tX -= 0.005;
    }
    else if(event.getKeySym () == "o" && event.keyDown ()){
        tY += 0.005;
    }
    else if(event.getKeySym () == "t" && event.keyDown ()){
        tY -= 0.005;
    }
    else if(event.getKeySym () == "v" && event.keyDown ()){
        tZ += 0.005;
    }
    else if(event.getKeySym () == "x" && event.keyDown ()){
        tZ -= 0.005;
    }

    eventStatus = true;
    std::cout << roll << " " << pitch << " " << yaw << " " << tX << " " << tY << " " << tZ << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniteTwoClouds( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn2 ){

    // *********************************** make two clouds single ***********************************************
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

    *cloud3 = *cloudIn1 + *cloudIn2;

    return cloud3;
}

