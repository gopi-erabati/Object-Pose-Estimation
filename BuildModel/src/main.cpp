#include <string>
#include <pcl/common/common.h>
#include "global.h"
#include "regmeshpcd.h"
#include "rosinterface.h"

int
main(int argc, char** argv)
{
    // ******************************** Command line parser for arguments *************************************
    cv::CommandLineParser parser(argc, argv,
                             #if CV_MAJOR_VERSION == 3
                                 "{ h help      | false                      | print this message   }"
                                 "{ o object    |                            | Provide the name of object }"
                                 "{ d directory | empty                      | Provide a directory of point clouds to form a model}"
                                 "{ l limits    | -0.2,0.3,-0.5,0.1,0.5,1.05 | X, Y and Z limits from camera(xmin,xmax,ymin,ymax,zmin,zmax)(no spaces!)  }"
                                 "{ s sensor    | kinect                     | 3D Sensor(kinect,astra,euclid)}"
                                 "{ cd corrdist | 0.005                      | Maximum Correspondence Distance (in m) for correspondence estimation}"
                                 "{ t threshold | 0.7                        | Correspondence Rejection Threhold (arccos of angle between correspondences)}"
                                 "{ m maxiter   | 500                        | Maximum Iterations for ICP to converge}"
                             #else
                                 "{ h  | help      | false                      | print this message   }"
                                 "{ o  | object    |                            | Provide the name of object }"
                                 "{ d  | directory | empty                      | Provide a directory of point clouds to form a model}"
                                 "{ l  | limits    | -0.2,0.3,-0.5,0.1,0.5,1.05 | X, Y and Z limits from camera(xmin,xmax,ymin,ymax,zmin,zmax)(no spaces!)  }"
                                 "{ s  | sensor    | kinect                     | 3D Sensor(kinect,astra,euclid)}"
                                 "{ cd | corrdist  | 0.005                      | Maximum Correspondence Distance (in m) for correspondence estimation}"
                                 "{ t  | threshold | 0.7                        | Correspondence Rejection Threhold (arccos of angle between correspondences)}"
                                 "{ m  | maxiter   | 500                        | Maximum Iterations for ICP to converge}"
                             #endif
                                 );

    // ******************************** Get Values of Different Arguments *************************************
    //print help
    bool helpReq = parser.get<bool>("h");
    if ( helpReq )
    {
#if CV_MAJOR_VERSION == 3
        parser.printMessage();
#else
        parser.printParams();
#endif
        //parser.printMessage();
        return 0;
    }

    //to get different arguments
    std::string objName = parser.get<std::string>("o"); //object name to form the model
    std::string sensor = parser.get<std::string>("s"); // sensor type used
    float corrDist = parser.get<float>("cd"); // correspondence distance for ICP
    float rejThresh = parser.get<float>("t"); // COrrespondence rejection threshold for ICP
    int maxIter = parser.get<int>("m"); // max iterations for ICP

    std::string limits = parser.get<std::string> ("l");
    std::vector<std::string> fieldString;
    boost::split( fieldString, limits, boost::is_any_of( "," ) );
    //std::cout << boost::lexical_cast<float>( fieldString.at(0)) << " , " << fieldString.at(5) << std::endl;
    std::vector<float> limitsD;
    for(int i = 0; i < fieldString.size(); ++i){
        limitsD.push_back(boost::lexical_cast<float>( fieldString.at(i)));
    }

    std::string pathToPcd = parser.get<std::string>("d");

    // ********************* Scanning or Loading Point Clouds ***********************************************
    //check if path to pcd files is given otherwise start capturing frames
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cloudVector;
    if(pathToPcd == "empty"){
        std::cout << "Start Scanning, press 'c' to capture and 'f' to finish capturing" << std::endl;
        //sensor types
        bool kinect = false; bool astra = false; bool euclid = false;
        if (sensor == "kinect"){
            kinect = true;
        }
        else if (sensor == "astra"){
            astra = true;
        }
        else if (sensor == "euclid"){
            euclid = true;
        }
        else{
            std::cerr << "Invalid Sensor Type" ;
            return 0;
        }

        //ROS INTERFACE
        RosInterface rosInterface(euclid, kinect, astra);
        bool status = rosInterface.startScan(argc, argv, cloudVector);
        std::cout << "Finished capturing point clouds!" << std::endl;

        std::cout << "Saving captured point clouds ..." << std::endl;
        //save clouds to disk
        std::stringstream pathToSavePcd;
        pathToSavePcd << "../" << objName;
        boost::filesystem::path pathToSavePcdBoost(pathToSavePcd.str());
        boost::filesystem::create_directory(pathToSavePcdBoost);

        for(int i = 0; i < cloudVector.size(); ++i){

            std::stringstream ss1;
            ss1 << "../" << objName << "/" << objName << i << ".pcd";
            std::string cloudSaveName = ss1.str();
            if (pcl::io::savePCDFile( cloudSaveName, *cloudVector.at(i), true) == 0)
            {
                //std::cout << "Saved scanned cloud" << i << endl;
            }
            else PCL_ERROR("Problem saving cluster cloud.\n");
        }
        std::cout << "Saved " << cloudVector.size() << " point clouds" << std::endl;
    }
    else{

        std::cout << "Loading point clouds ..." << std::endl;
        boost::filesystem::path pathToPcdBoost(pathToPcd);
        try{
            //check path to directory exists and is directory
            if(boost::filesystem::exists(pathToPcdBoost) && boost::filesystem::is_directory(pathToPcdBoost)){

                boost::filesystem::directory_iterator iter(pathToPcdBoost);
                boost::filesystem::directory_iterator end;

                while( iter != end){

                    boost::filesystem::path filePath(iter->path()); //get file path
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn ( new pcl::PointCloud<pcl::PointXYZRGB> );

                    if(filePath.extension() == ".pcd"){
                        //load the cloud
                        if ( pcl::io::loadPCDFile( filePath.string(), *cloudIn ) == -1 ){
                            PCL_ERROR (" could not load pcd file \n");
                        }
                    }
                    else if ( filePath.extension() == ".ply" ){
                        //load the cloud
                        if ( pcl::io::loadPLYFile( filePath.string(), *cloudIn ) == -1 ){
                            PCL_ERROR (" could not load pcd file \n");
                        }
                    }
                    else{
                        continue;
                    }
                    cloudVector.push_back(cloudIn);
                    iter++;

                }
            }
        }
        catch (boost::filesystem::filesystem_error &e){
            std::cerr << e.what() << std::endl;
            return 0;
        }
        std::cout << "Finished loading " << cloudVector.size() << " point clouds!" << std::endl;
    }

    // ********************* Plane Segmentation of clouds ***********************************************
    std::cout << "Segmenting point clouds to clutser only the object..." << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusterVector;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane (new pcl::PointCloud<pcl::PointXYZRGB> ), cloudTargetFiltered (new pcl::PointCloud<pcl::PointXYZRGB> );
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cloudVectorSeg;

    ObjectSegmentationPlane objSegPlane;
    ProcessingPcd procPcd;

    //create viewer to visualize
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("Segmentation Window"));
    viewer->setBackgroundColor( 0.0, 0.0, 0.0);
    viewer->addCoordinateSystem( 0.3 );
    viewer->initCameraParameters();
    viewer->resetCamera();

    for (int i = 0; i < cloudVector.size(); ++i){

        //show cloud
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbOriginal( cloudVector.at(i) );
        viewer->addPointCloud<pcl::PointXYZRGB> (cloudVector.at(i), rgbOriginal, "CloudOriginal");

        std::cout << cloudVector.at(i)->points.size() << std::endl;
        //segmentation
        cloudClusterVector.clear(); cloudPlane->clear(); cloudTargetFiltered->clear();
        cloudTargetFiltered = procPcd.getPassThrough(cloudVector.at(i), limitsD.at(0), limitsD.at(1), limitsD.at(2), limitsD.at(3), limitsD.at(4), limitsD.at(5) );
        bool isPlane = objSegPlane.getSegmentedObjectsOnPlane( cloudTargetFiltered, cloudClusterVector, cloudPlane );
        cloudVectorSeg.push_back(cloudClusterVector.at(0));

        //show plane and segmented object
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcloudPlane( cloudPlane );
        viewer->addPointCloud<pcl::PointXYZRGB>( cloudPlane, rgbcloudPlane, "CloudPlane" );
        viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "CloudPlane");

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbcloudObj( cloudClusterVector.at(0) );
        viewer->addPointCloud<pcl::PointXYZRGB>( cloudClusterVector.at(0), rgbcloudObj, "CloudObj" );
        viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "CloudObj");

        viewer->spinOnce(1000);
        //remove point clouds
        viewer->removeAllPointClouds();


    }
    std::cout << "Finished segmentation of " << cloudVectorSeg.size() << " point clouds!" << std::endl;
    viewer->close();

    // ********************* Alignment of Clouds ***********************************************
    std::cout << "Aliging the point clouds to build model ..." << std::endl;

    RegMeshPcd regMeshPcd;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudAligned ( new pcl::PointCloud<pcl::PointXYZRGB> );
    cloudAligned = regMeshPcd.registerPointClouds(cloudVectorSeg, corrDist, rejThresh, maxIter);

    std::cout << "Finished aligining point clouds - Model Formed!" << std::endl;

    //Save Aligned cloud
    std::stringstream pathToSavePcd1;
    pathToSavePcd1 << "../" << objName;
    boost::filesystem::path pathToSavePcdBoost1(pathToSavePcd1.str());
    if( !boost::filesystem::exists(pathToSavePcdBoost1)){
        boost::filesystem::create_directory(pathToSavePcdBoost1);
    }
    std::stringstream ss;
    ss << "../" << objName << "/" << objName << "Aligned.pcd";
    std::string cloudAlignedFileName = ss.str();
    if (pcl::io::savePCDFile( cloudAlignedFileName, *cloudAligned, true) == 0)
    {
        std::cout << "Saved Aligned Cloud at " << cloudAlignedFileName << "." << std::endl;
    }
    else PCL_ERROR("Problem saving %s.\n", cloudAlignedFileName.c_str());

    // *********************Meshing of Aligned Cloud ***********************************************
    std::cout << "Meshing the model ..." << std::endl;
    pcl::PolygonMesh cloudMesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAlignedXYZ ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::copyPointCloud(*cloudAligned, *cloudAlignedXYZ);
    cloudMesh = regMeshPcd.generateMesh(cloudAlignedXYZ);
    std::cout << "Meshing finished!!!" << std::endl;

    std::stringstream ssMesh;
    ssMesh << "../" << objName << "/" << objName << "Mesh.vtk";
    std::string meshFileName = ssMesh.str();
    pcl::io::saveVTKFile (meshFileName, cloudMesh);
    std::cout << "Saved Mesh at " << meshFileName << "." << std::endl;

    return 0;
}


