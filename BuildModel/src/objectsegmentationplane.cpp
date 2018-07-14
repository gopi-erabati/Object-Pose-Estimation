#include "../include/objectsegmentationplane.h"

ObjectSegmentationPlane::ObjectSegmentationPlane()
{
}

//function to passthrough and downsample cloud if required
pcl::PointCloud<PointTObj>::Ptr ObjectSegmentationPlane::getFiltered( pcl::PointCloud<PointTObj>::Ptr p_cloudInput ){

    pcl::PointCloud<PointTObj>::Ptr cloudOutput ( new pcl::PointCloud<PointTObj> );

    //apply passthrough filter
    ProcessingPcd processingPcd;
    // (X -0.2 0.3 , Y -0.5 0.1,  Z 0.5 1.05 Brick and Dril) (random X -0.2 0.3 Y -0.5 0.3, Z 0.5 1.2) KINECT

    // (X -0.5 0.5, Y -0.5 0.3, Z 0.5 1.6 ASTRA BRICK)
    float minX = -0.4, maxX = 0.6, minY = -0.5, maxY = 0.5, minZ = 0.7, maxZ = 1.4;
    //float minX = -0.2, maxX = 0.4, minY = -0.5, maxY = 0.1, minZ = 0.5, maxZ = 1.2;

    cloudOutput =  processingPcd.getPassThrough( p_cloudInput, minX, maxX, minY, maxY, minZ, maxZ);
    //cloudOutput = processingPcd.getDownSampled(cloudOutput, 0.001);

    std::stringstream ss;
    ss << "/home/gkerabat/Documents/Project/DataSets/Kinect/Bottle/SegPointClouds/cloud1.pcd";
    if (pcl::io::savePCDFile( ss.str(), *cloudOutput, true) == 0)
    {
        std::cout << "Saved filtered cloud" << "." << endl;
    }
    else PCL_ERROR("Problem saving cluster cloud.\n");

    //apply downsample
//    double leafSize = 0.01;
//    cloudOutput = processingPcd.getDownSampled( p_cloudInput, leafSize);

    return cloudOutput;
}

//function to do planar segmentation basing on pcl::SACSegmentation and return the indices of plane and coeff.
bool ObjectSegmentationPlane::getPlaneIndicesAndCoeffSAC( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::ModelCoefficients::Ptr p_modelCoeff ){


    sacSeg.setOptimizeCoefficients( true );
    sacSeg.setInputCloud( p_cloudInput );
    sacSeg.setModelType( pcl::SACMODEL_PLANE );
    sacSeg.setMethodType( pcl::SAC_RANSAC );
    sacSeg.setDistanceThreshold( 0.01 ); //low gives good results 0.02
    sacSeg.setAxis( Eigen::Vector3f( 0.0, 1.0, 0.0 ) );
    sacSeg.setEpsAngle( 60.0f * (3.14/180.0f));
    sacSeg.segment( *p_indices, *p_modelCoeff );

    if (p_modelCoeff->values.size() == 0){
        std::cout << "No Plane Detected";
        return false;
    }

    return true;

}

//function to extract plane and non-plane cloud
void ObjectSegmentationPlane::getPlaneAndNonPlaneCloud( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::PointCloud<PointTObj>::Ptr p_cloudPlane, pcl::PointCloud<PointTObj>::Ptr p_cloudNotPlane ){


    extractIndices.setInputCloud( p_cloudInput );
    extractIndices.setIndices( p_indices );

    //extract planar cloud
    extractIndices.setNegative( false );
    extractIndices.filter( *p_cloudPlane );

    //extract rest of cloud
    extractIndices.setNegative( true );
    extractIndices.filter( *p_cloudNotPlane );
}

//function to perform euclidean clustering and return cluster indices
std::vector<pcl::PointIndices> ObjectSegmentationPlane::getClusters (pcl::PointCloud<PointTObj>::Ptr p_cloudInput ){

    //create a kdTree for Nearest Neighbour search
    pcl::search::KdTree<PointTObj>::Ptr kdTree ( new pcl::search::KdTree<PointTObj> );
    kdTree->setInputCloud( p_cloudInput );

    std::vector<pcl::PointIndices> clusterIndices;
    euclideanClustExtraction.setInputCloud( p_cloudInput );
    euclideanClustExtraction.setSearchMethod( kdTree );
    euclideanClustExtraction.setClusterTolerance( 0.1 ); //0.02 for brick //0.5 //less tolerance more clusters
    euclideanClustExtraction.setMinClusterSize( 100 );
    euclideanClustExtraction.setMaxClusterSize( 1e5 );
    euclideanClustExtraction.extract( clusterIndices );

    return clusterIndices;

}



//function to project inliers into plane
pcl::PointCloud<PointTObj>::Ptr ObjectSegmentationPlane::getProjectedCloud ( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::ModelCoefficients::Ptr p_modelCoeff ){

    pcl::PointCloud<PointTObj>::Ptr cloudOutput ( new pcl::PointCloud<PointTObj> );

    pcl::ProjectInliers<PointTObj> projectInliers;
    projectInliers.setInputCloud( p_cloudInput );
    projectInliers.setModelType( pcl::SACMODEL_PLANE );
    projectInliers.setIndices( p_indices );
    projectInliers.setModelCoefficients( p_modelCoeff );
    projectInliers.filter( *cloudOutput );

    return cloudOutput;
}

//function to create convex hull
pcl::PointCloud<PointTObj>::Ptr ObjectSegmentationPlane::getConvexHull ( pcl::PointCloud<PointTObj>::Ptr p_cloudInput ){

    pcl::PointCloud<PointTObj>::Ptr cloudOutput ( new pcl::PointCloud<PointTObj> );

    pcl::ConvexHull<PointTObj> convexHull;
    convexHull.setInputCloud( p_cloudInput );
    convexHull.reconstruct( *cloudOutput );

    return cloudOutput;
}

//function to take and point cloud and return vector of pointcloud clusters using ConvexHull
bool ObjectSegmentationPlane::getSegmentedObjectsOnPlane( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, std::vector<pcl::PointCloud<PointTObj>::Ptr> &cloudClusterVector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_cloudPlane){

    /* ************************************************
     * 1. Get the point cloud filtered
     * 2. Get plane indices and Coefficients
     * 3. Project inliers on plane and get projected cloud
     * 4. Get convex Hull of projected cloud
     * 5. Get polygonal prism data of filtered cloud using convex hull, to get objects attached to plane
     * 6. Now, we have plane and associated objects, so again apply plane detction and get indices.
     * 7. extract indices which doesnot belong to plane to get the segmented objects on plane
     * 8. apply euclidean clustering to segment each object individually. */
    /* *************************************************/

    //std::vector<pcl::PointCloud<PointTObj>::Ptr> cloudClusterVector;
    pcl::PointCloud<PointTObj>::Ptr cloudFiltered ( new pcl::PointCloud<PointTObj> );
    pcl::PointCloud<PointTObj>::Ptr cloudProjected ( new pcl::PointCloud<PointTObj> );
    pcl::PointCloud<PointTObj>::Ptr cloudHull ( new pcl::PointCloud<PointTObj> );
    pcl::PointCloud<PointTObj>::Ptr cloudplane ( new pcl::PointCloud<PointTObj> );
    pcl::PointCloud<PointTObj>::Ptr cloudNotPlane ( new pcl::PointCloud<PointTObj> );
    p_cloudPlane = pcl::PointCloud<PointTObj>::Ptr ( new pcl::PointCloud<PointTObj> );


    //filter the cloud
    //cloudFiltered = getFiltered( p_cloudInput );
    //pcl::copyPointCloud( *cloudFiltered, *cloudClusterFirst); return true;
    pcl::copyPointCloud( *p_cloudInput, *cloudFiltered);

    pcl::ModelCoefficients::Ptr modelCoefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr indicesPoint ( new pcl::PointIndices );

    //get plane indices and coeff
    bool isPlane = getPlaneIndicesAndCoeffSAC( cloudFiltered, indicesPoint, modelCoefficients );

    if (!isPlane){
        pcl::copyPointCloud( *cloudFiltered, *cloudClusterVector.at(0));
        return false;
    }

    // ****** Processes cloud which is not plane to only get object clusters present on plane *****
    //project inliers on plane and get projected cloud
    cloudProjected = getProjectedCloud( cloudFiltered, indicesPoint, modelCoefficients );


    //get the convex hull of projected cloud
    cloudHull = getConvexHull( cloudProjected );

    //get extreme four corners of hull
    std::vector<float> vectorX, vectorY, vectorZ;

    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D (*cloudHull, minPt, maxPt);

    vectorX.push_back(minPt.x - 0.1); vectorY.push_back(minPt.y - 0.1);
    vectorX.push_back(minPt.x - 0.1); vectorY.push_back(maxPt.y + 0.1);
    vectorX.push_back(maxPt.x + 0.1); vectorY.push_back(maxPt.y + 0.1);
    vectorX.push_back(maxPt.x + 0.1); vectorY.push_back(minPt.y - 0.1);

    //calculate Z from plane equation
    float a = modelCoefficients->values.at(0);
    float b = modelCoefficients->values.at(1);
    float c = modelCoefficients->values.at(2);
    float d = modelCoefficients->values.at(3);
    for(int i=0; i < 4; ++i){
        float x = vectorX.at(i);
        float y = vectorY.at(i);
        float z = -((a*x) + (b*y) + d)/c;
        vectorZ.push_back(z);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHullNew ( new pcl::PointCloud<pcl::PointXYZRGB> );
    convexHullNew->width = 4;
    convexHullNew->height = 1;
    convexHullNew->is_dense = false;
    convexHullNew->points.resize(4);
    for (int i = 0; i < 4 ; ++i){
        convexHullNew->points[i].x = vectorX.at(i);
        convexHullNew->points[i].y = vectorY.at(i);
        convexHullNew->points[i].z = vectorZ.at(i);
        //Color
        uint8_t r = 255;
        uint8_t g = 0;
        uint8_t b = 0;
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        convexHullNew->points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }


    //get polygonal prism data
    pcl::PointIndices::Ptr cloudIndices ( new pcl::PointIndices );
    pcl::ExtractPolygonalPrismData<PointTObj> extractPolyData;
    extractPolyData.setInputCloud( cloudFiltered );
    extractPolyData.setInputPlanarHull( convexHullNew );
    extractPolyData.segment( *cloudIndices );

    //form the cloud
    pcl::PointCloud<PointTObj>::Ptr cloudObjWithPlane (new pcl::PointCloud<PointTObj>);
    for (std::vector<int>::const_iterator pit = cloudIndices->indices.begin (); pit != cloudIndices->indices.end (); ++pit){
        cloudObjWithPlane->points.push_back (cloudFiltered->points[*pit]);
    }
    cloudObjWithPlane->width = cloudObjWithPlane->points.size ();
    cloudObjWithPlane->height = 1;
    cloudObjWithPlane->is_dense = true;

    //get plane and not plane cloud
    //get plane indices and coeff
    isPlane = getPlaneIndicesAndCoeffSAC( cloudObjWithPlane, indicesPoint, modelCoefficients );

    if (!isPlane){
        pcl::copyPointCloud( *cloudFiltered, *cloudClusterVector.at(0));
        return false;
    }

    getPlaneAndNonPlaneCloud( cloudObjWithPlane, indicesPoint, cloudplane, cloudNotPlane );
    pcl::copyPointCloud(*cloudplane, *p_cloudPlane);

    //get clusters for the not plane cloud
    std::vector<pcl::PointIndices> clusterIndices;
    clusterIndices = getClusters( cloudNotPlane );

    std::cout << "number of clusters detected :" << clusterIndices.size() << std::endl;
    //cloudClusterFirst = pcl::PointCloud<PointTObj>::Ptr  ( new pcl::PointCloud<PointTObj> );

    //save clusters
    int j = 0;
    long int previousCloudSize = 1; //some small number
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {

        pcl::PointCloud<PointTObj>::Ptr cloudCluster (new pcl::PointCloud<PointTObj>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloudCluster->points.push_back (cloudNotPlane->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

//        if (cloudCluster->points.size() < 0.3 * previousCloudSize ){
//            continue;
//        }

//        if ( j== 0){
//            *cloudClusterFirst = *cloudCluster;
//        }

//        std::stringstream ss;
//        ss << "/home/gkerabat/Documents/Project/DataSets/Kinect/Brick/StuffExtra/cloudCluster_" << j << ".pcd";
//        if (pcl::io::savePCDFile( ss.str(), *cloudCluster, true) == 0)
//        {
//            std::cout << "Saved cluster cloud " << j << endl;
//        }
//        else PCL_ERROR("Problem saving cluster cloud.\n");

        //save to vector
        cloudClusterVector.push_back( cloudCluster );
        j++;
        previousCloudSize = cloudCluster->points.size();
    }

    return true;

}


//function to take and point cloud and return vector of pointcloud clusters
std::vector<pcl::PointCloud<PointTObj>::Ptr> ObjectSegmentationPlane::getSegmentedObjectsExceptPlane( pcl::PointCloud<PointTObj>::Ptr p_cloudInput ){


    //filter the cloud
    pcl::PointCloud<PointTObj>::Ptr cloudFiltered = getFiltered( p_cloudInput );

    std::vector<pcl::PointCloud<PointTObj>::Ptr> cloudClusterVector;
    pcl::PointCloud<PointTObj>::Ptr cloudTemp ( new pcl::PointCloud<PointTObj> );
    pcl::PointCloud<PointTObj>::Ptr cloudplane ( new pcl::PointCloud<PointTObj> );
    pcl::PointCloud<PointTObj>::Ptr cloudNotPlane ( new pcl::PointCloud<PointTObj> );
    *cloudTemp = *cloudFiltered;

    long int nr_points = (long int) cloudTemp->points.size();

    while( cloudTemp->points.size() > 0.3 * nr_points ){

        pcl::ModelCoefficients::Ptr modelCoefficients ( new pcl::ModelCoefficients );
        pcl::PointIndices::Ptr indicesPoint ( new pcl::PointIndices );

        //get plane indices and coeff
        bool plane = getPlaneIndicesAndCoeffSAC( cloudTemp, indicesPoint, modelCoefficients );

        if (indicesPoint->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }


        //get plane and not plane cloud
        getPlaneAndNonPlaneCloud( cloudTemp, indicesPoint, cloudplane, cloudNotPlane );

        *cloudTemp = *cloudNotPlane;
    }
    *cloudNotPlane = *cloudTemp;

    //get clusters for the not plane cloud
    std::vector<pcl::PointIndices> clusterIndices;
    clusterIndices = getClusters( cloudNotPlane );

    std::cout << clusterIndices.size() << std::endl;


    //save clusters
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {

        pcl::PointCloud<PointTObj>::Ptr cloudCluster (new pcl::PointCloud<PointTObj>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloudCluster->points.push_back (cloudNotPlane->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

//        std::stringstream ss;
//        ss << "../DataSets/objseg/cloudCluster_" << j << ".pcd";
//        if (pcl::io::savePCDFile( ss.str(), *cloudCluster, true) == 0)
//        {
//            std::cout << "Saved cluster cloud" << "." << endl;
//        }
//        else PCL_ERROR("Problem saving ICP cloud.\n");



        cloudClusterVector.push_back( cloudCluster );
        j++;
    }

    return cloudClusterVector;

}
