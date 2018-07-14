#include "segmentationregiongrow.h"

SegmentationRegionGrow::SegmentationRegionGrow()
{
}


//function for segmentation using region growing based on normals
pcl::PointCloud<PointTSeg>::Ptr SegmentationRegionGrow::getSegmentRegGrow( pcl::PointCloud<PointTSeg>::Ptr p_cloudInput){

    pcl::search::KdTree<PointTSeg>::Ptr tree ( new pcl::search::KdTree<PointTSeg> );
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);


    cloudFilteredZ = pcl::PointCloud<PointTSeg>::Ptr  (new pcl::PointCloud<PointTSeg> );

    pass.setInputCloud (p_cloudInput);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.2);
    pass.filter (*cloudFilteredZ);


    normalEstimator.setSearchMethod (tree);
    normalEstimator.setInputCloud (cloudFilteredZ);
    normalEstimator.setKSearch (30);
    normalEstimator.compute (*normals);

    reg.setMinClusterSize (500);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (15);
    reg.setInputCloud (cloudFilteredZ);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusterIndices;
    reg.extract (clusterIndices);

    std::cout << "number of clusters detected :" << clusterIndices.size() << std::endl;
    pcl::PointCloud<PointTSeg>::Ptr cloudClusterFirst ( new pcl::PointCloud<PointTSeg> );

    //save clusters
    int j = 0;
    long int previousCloudSize = 1; //some small number
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {

        pcl::PointCloud<PointTSeg>::Ptr cloudCluster (new pcl::PointCloud<PointTSeg>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloudCluster->points.push_back (p_cloudInput->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        //          if (cloudCluster->points.size() < 0.3 * previousCloudSize ){
        //              continue;
        //          }

        //          if ( j== 0){
        *cloudClusterFirst = *cloudCluster;
        //          }

        std::stringstream ss;
        ss << "/home/gkerabat/Documents/Project/DataSets/Kinect/Brick/Test_NotPlane_16-28-42/SegPointClouds/cloudCluster_" << j << ".pcd";
        if (pcl::io::savePCDFile( ss.str(), *cloudCluster, true) == 0)
        {
            std::cout << "Saved cluster cloud" << "." << endl;
        }
        else PCL_ERROR("Problem saving cluster cloud.\n");

        //save to vector
        //          cloudClusterVector.push_back( cloudCluster );
        j++;
        //          previousCloudSize = cloudCluster->points.size();
    }

    return cloudClusterFirst;

}

//function for segmentation using region growing based on colors
pcl::PointCloud<PointTSeg>::Ptr SegmentationRegionGrow::getSegmentRegGrowRgb( pcl::PointCloud<PointTSeg>::Ptr p_cloudInput ){

    pcl::search::KdTree<PointTSeg>::Ptr tree ( new pcl::search::KdTree<PointTSeg> );

    cloudFilteredZ = pcl::PointCloud<PointTSeg>::Ptr  (new pcl::PointCloud<PointTSeg> );
    indices = pcl::IndicesPtr ( new std::vector <int> );

    pass.setInputCloud (p_cloudInput);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.1);
    pass.filter (*indices);

    regRgb.setInputCloud (p_cloudInput);
    regRgb.setIndices (indices);
    regRgb.setSearchMethod (tree);
    regRgb.setDistanceThreshold (0.1);
    regRgb.setPointColorThreshold (10);
    regRgb.setRegionColorThreshold (10);
    regRgb.setMinClusterSize (700);

    std::vector <pcl::PointIndices> clusterIndices;
    regRgb.extract (clusterIndices);

    std::cout << "number of clusters detected :" << clusterIndices.size() << std::endl;
    pcl::PointCloud<PointTSeg>::Ptr cloudClusterFirst ( new pcl::PointCloud<PointTSeg> );

    //save clusters
    int j = 0;
    long int previousCloudSize = 1; //some small number
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {

        pcl::PointCloud<PointTSeg>::Ptr cloudCluster (new pcl::PointCloud<PointTSeg>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloudCluster->points.push_back (p_cloudInput->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        //          if (cloudCluster->points.size() < 0.3 * previousCloudSize ){
        //              continue;
        //          }

        //          if ( j== 0){
        *cloudClusterFirst = *cloudCluster;
        //          }

        std::stringstream ss;
        ss << "/home/gkerabat/Documents/Project/DataSets/Kinect/Brick/Test_NotPlane_16-28-42/SegPointClouds/cloudCluster_" << j << ".pcd";
        if (pcl::io::savePCDFile( ss.str(), *cloudCluster, true) == 0)
        {
            std::cout << "Saved cluster cloud" << "." << endl;
        }
        else PCL_ERROR("Problem saving cluster cloud.\n");

        //save to vector
        //          cloudClusterVector.push_back( cloudCluster );
        j++;
        //          previousCloudSize = cloudCluster->points.size();
    }

    return cloudClusterFirst;



}
