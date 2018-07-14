#ifndef OBJECTSEGMENTATIONPLANE_H
#define OBJECTSEGMENTATIONPLANE_H

// *************** Class to do clustering of Objects on a Plane *********************************

#include <global.h>
#include <processingpcd.h>
//for getting plane indices and coeff using sac segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
//for extracting pc from pc using indices
#include <pcl/filters/extract_indices.h>
//for euclidean clusters
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
//for projecting
#include <pcl/filters/project_inliers.h>
//for convex hull
#include <pcl/surface/convex_hull.h>
//to get polygonal data
#include <pcl/segmentation/extract_polygonal_prism_data.h>

typedef pcl::PointXYZRGB PointTObj;

//class to segment objects on top of table and return the clusters of objects
class ObjectSegmentationPlane
{

    //create SacSgementation object
    pcl::SACSegmentation<PointTObj> sacSeg;

    //create Extract Indices Object
    pcl::ExtractIndices<PointTObj> extractIndices;

    //create cluster extraction object
    pcl::EuclideanClusterExtraction<PointTObj> euclideanClustExtraction;

public:
    ObjectSegmentationPlane();

    //function to passthrough and downsample cloud if required
    pcl::PointCloud<PointTObj>::Ptr getFiltered( pcl::PointCloud<PointTObj>::Ptr p_cloudInput );

    //function to do planar segmentation basing on pcl::SACSegmentation and return the indices of plane and coeff.
    bool getPlaneIndicesAndCoeffSAC( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::ModelCoefficients::Ptr p_modelCoeff );

    //function to extract plane and non-plane cloud
    void getPlaneAndNonPlaneCloud( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::PointCloud<PointTObj>::Ptr p_cloudPlane, pcl::PointCloud<PointTObj>::Ptr p_cloudNotPlane );

    //function to perform euclidean clustering and return cluster indices
    std::vector<pcl::PointIndices> getClusters (pcl::PointCloud<PointTObj>::Ptr p_cloudInput );

    //function to take and point cloud and return vector of pointcloud clusters excpet plane
    std::vector<pcl::PointCloud<PointTObj>::Ptr> getSegmentedObjectsExceptPlane( pcl::PointCloud<PointTObj>::Ptr p_cloudInput );

    //function to project inliers into plane
    pcl::PointCloud<PointTObj>::Ptr getProjectedCloud ( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::ModelCoefficients::Ptr p_modelCoeff );

    //function to create convex hull
    pcl::PointCloud<PointTObj>::Ptr getConvexHull ( pcl::PointCloud<PointTObj>::Ptr p_cloudInput );

    //function to take and point cloud and return vector of pointcloud clusters using ConvexHull
    bool getSegmentedObjectsOnPlane( pcl::PointCloud<PointTObj>::Ptr p_cloudInput, std::vector<pcl::PointCloud<PointTObj>::Ptr> &cloudClusterVector, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_cloudPlane );
};

#endif // OBJECTSEGMENTATIONPLANE_H
