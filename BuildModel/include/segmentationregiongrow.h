#ifndef SEGMENTATIONREGIONGROW_H
#define SEGMENTATIONREGIONGROW_H

#include "global.h"

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/passthrough.h>


typedef pcl::PointXYZRGB PointTSeg;

class SegmentationRegionGrow
{

    pcl::NormalEstimation<PointTSeg, pcl::Normal> normalEstimator;
    pcl::IndicesPtr indices;
    pcl::PassThrough<PointTSeg> pass;
    pcl::RegionGrowing<PointTSeg, pcl::Normal> reg;
    pcl::RegionGrowingRGB<PointTSeg> regRgb;

    pcl::PointCloud<PointTSeg>::Ptr cloudFilteredZ;

public:
    SegmentationRegionGrow();

    //function for segmentation using region growing based on normals
    pcl::PointCloud<PointTSeg>::Ptr getSegmentRegGrow( pcl::PointCloud<PointTSeg>::Ptr p_cloudInput );

    //function for segmentation using region growing based on colors
    pcl::PointCloud<PointTSeg>::Ptr getSegmentRegGrowRgb( pcl::PointCloud<PointTSeg>::Ptr p_cloudInput );
};

#endif // SEGMENTATIONREGIONGROW_H
