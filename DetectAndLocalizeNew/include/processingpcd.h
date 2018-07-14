#ifndef PROCESSINGPCD_H
#define PROCESSINGPCD_H

// ***************** Class to apply Processing to Point Clouds ************************************

#include "global.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>

typedef pcl::PointXYZRGB PointTProc;


class ProcessingPcd
{
public:
    ProcessingPcd();

    //function to apply pass through filter to pointclouds in X, Y and Z limits
    pcl::PointCloud<PointTProc>::Ptr getPassThrough( pcl::PointCloud<PointTProc>::Ptr p_cloud,
                                                            float p_minX, float p_maxX, float p_minY, float p_maxY, float p_minZ, float p_maxZ );

    //function to apply voxel grid downsampling if required
    pcl::PointCloud<PointTProc>::Ptr getDownSampled( pcl::PointCloud<PointTProc>::Ptr p_cloud, float p_leafSize);

    //function to remove outliers
    pcl::PointCloud<PointTProc>::Ptr getOutlierRemove( pcl::PointCloud<PointTProc>::Ptr p_cloud, float p_threshold);

    //function to perform smoothing of pc
    pcl::PointCloud<PointTProc>::Ptr getSmooth( pcl::PointCloud<PointTProc>::Ptr p_cloud, float p_searchRadius);

    //function to filter based on color
    pcl::PointCloud<PointTProc>::Ptr getFilterRgb( pcl::PointCloud<PointTProc>::Ptr p_cloud, int p_minR, int p_maxR, int p_minG, int p_maxG, int p_minB, int p_maxB);
};

#endif // PROCESSINGPCD_H
