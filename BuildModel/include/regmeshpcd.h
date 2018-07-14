#ifndef REGMESHPCD_H
#define REGMESHPCD_H

// ***************** Class to Register pointclouds and for a Mesh ************************************

#include "global.h"
#include "processingpcd.h"
#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_mod.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>


typedef pcl::PointXYZRGB PointTReg;


class RegMeshPcd
{
    float corrRejThreshNormAngle;
public:
    RegMeshPcd();

    //function to get ICP aligned pointcloud using single ICP
    pcl::PointCloud<PointTReg>::Ptr getIcp ( pcl::PointCloud<PointTReg>::Ptr p_cloudSource,
                                                          pcl::PointCloud<PointTReg>::Ptr p_cloudTarget,
                                                          float p_maxCorrDist, float p_ransacStatOutThresh, int p_maxIterations );

    //function to align pointclouds by performing icp twice
    pcl::PointCloud<PointTReg>::Ptr getIcp2 ( pcl::PointCloud<PointTReg>::Ptr p_cloudSource,
                                                           pcl::PointCloud<PointTReg>::Ptr p_cloudTarget,
                                                           float p_maxCorrDist1, float p_ransacStatOutThresh1, int p_maxIterations1,
                                                           float p_maxCorrDist2, float p_ransacStatOutThresh2, int p_maxIterations2);

    //function to get ICP aligned pointcloud using ICPwithNormals
    pcl::PointCloud<PointTReg>::Ptr getIcpNormal ( pcl::PointCloud<PointTReg>::Ptr p_cloudSource,
                                                                pcl::PointCloud<PointTReg>::Ptr p_cloudTarget,
                                                                float p_maxCorrDist, float p_ransacStatOutThresh, int p_maxIterations );

    //function to register pointclouds using ICPs
    pcl::PointCloud<PointTReg>::Ptr registerPointClouds ( std::vector<pcl::PointCloud<PointTReg>::Ptr> & cloudVector, float maxCorrDist, float corrRejThresh, int maxIter );

    //function to generate mesh
    pcl::PolygonMesh generateMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud );
};

#endif // REGMESHPCD_H
