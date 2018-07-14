#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

// ********************** Class for Coarse and Fine Pose Estimation of Objects using Model ************************

#include "global.h"

#if (PCL_MINOR_VERSION >= 7 && PCL_REVISION_VERSION >= 2)
#include <pcl/registration/icp_mod.h>
#include <pcl/registration/correspondence_estimation_mod.h>
#else
#include <pcl/registration/icp_modCorr.h>
#include <pcl/registration/correspondence_estimation.h>
#endif

//for keypoint and feature descriptor
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/search/kdtree.h>
//for correspondence estimation
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
//for Rejection of false correspondences
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_self_occluded_normal.h>
//for transofrmation estimation
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_lm.h>
//for keypoints
#include <pcl/keypoints/iss_3d.h>

#include <pcl/registration/sample_consensus_prerejective.h>


typedef pcl::PointXYZRGB PointT; //RGB possible

class PoseEstimator
{
    //variables
    pcl::PointCloud<PointT>::Ptr alignedSource,cloudModel;
    double fitnessScoreFine, alignedStrength;
    Eigen::Matrix4f finalPose;
    int firstTimePose;

    //Objects
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
    pcl::VoxelGrid<PointT> voxGrid;
    pcl::UniformSampling<PointT> uniSamp;
    pcl::NormalEstimation<PointT, pcl::Normal> normEst;
    pcl::registration::TransformationEstimationSVD<PointT,PointT> svd;


public:
    PoseEstimator();

    // function to estimate intial alignment transform (pose)
    Eigen::Matrix4f estimateCoarsePose( pcl::PointCloud<PointT>::Ptr p_sourceCloud, pcl::PointCloud<PointT>::Ptr p_targetCloud );

    //function to get keypoints and feature descriptors of cloud
    void getPersistentFeatures( pcl::PointCloud<pcl::PointXYZ>::Ptr &p_inputCloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cloudFeatures, boost::shared_ptr<std::vector<int> > &cloudIndices );

    //function to get keypoints and feature descriptors of cloud using ISS and FPFH
    void getFpfhFeatures( pcl::PointCloud<PointT>::Ptr &p_inputCloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cloudFeatures, pcl::PointCloud<PointT>::Ptr &cloudKeyPoints );

    //function to subsample and calculate normals
    void subSampleAndCalculateNormals (pcl::PointCloud<PointT>::Ptr p_inputCloud, pcl::PointCloud<PointT>::Ptr &p_inputCloudSubSampled, pcl::PointCloud<pcl::Normal>::Ptr &p_inputCloudSubSampledNormal, double leafSize );

    // function to estimate fine alignment transform (pose)
    Eigen::Matrix4f estimateFinePose( pcl::PointCloud<PointT>::Ptr &p_sourceCloud, pcl::PointCloud<PointT>::Ptr p_targetCloud );

    //function to estimate final alignemnet transform (pose) with intial and fine alignments
    Eigen::Matrix4f estimateFinalPose ( pcl::PointCloud<PointT>::Ptr &p_sourceCloud, pcl::PointCloud<PointT>::Ptr p_targetCloud, double &fitnessScore, double &alignStrength);

};

#endif // POSEESTIMATOR_H
