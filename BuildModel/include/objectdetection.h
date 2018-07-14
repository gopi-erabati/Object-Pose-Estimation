#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

#include <global.h>

typedef pcl::PointXYZRGB PointTDet;

typedef std::pair<std::string, std::vector<float> > vfhModel;

class ObjectDetection
{
    pcl::NormalEstimation<PointTDet, pcl::PointXYZRGBNormal> normEst;
    pcl::search::KdTree<PointTDet>::Ptr kdTree;

    pcl::VFHEstimation<PointTDet, pcl::PointXYZRGBNormal, pcl::VFHSignature308> vfhEst;

    std::vector<vfhModel> models;
    flann::Index<flann::ChiSquareDistance<float> > *index;


public:
    ObjectDetection();

    //function to get VFH Features
    void getVfhFeature ( pcl::PointCloud<PointTDet>::Ptr p_cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr &p_cloudVfh);

    // training stage to get kdTree representation of VFH descriptors
    void getkdTreeRepresentation (const boost::filesystem::path &base_dir);

    void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                       std::vector<vfhModel> &models);

    bool loadHist (const boost::filesystem::path &path, vfhModel &vfh);

    bool getObjectName( pcl::PointCloud<PointTDet>::Ptr p_cloud, std::string &p_objName);

    void loadTrainData();

    bool loadFileList (std::vector<vfhModel> &models, const std::string &filename);

    inline void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfhModel &model,
                    int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);
};

#endif // OBJECTDETECTION_H
