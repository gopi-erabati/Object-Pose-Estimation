#ifndef EXTRACTSURF_H
#define EXTRACTSURF_H

#include <global.h>

class ExtractSURF
{
    //constructor varaibles
    int m_MinHesssian;

public:
    //contructor to intialise minHessian distance for SURF keypoint detection
    ExtractSURF( int minHessian );

    //function to return SURF descriptors
    cv::Mat getSURFFeatures( cv::Mat image );

    //function to return SURF descriptors and keypoints
    cv::Mat getSURFFeatures(cv::Mat image, std::vector<cv::KeyPoint> &keyPoints);

    //function to match SURF Features
    // type 1  - Flann Based Matcher with distance measure good match selection
    // type 2 - BrutForce Macther with distance ratio selection of good matches
    void matchSURFFeatures(cv::Mat image1, cv::Mat image2 , int type);


};

#endif // EXTRACTSURF_H
