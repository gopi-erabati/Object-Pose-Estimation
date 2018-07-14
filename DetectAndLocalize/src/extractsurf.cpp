#include "../include/extractsurf.h"

//contructor to intialise minHessian distance for SURF keypoint detection
ExtractSURF::ExtractSURF( int minHessian ):m_MinHesssian( minHessian )
{
}

//function to extract SURF descriptors
cv::Mat ExtractSURF::getSURFFeatures( cv::Mat image ){

    //Detect the keypoints using SURF detector
    //declare SURF detctor
    cv::SurfFeatureDetector detector( m_MinHesssian );

    //to store keypoints in detection
    std::vector<cv::KeyPoint> keyPoints;

    //detect the keypoints and store in keyPoints varaible
    cv::Mat imageGray;
    if (image.channels() == 3){ //to check if its rgb image or grayscale
        cv::cvtColor(image, imageGray, CV_RGB2GRAY);
    }
    else{
        imageGray = image.clone();
    }
    detector.detect( imageGray, keyPoints);

    //Compute Descriptors
    cv::SurfDescriptorExtractor extractor;

    //to store descriptors
    cv::Mat descriptors;

    //compute the descriptors
    extractor.compute( imageGray, keyPoints, descriptors );

    //to draw keypoints
    cv::Mat imgWithKP;
    cv::drawKeypoints( image, keyPoints, imgWithKP, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // show KeyPoints
    cv::imshow("KeyPoints", imgWithKP );
    cv::waitKey();

    return descriptors;

}

//function to extract SURF descriptors
cv::Mat ExtractSURF::getSURFFeatures(cv::Mat image, std::vector<cv::KeyPoint> &keyPoints){

    //Detect the keypoints using SURF detector
    //declare SURF detctor
    cv::SurfFeatureDetector detector( m_MinHesssian );

    //detect the keypoints and store in keyPoints varaible
    cv::Mat imageGray;
    if (image.channels() == 3){ //to check if its rgb image or grayscale
        cv::cvtColor(image, imageGray, CV_RGB2GRAY);
    }
    else{
        imageGray = image.clone();
    }
    detector.detect( imageGray, keyPoints);

    //Compute Descriptors
    cv::SurfDescriptorExtractor extractor;

    //to store descriptors
    cv::Mat descriptors;

    //compute the descriptors
    extractor.compute( imageGray, keyPoints, descriptors );

    //to draw keypoints
    cv::Mat imgWithKP;
    cv::drawKeypoints( image, keyPoints, imgWithKP, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    return descriptors;

}

//function to match SURF Features
void ExtractSURF::matchSURFFeatures( cv::Mat image1, cv::Mat image2, int type){

    std::vector<cv::KeyPoint> keyPoints1, keyPoints2;

    //get SURF descriptors for two images
    cv::Mat descriptors1 = getSURFFeatures( image1, keyPoints1 );
    cv::Mat descriptors2 = getSURFFeatures( image2, keyPoints2 );

    std::vector<cv::DMatch> goodMatches;
    // type 1  - Flann Based Matcher with distance measure good match selection
    // type 2 - BrutForce Macther with distance ratio selection of good matches
    if (type == 1){


        //Matching descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher;

        //varaible to store matches
        std::vector<cv::DMatch> matches;

        //find matches
        matcher.match( descriptors1, descriptors2, matches);

        // excluding matches based on distances
        double maxDist = 0; double minDist = 100;

        //calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors1.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < minDist ) minDist = dist;
            if( dist > maxDist ) maxDist = dist;
        }

        // Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        // or a small arbitary value ( 0.02 ) in the event that min_dist is very
        // small)
        for( int i = 0; i < descriptors1.rows; i++ )
        {
            if( matches[i].distance <= std::max(3*minDist, 0.02) )
            {
                goodMatches.push_back( matches[i] );
            }
        }

    }
    else if(type == 2){


        //Matching descriptor vectors using bruteForce matcher
        std::vector<std::vector<cv::DMatch> > matches;
        cv::BFMatcher matcher;
        matcher.knnMatch(descriptors1, descriptors2, matches, 2);  // Find two nearest matches
        for (int i = 0; i < matches.size(); ++i)
        {
            const float ratio = 0.7; // As in Lowe's paper; can be tuned
            if (matches[i][0].distance < ratio * matches[i][1].distance)
            {
                goodMatches.push_back(matches[i][0]);
            }
        }

    }


    // Draw only "good" matches
    cv::Mat imgMatches;
    cv::drawMatches( image1, keyPoints1, image2, keyPoints2,
                     goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // Show detected matches
    imshow( "Good Matches", imgMatches );
    cv::waitKey();

}
