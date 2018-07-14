#include "../include/datagrabber.h"

//Constructor to set the Sensor
DataGrabber::DataGrabber(bool p_euclidSensor, bool p_kinectSensor, bool p_astraSensor):euclidSensor( p_euclidSensor ), kinectSensor ( p_kinectSensor ), astraSensor ( p_astraSensor )
{
}

//function to convert RGB and Depth Maps to Point Clouds
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > DataGrabber::rgbd2Pcl( cv::Mat p_imageRgb, cv::Mat p_imageDepth ){

    //create a new point cloud , reset it and resize the number of points as image size
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->height = p_imageRgb.rows;
    cloud->width = p_imageRgb.cols;
    cloud->is_dense = true;
    cloud->points.resize( p_imageRgb.rows * p_imageRgb.cols );

    int pointCount = 0; //variable to count points

    // start for all rows and columns of image
    for( int j=0; j<p_imageRgb.cols; j++ ) // y
    {
        for( int i=0; i<p_imageRgb.rows; i++ ) // x
        {
            float X, Y, Z, depth; //for 3D Point values

            depth = p_imageDepth.at<unsigned short>( i, j ); //Euclid



            // Render the 3D values
            depthToMeter( i, j, depth, X, Y, Z );


            // Remove features which are out of  senser range
            if( Z == 0 || Z > 2.0){
                continue;
            }

            //std::cout << Y << " " << X << " " << Z << std::endl;

            // Write out the colored 3D point
            //Points
            cloud->points[pointCount].y = X;
            cloud->points[pointCount].x = Y;
            cloud->points[pointCount].z = Z;
            //Color
            uint8_t r = p_imageRgb.at<cv::Vec3b>(i,j)[2];
            uint8_t g = p_imageRgb.at<cv::Vec3b>(i,j)[1];
            uint8_t b = p_imageRgb.at<cv::Vec3b>(i,j)[0];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cloud->points[pointCount].rgb = *reinterpret_cast<float*>(&rgb);



            pointCount++;
        }
    }

    // erase redundant points
    cloud->erase(cloud->points.begin()+pointCount, cloud->points.end());
    return cloud;

}

//function to convert Depth Maps to Point Clouds
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > DataGrabber::rgbd2Pcl( cv::Mat p_imageDepth ){

    //create a new point cloud , reset it and resize the number of points as image size
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->height = p_imageDepth.rows;
    cloud->width = p_imageDepth.cols;
    cloud->is_dense = true;
    cloud->points.resize( p_imageDepth.rows * p_imageDepth.cols );

    int pointCount = 0; //variable to count points

    // start for all rows and columns of image
    for( int j=0; j<p_imageDepth.cols; j++ ) // y
    {
        for( int i=0; i<p_imageDepth.rows; i++ ) // x
        {
            float X, Y, Z, depth; //for 3D Point values

            depth = p_imageDepth.at<unsigned short>( i, j ); //Euclid

            // Render the 3D values
            depthToMeter( i, j, depth, X, Y, Z );


            // Remove features which are out of  senser range
            if( Z == 0 ){
                continue;
            }

            //std::cout << Y << " " << X << " " << Z << std::endl;

            // Write out the colored 3D point
            //Points
            cloud->points[pointCount].y = X;
            cloud->points[pointCount].x = Y;
            cloud->points[pointCount].z = Z;
            //Color
            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cloud->points[pointCount].rgb = *reinterpret_cast<float*>(&rgb);



            pointCount++;
        }
    }

    cloud->erase(cloud->points.begin()+pointCount, cloud->points.end());
    return cloud;


}

//function to convert the depth values from depth image to meters
void DataGrabber::depthToMeter( const float p_FeatX, const float p_FeatY, const float p_RawDisparity,
                               float &p_X, float &p_Y, float &p_Z ){

    float fx, fy, cx, cy, sclFactor;

    if (euclidSensor ){
        if(p_RawDisparity <= 0 || p_RawDisparity > 65536 || std::isnan(p_RawDisparity)){
            p_X = 0; p_Y = 0; p_Z = 0; return;

        }

        // EUCLID
            fx = 306.178; // focal length x
            fy = 306.929; // focal length y
            cx = 158.523; // optical center x
            cy = 122.747; // optical center y
            sclFactor = 1000.0;
    }

    if ( kinectSensor ){

        if ( p_RawDisparity <= 0.0 || std::isnan(p_RawDisparity)){
            p_X = 0; p_Y = 0; p_Z = 0; return;
        }
        //KINECT
        fx = 525.0; // focal length x 570.342
        fy = 525.0; // focal length y 570.342
        cx = 319.5; // optical center x
        cy = 239.5; // optical center y
        sclFactor = 1000.0;
    }

    if ( astraSensor ){

        if ( p_RawDisparity <= 0.0 || std::isnan(p_RawDisparity)){
            p_X = 0; p_Y = 0; p_Z = 0; return;
        }
        //KINECT
        fx = 570.342; // focal length x
        fy = 570.342; // focal length y
        cx = 314.5; // optical center x
        cy = 235.5; // optical center y
        sclFactor = 1000.0;

    }


    // Recall the camera projective projection model
    p_Z = p_RawDisparity / sclFactor;
    p_X = (p_FeatX - cx) * p_Z / fx;
    p_Y = (p_FeatY - cy) * p_Z / fy;


}

