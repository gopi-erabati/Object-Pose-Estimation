#ifndef MISCELLANEOUS_H
#define MISCELLANEOUS_H

// ***************************** Miscellaneous Functions ******************************************

#include <QFileDialog>
#include <QString>
#include <QStringList>
#include <string>
#include "global.h"
#include "datagrabber.h"
#include "processingpcd.h"
#include "regmeshpcd.h"
#include "objectsegmentationplane.h"
//#include "objectdetection.h"
#include "geometry.h"

float roll = 0; float pitch = 0; float yaw = 0; float tX = 0;float tY = 0;float tZ = 0; bool eventStatus = false;

//Fucntion to take a list of point clouds using window dialogue and apply table top segmentation and save clusters
//void tableTop();

//Function to take RGB and Depth Images and Convert to PointClouds
//void convertToPointCloud();

//Function to point cloud frames and apply ICP
//void getIcp();

//Fucntion to get VFH Features
//void getVfh();

//Function to translate and rotate clouds using keyboard commands
//Rotations (in degrees)
//Roll - a (+2) ; b (-2)
//Pitch - d (+2) ; h (-2)
//Yaw - i (+2) ; k (-2)
//Translations (in m)
//X - m (+0.005) ; n (-0.005)
//Y - o (+0.005) ; t (-0.005)
//Z - v (+0.005) ; x (-0.005)
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);
void transAndRotPcd( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn2 );


//Function to unite two point clouds as one!
pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniteTwoClouds( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn2 );


#endif // MISCELLANEOUS_H
