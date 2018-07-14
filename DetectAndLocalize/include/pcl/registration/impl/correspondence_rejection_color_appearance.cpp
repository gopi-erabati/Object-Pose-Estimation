/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Perception, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/registration/correspondence_rejection_color_appearance.h>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
// do opencv 2 code
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#elif CV_MAJOR_VERSION == 3
// do opencv 3 code
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif


using namespace pcl;
using namespace cv;
using namespace std;

float dist_L2(float *color_p, float *centers_p){
  
  return (color_p[0] - centers_p[0]) * (color_p[0] - centers_p[0]) +
  (color_p[1] - centers_p[1]) * (color_p[1] - centers_p[1]) +
  (color_p[2] - centers_p[2]) * (color_p[2] - centers_p[2]) +
  (color_p[3] - centers_p[3]) * (color_p[3] - centers_p[3]) +
  (color_p[4] - centers_p[4]) * (color_p[4] - centers_p[4]) +
  (color_p[5] - centers_p[5]) * (color_p[5] - centers_p[5]);
}

template <typename T>
void rgb2Lab(T &r, T &g, T &b, T &L, T &A, T &B){
  
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::CorrespondenceRejectorColorAppearance::getRemainingCorrespondences (
                                                                                       const Correspondences& original_correspondences,
                                                                                       Correspondences& remaining_correspondences)
{
  if (!data_container_ and cloud_rgb_==NULL)
  {
    PCL_ERROR ("[registratin::%s::getRemainingCorrespondences] DataContainer object is not initialized!\n", getClassName ().c_str ());
    return;
  }
  PointCloud<PointXYZRGBA>::ConstPtr tgt;
  
  if (cloud_rgb_==NULL)
    tgt = boost::static_pointer_cast<DataContainer<PointXYZRGBA> > (data_container_)->getInputTarget ();
  else
    tgt = *cloud_rgb_;
  
  unsigned int number_valid_correspondences = 0;
  remaining_correspondences.resize (original_correspondences.size ());
  
  int i = 0;
  int j = 0;
  int r = 0;
  const float gamma = 20;
  float *centers_p = NULL;
  
  float dist;
  float min_dist;
  
  PointXYZRGBA ptTmp;
  
  
  // Used precomputed centers
  if (center_most_distinctive != -1){
    centers_p = (float*) centers.data;
    float color_tmp[6];
    int idx = 0;
    
    // Test each correspondence
    for (i = 0, r = 0; i < original_correspondences.size (); ++i, r+= 6)
    {
      const PointXYZRGBA &pt = tgt->points[original_correspondences[i].index_match];
      color_tmp[0] = pt.r;
      color_tmp[1] = pt.b;
      color_tmp[2] = pt.g;
      
      color_tmp[3] = gamma*pt.x;
      color_tmp[4] = gamma*pt.y;
      color_tmp[5] = gamma*pt.z;
      
      
      min_dist = FLT_MAX;
      
      for (j = 0; j < 30; j+=6) {
        // Calc euclidian distance
        dist = dist_L2(&color_tmp[0], centers_p+j);
        // find minimum distance and set label
        if (dist < min_dist){
          idx = j;
          min_dist = dist;
        }
      }
      
      idx /= 6;
      
      if (idx != center_most_distinctive)
        remaining_correspondences[number_valid_correspondences++] = original_correspondences[i];
      
    }
  }
  else {
    
    if (original_correspondences.size () < 6 ){
      remaining_correspondences = original_correspondences;
      return;
    }
    
    Mat color(original_correspondences.size (), 6, CV_32F);
    float *color_p = (float*)color.data;
    
    Mat labels;
    int *labels_p = NULL;
    
    // Test each correspondence
    for (i = 0, r = 0; i < original_correspondences.size (); ++i, r+= 6)
    {
      const PointXYZRGBA &pt = tgt->points[original_correspondences[i].index_match];
      color_p[r]   = pt.r;
      color_p[r+1] = pt.b;
      color_p[r+2] = pt.g;
      
      color_p[r+3] = gamma*pt.x;
      color_p[r+4] = gamma*pt.y;
      color_p[r+5] = gamma*pt.z;
      
    }
    
    kmeans(color, 5, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 500, 0.001), 1, KMEANS_PP_CENTERS, centers);
    centers_p = (float*) centers.data;
    labels_p = (int*)labels.data;
    
    // Convert to Lab space color
    Mat rgb(5,1, CV_8UC3);
    Mat Lab(5,1, CV_8UC3);
    uchar *rgb_p = rgb.data;
    for (i = 0, j=0; i < 5; i+=3, j+=6){
      rgb_p[i]   = centers_p[j];
      rgb_p[i+1] = centers_p[j+1];
      rgb_p[i+2] = centers_p[j+2];
    }
    cvtColor(rgb, Lab, CV_RGB2Lab);
    uchar *lab_p = Lab.data;
    for (i = 0, j=0; i < 15; i+=3, j+=6){
      centers_p[j]   = lab_p[i] ;
      centers_p[j+1] = lab_p[i+1];
      centers_p[j+2] = lab_p[i+2];
    }
    
    
    // Compute cost
    float cst[5]; memset(&cst[0], 0, sizeof(float)*5);
    for (i = 0; i < 5; ++i){
      for (j = i + 1; j < 5; ++j){
        dist = dist_L2(centers_p+i, centers_p+j);
        cst[i] += dist;
        cst[j] += dist;
      }
    }
    // Find the cluster with the highest cost
    dist = 0.f;
    for (i = 0; i < 5; ++i){
      if (cst[i] > dist){
        dist = cst[i];
        j = i;
      }
    }
    
    center_most_distinctive = j;
    
    // Remove all correspondences of cluster j
    for (i = 0, r = 0; i < original_correspondences.size (); ++i, r+= 6)
      if (labels_p[i] != j)
        remaining_correspondences[number_valid_correspondences++] = original_correspondences[i];
  }
  
  remaining_correspondences.resize (number_valid_correspondences);
}



















