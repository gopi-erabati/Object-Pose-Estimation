# Object-Pose-Estimation

**Contents**

* [Introduction](#introduction)
* [Abstract](#abstract)
* [Prerequisites](#prerequisites)
* [Working](#working)
* [Videos](#videos)
* [Repository](#repository)

## Introduction

This repository deals with the code and documentation of the project "3D Object Detection and Relative Localization using a 3D Sensor Embedded on a Mobile Robot".
This work is carried at LAAS-CNRS, Toulouse, France as a part of my Master thesis internship under the supervision of Dr. Frederic Lerasale. 
Code, report and user manuals can be found in this repository.

## Abstract

The technology in current research scenario is marching towards automation for higher productivity with accurate and precise product development. Vision and Robotics are domains which work to create autonomous systems and are the key technology in quest for mass productivity. The automation in an industry can be achieved by detecting interactive objects and estimating the pose to manipulate them. Therefore the object localization ( i.e., pose) includes position and orientation of object, has profound significance. The application of object pose estimation varies from industry automation to entertainment industry and from health care to surveillance. The objective of pose estimation of objects is very significant in many cases, like in order for the robots to manipulate the objects, for accurate rendering of Augmented Reality (AR) among others.

This thesis tries to solve the issue of object pose estimation using 3D data of scene acquired from 3D sensors (e.g. Kinect, Orbec Astra Pro among others). The 3D data has an advantage of independence from object texture and invariance to illumination. The proposal is divided into two phases : An offline phase where the 3D model template of the object ( for estimation of pose) is built using Iterative Closest Point (ICP) algorithm. And an online phase where the pose of the object is estimated by aligning the scene to the model using ICP, provided with an initial alignment using 3D descriptors (like Fast Point Feature Transform (FPFH)).The approach we develop is to be integrated on two different platforms : 1)Humanoid robot ‘Pyrene’ which has Orbec Astra Pro 3D sensor for data acquisition, and 2)Unmanned Aerial Vehicle (UAV) which has Intel Realsense Euclid on it. The datasets of objects (like electric drill, brick, a small cylinder, cake box) are acquired using Microsoft Kinect, Orbec Astra Pro and Intel RealSense Euclid sensors to test the performance of this technique. The objects which are used to test this approach are the ones which are used by robot. This technique is tested in two scenarios, firstly, when the object is on the table and secondly when the object is held in hand by a person. The range of objects from the sensor is 0.6 to 1.6m. This technique could handle occlusions of the object by hand (when we hold the object), as ICP can work even if partial object is visible in the scene.

## Prerequisites

1. OpenCV 2.4.8 or higher
2. PCL 1.7.1 or higher
3. ROS Indigo or higher
4. CMAKE 2.8.9 or higher

## Working

The process to build and run this code is exaplined in user manuals in this repository.

## Videos

The links to video demonstrations of this approach to detect object pose are below : 

## Repository

### BuildModel

This folder contains the code to reconstruct (build) a 3D model of object by capturing the frames of object from different view angles for reconstruction.

### DetectAndLocalise

This folder contains the code to estimate the pose of the object using a given model of the object.
