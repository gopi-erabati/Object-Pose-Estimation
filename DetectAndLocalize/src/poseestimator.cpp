#include "../include/poseestimator.h"

PoseEstimator::PoseEstimator()
{
    firstTimePose = 0;
    fitnessScoreFine = 10; //random high value
    alignedStrength = 0.0;
    finalPose << Eigen::Matrix4f::Identity(); //1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //identity

    alignedSource = pcl::PointCloud<PointT>::Ptr ( new pcl::PointCloud<PointT> );
    cloudModel = pcl::PointCloud<PointT>::Ptr ( new pcl::PointCloud<PointT> );

}

// function to estimate intial alignment transform (pose)
Eigen::Matrix4f PoseEstimator::estimateCoarsePose( pcl::PointCloud<PointT>::Ptr p_sourceCloud, pcl::PointCloud<PointT>::Ptr p_targetCloud ){

    /* ************* Steps **********************
     * 1. Get Keypoints and feature descriptors of keypoints for source and target clouds
     * 2. Get correspondences for source and target clouds
     * 3. Reject false correspondences
     * 4. Estimate transform (pose) based on filtered correspondences
     * *******************************************/

    pcl::PointCloud<PointT>::Ptr sourceCloud (new pcl::PointCloud<PointT> );
    pcl::copyPointCloud( *p_sourceCloud, *sourceCloud );
    pcl::PointCloud<PointT>::Ptr targetCloud (new pcl::PointCloud<PointT> );
    pcl::copyPointCloud( *p_targetCloud, *targetCloud );


    // Get keypoints and feature descriptors of source and target clouds
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceCloudFeatures ( new pcl::PointCloud<pcl::FPFHSignature33> );
    pcl::PointCloud<PointT>::Ptr sourceCloudKeyPoints (new pcl::PointCloud<PointT>);
    getFpfhFeatures(sourceCloud, sourceCloudFeatures, sourceCloudKeyPoints );

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetCloudFeatures ( new pcl::PointCloud<pcl::FPFHSignature33> );
    pcl::PointCloud<PointT>::Ptr targetCloudKeyPoints ( new pcl::PointCloud<PointT> );
    getFpfhFeatures(targetCloud, targetCloudFeatures, targetCloudKeyPoints );

    if(targetCloudFeatures->points.size() < 10){
        pcl::copyPointCloud( *p_sourceCloud, *alignedSource);
        std::cout << "NO target cloud in Initial Alignment" << std::endl;
        Eigen::Matrix4f pose; pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //identity
        return pose;
    }


    //Sample Consensus Initail Alignment
    pcl::PointCloud<PointT>::Ptr result ( new pcl::PointCloud<PointT> );
    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sacia;
    sacia.setInputSource( sourceCloud );
    sacia.setInputTarget( targetCloud );
    sacia.setSourceFeatures( sourceCloudFeatures );
    sacia.setTargetFeatures( targetCloudFeatures );
    sacia.setMaximumIterations( 400 ); //1000
    sacia.setNumberOfSamples(5); //3
    sacia.setCorrespondenceRandomness(5);
    sacia.setMaxCorrespondenceDistance( 0.05 ); //0.15 //0.005 //0.03 GT workfine
    sacia.setMinSampleDistance( 0.01f ); //0.01 //0.001 //0.008 GT workfine
    {
        pcl::ScopeTime t("Initial Alignment");
        sacia.align (*result);
    }
    Eigen::Matrix4f pose = sacia.getFinalTransformation();
    //std::cout << "SCA_IA converged with score: " << sacia.getFitnessScore() << std::endl;

    pcl::PointCloud<PointT>::Ptr alignedCloud ( new pcl::PointCloud<PointT> );
    pcl::transformPointCloud( *p_sourceCloud, *alignedCloud, pose);

    pcl::copyPointCloud( *alignedCloud, *alignedSource);

    return pose;
}

////function to get keypoints and feature descriptors of cloud
//void PoseEstimator::getPersistentFeatures( pcl::PointCloud<pcl::PointXYZ>::Ptr &p_inputCloud,  pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cloudFeatures, boost::shared_ptr<std::vector<int> > &cloudIndices ){

//    //get downsampled and normals
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSubSampled;
//    pcl::PointCloud<pcl::Normal>::Ptr cloudSubSampledNormal;
//    subSampleAndCalculateNormals( p_inputCloud, cloudSubSampled, cloudSubSampledNormal );
//    *p_inputCloud = *cloudSubSampled;

//    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> mulSclFeatPer;
//    mulSclFeatPer.computeFeaturesAtAllScales();
//    std::vector<float> scaleValues;
//    float myints[] = { 2.0f, 1.0f, 1.5f }; scaleValues.assign( myints, myints+3 );
//    mulSclFeatPer.setScalesVector( scaleValues );

//    // define a feature descriptor
//    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhEstimator ( new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> );
//    fpfhEstimator->setInputCloud( cloudSubSampled );
//    fpfhEstimator->setInputNormals( cloudSubSampledNormal );
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZ> );
//    fpfhEstimator->setSearchMethod( tree );
//    fpfhEstimator->setRadiusSearch(0.01f);
//    mulSclFeatPer.setFeatureEstimator( fpfhEstimator );
//    mulSclFeatPer.setDistanceMetric( pcl::CS );

//    //get keypoints(inidces) and features
//    cloudFeatures = pcl::PointCloud<pcl::FPFHSignature33>::Ptr ( new pcl::PointCloud<pcl::FPFHSignature33> () );
//    cloudIndices = boost::shared_ptr<std::vector<int> > (new std::vector<int> () );
//    mulSclFeatPer.determinePersistentFeatures( *cloudFeatures, cloudIndices );


//}


//function to get keypoints and feature descriptors of cloud using ISS and FPFH
void PoseEstimator::getFpfhFeatures( pcl::PointCloud<PointT>::Ptr &p_inputCloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cloudFeatures, pcl::PointCloud<PointT>::Ptr &cloudKeyPoints ){

    //get normals
    //get downsampled and normals
    pcl::PointCloud<PointT>::Ptr cloudSubSampled ( new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudSubSampledNormal ( new pcl::PointCloud<pcl::Normal> );
    subSampleAndCalculateNormals( p_inputCloud, cloudSubSampled, cloudSubSampledNormal, 0.01 ); //0.008 on GT working fine
    p_inputCloud->clear();
    *p_inputCloud = *cloudSubSampled;

    // FPFH Descriptor
    fpfhEstimation.setInputCloud( cloudSubSampled );
    fpfhEstimation.setRadiusSearch( 0.03 );
    fpfhEstimation.setInputNormals( cloudSubSampledNormal );
    cloudFeatures = pcl::PointCloud<pcl::FPFHSignature33>::Ptr ( new pcl::PointCloud<pcl::FPFHSignature33> );
    fpfhEstimation.compute( *cloudFeatures );


}

//function to subsample and calculate normals
void PoseEstimator::subSampleAndCalculateNormals (pcl::PointCloud<PointT>::Ptr p_inputCloud, pcl::PointCloud<PointT>::Ptr &p_inputCloudSubSampled, pcl::PointCloud<pcl::Normal>::Ptr &p_inputCloudSubSampledNormal, double leafSize ){

    //downsample
    p_inputCloudSubSampled = pcl::PointCloud<PointT>::Ptr ( new pcl::PointCloud<PointT> () );
    //pcl::VoxelGrid<PointT> voxGrid;
    //voxGrid.setInputCloud( p_inputCloud );
    //voxGrid.setLeafSize( leafSize, leafSize, leafSize);
    //voxGrid.filter( *p_inputCloudSubSampled );
    //*p_inputCloudSubSampled = *p_inputCloud;

    uniSamp.setInputCloud( p_inputCloud );
    uniSamp.setRadiusSearch( leafSize );
    pcl::PointCloud<int> keyPointIndices;
    uniSamp.compute( keyPointIndices);
    pcl::copyPointCloud( *p_inputCloud, keyPointIndices.points, *p_inputCloudSubSampled);


    //Normal estimation
    p_inputCloudSubSampledNormal =  pcl::PointCloud<pcl::Normal>::Ptr  ( new pcl::PointCloud<pcl::Normal> () );
    //create kdTree for search
    pcl::search::KdTree<PointT>::Ptr kdtree ( new pcl::search::KdTree<PointT> );
    normEst.setSearchMethod( kdtree );
    normEst.setKSearch( 30 ); //30
    //normEst.setRadiusSearch(0.03);
    normEst.setInputCloud( p_inputCloudSubSampled );
    normEst.compute( *p_inputCloudSubSampledNormal );

}

// function to estimate fine alignment transform (pose)
Eigen::Matrix4f PoseEstimator::estimateFinePose( pcl::PointCloud<PointT>::Ptr &p_sourceCloud, pcl::PointCloud<PointT>::Ptr p_targetCloud ){

    /* ***************** STEPS ********************
     * 1. Downsample and calcualte normals of clouds
     * 2. Correspondence estimation
     *      2.1 Correspondence_estimation
     *      2.2 correspondence_estimation_organized_projection
     *      2.3 correspondence_estimation_noraml_shooting
     * 3. Correspondence Rejection
     *      3.1 correpondence_rejection_medain_distance
     *      3.2 correpondence_rejection_one_to_one
     *      3.3 correpondence_rejection_surface_normal
     *      3.4 correpondence_rejection_sample_consensus
     *      3.5 correpondence_rejection_var_trimmed
     * 4.  Transformation Estimation
     *      4.1 transformation_estimation_point_to-plane
     *      4.2 transformation_estimation_lm
     *      4.3 transformation_estimation_svd
     * 5. Termination Criteria
     *          default_convergence_criteria
     * ********************************************/

    pcl::PointCloud<PointT>::Ptr sourceCloud (new pcl::PointCloud<PointT> );
    pcl::copyPointCloud( *p_sourceCloud, *sourceCloud );
    pcl::PointCloud<PointT>::Ptr targetCloud (new pcl::PointCloud<PointT> );
    pcl::copyPointCloud( *p_targetCloud, *targetCloud );

    //remove NaN points
    const pcl::PointCloud<PointT>::Ptr sourceCloudConstPtr = sourceCloud;
    const pcl::PointCloud<PointT>::Ptr targetCloudConstPtr = targetCloud;
    std::vector<int> index;
    pcl::removeNaNFromPointCloud( *sourceCloudConstPtr, *sourceCloud, index );
    pcl::removeNaNFromPointCloud( *targetCloudConstPtr, *targetCloud, index );

    //1. Downsample and normal estimation
    //get downsampled and normals "Source"
    pcl::PointCloud<PointT>::Ptr cloudSourceSubSampled;
    pcl::PointCloud<pcl::Normal>::Ptr cloudSourceSubSampledNormal;
    subSampleAndCalculateNormals( sourceCloud, cloudSourceSubSampled, cloudSourceSubSampledNormal, 0.008 ); //0.005
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSourceSubSampledPointNormal (new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    pcl::copyPointCloud( *cloudSourceSubSampled, *cloudSourceSubSampledPointNormal);
    pcl::copyPointCloud(*cloudSourceSubSampledNormal, *cloudSourceSubSampledPointNormal );
    // get downsampled and normal "target"
    pcl::PointCloud<PointT>::Ptr cloudTargetSubSampled;
    pcl::PointCloud<pcl::Normal>::Ptr cloudTargetSubSampledNormal;
    subSampleAndCalculateNormals( targetCloud, cloudTargetSubSampled, cloudTargetSubSampledNormal, 0.008 ); //0.005
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTargetSubSampledPointNormal (new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    pcl::copyPointCloud( *cloudTargetSubSampled, *cloudTargetSubSampledPointNormal);
    pcl::copyPointCloud(*cloudTargetSubSampledNormal, *cloudTargetSubSampledPointNormal );


    //remove NaN normals
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSourceSubSampledPointNormalConstPtrTemp = cloudSourceSubSampledPointNormal;
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTargetSubSampledPointNormalConstPtrTemp = cloudTargetSubSampledPointNormal;
    pcl::removeNaNNormalsFromPointCloud( *cloudSourceSubSampledPointNormalConstPtrTemp, *cloudSourceSubSampledPointNormal, index);
    pcl::removeNaNNormalsFromPointCloud( *cloudTargetSubSampledPointNormalConstPtrTemp, *cloudTargetSubSampledPointNormal, index);

    if(cloudTargetSubSampledPointNormal->points.size() < 100){
        std::cout << "NO target cloud in Final Alignment" << std::endl;
        pcl::copyPointCloud(*p_sourceCloud, *p_sourceCloud);
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); // pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //identity
        return pose;
    }

    // 2. Correspodndence estimation
    pcl::CorrespondencesPtr correspondences ( new pcl::Correspondences );

//    //2.1 correspondence_estimation
//    pcl::registration::CorrespondenceEstimation<PointT, PointT>::Ptr corrEst (new pcl::registration::CorrespondenceEstimation<PointT, PointT> );
//    corrEst->setInputSource( cloudSourceSubSampled );
//    corrEst->setInputTarget( cloudTargetSubSampled );
//    corrEst->determineCorrespondences( *correspondences, 0.1 );

//    // 2.2 correspondence_estimation_organized_projection
//    pcl::registration::CorrespondenceEstimationOrganizedProjection<PointT, PointT>::Ptr corrEstOrgProj ( new pcl::registration::CorrespondenceEstimationOrganizedProjection<PointT, PointT> );
//    corrEstOrgProj->setInputSource( cloudSourceSubSampled );
//    corrEstOrgProj->setInputTarget( cloudTargetSubSampled );
//    corrEstOrgProj->setDepthThreshold( 0.1 );
//    corrEstOrgProj->determineCorrespondences( *correspondences, 0.15 );

    // 2.3 correspondence_estimation_noraml_shooting
    pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr corrEstNormShoot ( new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> );
    corrEstNormShoot->setInputSource( cloudSourceSubSampledPointNormal );
    corrEstNormShoot->setSourceNormals( cloudSourceSubSampledPointNormal );
    corrEstNormShoot->setInputTarget( cloudTargetSubSampledPointNormal );
    corrEstNormShoot->setKSearch( 20 ); //5
    corrEstNormShoot->determineCorrespondences( *correspondences, 0.01 ); //0.05 Brick //0.01GT workfine


    // 3. Correspondence Rejection
    pcl::CorrespondencesPtr correspondencesFiltered ( new pcl::Correspondences );

//    // 3.1 correpondence_rejection_medain_distance
//    pcl::registration::CorrespondenceRejectorMedianDistance::Ptr corrRejmedDist ( new pcl::registration::CorrespondenceRejectorMedianDistance );
//    corrRejmedDist->setInputCorrespondences( correspondences );
//    corrRejmedDist->setMedianFactor( 0.8 );
//    corrRejmedDist->getCorrespondences( *correspondencesFiltered );

//    // 3.2 correpondence_rejection_one_to_one
//    pcl::registration::CorrespondenceRejectorOneToOne::Ptr corrRejOneToOne ( new pcl::registration::CorrespondenceRejectorOneToOne );
//    corrRejOneToOne->getRemainingCorrespondences( *correspondences, *correspondencesFiltered );

    // 3.3 correpondence_rejection_surface_normal
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr corrRejSurNorm (new pcl::registration::CorrespondenceRejectorSurfaceNormal );
    corrRejSurNorm->initializeDataContainer<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>();
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSourceSubSampledPointNormalConstPtr = cloudSourceSubSampledPointNormal;
    corrRejSurNorm->setInputSource<pcl::PointXYZRGBNormal>(cloudSourceSubSampledPointNormalConstPtr);
    corrRejSurNorm->setInputNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloudSourceSubSampledPointNormalConstPtr);
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTargetSubSampledPointNormalConstPtr = cloudTargetSubSampledPointNormal;
    corrRejSurNorm->setInputTarget<pcl::PointXYZRGBNormal>(cloudTargetSubSampledPointNormalConstPtr);
    corrRejSurNorm->setTargetNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloudTargetSubSampledPointNormalConstPtr);
    corrRejSurNorm->setThreshold( 0.7 ); //0.7 Brick
    corrRejSurNorm->getRemainingCorrespondences( *correspondences, *correspondencesFiltered );

//    // 3.4 correpondence_rejection_sample_consensus
//    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr corrRejSAC ( new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> );
//    corrRejSAC->setInputSource( cloudSourceSubSampled );
//    corrRejSAC->setInputTarget( cloudTargetSubSampled );
//    corrRejSAC->setInlierThreshold( 0.7 );
//    corrRejSAC->setMaximumIterations( 1000 );
//    corrRejSAC->getRemainingCorrespondences( *correspondences, *correspondencesFiltered );

//    // 3.5 correpondence_rejection_var_trimmed
//    pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr corrRejVarTrim ( new pcl::registration::CorrespondenceRejectorVarTrimmed );
//    corrRejVarTrim->setMinRatio( 0.3 );
//    corrRejVarTrim->getRemainingCorrespondences( *correspondences, *correspondencesFiltered );

    // 3.6 Self OCcluded Normal
#if (PCL_MINOR_VERSION >= 7 && PCL_REVISION_VERSION >= 2)
    pcl::registration::CorrespondenceRejectorSelfOccludedNormal::Ptr corrRejSelfNorm ( new pcl::registration::CorrespondenceRejectorSelfOccludedNormal );
    corrRejSelfNorm->setThreshold( 0.6 );
#endif

    // 4. Transformation Estimation
    //Eigen::Matrix4f pose;

    // 4.1 transformation_estimation_point_to-plane
    pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transfEstpointToPlane ( new pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> );
    //transfEstpointToPlane->estimateRigidTransformation( *cloudSourceSubSampledPointNormal, *cloudTargetSubSampledPointNormal, *correspondencesFiltered, pose);

//    // 4.2 transformation_estimation_lm
    //pcl::registration::TransformationEstimationLM<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transfEstLm ( new pcl::registration::TransformationEstimationLM<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> );
//   // transfEstLm->estimateRigidTransformation( *cloudSourceSubSampled, *cloudTargetSubSampled, *correspondencesFiltered, pose);

//    // 4.3 transformation_estimation_svd
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transfEstSvd ( new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> );
//    //transfEstSvd->estimateRigidTransformation( *cloudSourceSubSampled, *cloudTargetSubSampled, *correspondencesFiltered, pose);

    //ICP
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
    // Set the input source and target
    icp.setInputSource ( cloudSourceSubSampledPointNormal );
    icp.setInputTarget ( cloudTargetSubSampledPointNormal );

    // Set the max correspondence distance , correspondences with higher distances will be ignored
    //icp.setMaxCorrespondenceDistance ( 0.05 );

    // set the ransac outlier rejection threshold
    //icp.setRANSACOutlierRejectionThreshold( 0.01 );

    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations ( 100 );

    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);

    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-8);

    //set correspondence estimation
    icp.setCorrespondenceEstimation( corrEstNormShoot );

    //set Rejectors
    icp.addCorrespondenceRejector( corrRejSurNorm );
#if (PCL_MINOR_VERSION >= 7 && PCL_REVISION_VERSION >= 2)
    icp.addCorrespondenceRejector( corrRejSelfNorm );
#endif


    //set transofrmation estimation
    icp.setTransformationEstimation( transfEstSvd );

    // Perform the alignment
    //cloud to store the result
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudAligned (new pcl::PointCloud<pcl::PointXYZRGBNormal> );

    {

        pcl::ScopeTime t("Final Alignment");
        icp.align ( *cloudAligned);

    }

    fitnessScoreFine = icp.getFitnessScore();
    //std::cout << "ICP converged with score: " << fitnessScoreFine << std::endl;

    Eigen::Matrix4f pose = icp.getFinalTransformation();
    pcl::PointCloud<PointT>::Ptr alignedCloud (new pcl::PointCloud<PointT> );
    pcl::transformPointCloud( *p_sourceCloud, *alignedCloud, pose);
    *p_sourceCloud = *alignedCloud;
    //*alignedCloud += *p_targetCloud;

    alignedStrength = icp.getAlignStrength();
    std::cout << "Aligned Strength : " << alignedStrength << std::endl;

//    //save
//    std::stringstream ss;
//    ss << "/home/gkerabat/Documents/Project/DataSets/test/PointClouds/FilteredClouds/cloudAlignedFine.pcd";
//    if (pcl::io::savePCDFile( ss.str(), *alignedCloud, true) == 0)
//    {
//        std::cout << "Saved fine aligned cloud" << "." << endl;
//    }
//    else PCL_ERROR("Problem saving cluster cloud.\n");


    return pose;


}


//function to estimate final alignemnet transform (pose) with intial and fine alignments
Eigen::Matrix4f PoseEstimator::estimateFinalPose ( pcl::PointCloud<PointT>::Ptr &p_sourceCloud, pcl::PointCloud<PointT>::Ptr p_targetCloud, double &fitnessScore, double &alignStrength ){

    //Copy the original source (Model) for first frame, which is used later to find transformation
    if(firstTimePose == 0){
        pcl::copyPointCloud(*p_sourceCloud, *cloudModel);
    }
    firstTimePose++;

    //variables
    Eigen::Matrix4f coarsePose, finePose;
    coarsePose << Eigen::Matrix4f::Identity(); //1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //identity

    // COARSE ALIGNMENT (SCA-IA)
    //If fitness score of previous fine transormation is > 0.0001 then agaian perform intial alignment otherwise No!
    if (!p_targetCloud->empty()){

        if ( (fitnessScoreFine > 0.0001) ){

           coarsePose  = estimateCoarsePose( p_sourceCloud, p_targetCloud );

        }
    }
    else{

        coarsePose << Eigen::Matrix4f::Identity(); //1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //identity
    }

    // FINE ALIGNMENT (ICP)
    //check cloud empty
    if (!p_targetCloud->empty()){

        finePose = estimateFinePose( alignedSource, p_targetCloud );
    }
    else{
        finePose << Eigen::Matrix4f::Identity(); //1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; //identity
    }

    // Get Combined pose of coarse and fine
    Eigen::Matrix4f pose = coarsePose * finePose;

    // We use the previous aligned model as an initial estimate to current pose , so we calculate the pose of
    // previous aligned model
    Eigen::Matrix4f rigidmodelPose = Eigen::Matrix4f::Identity();
    if( firstTimePose != 0){

        //transformation estimation between model and sourcecloud
        pcl::Correspondence corr;
        pcl::Correspondences corres;
        for(int i=0; i < cloudModel->points.size(); ++i){
            corr.index_match = corr.index_query = i;
            corres.push_back(corr);
        }
        svd.estimateRigidTransformation(*cloudModel, *p_sourceCloud, corres, rigidmodelPose);
    }

    // Get the Final Pose
    finalPose = rigidmodelPose * pose;

    *p_sourceCloud = *alignedSource; // copy alignedsource for display

    // Update the scores
    fitnessScore = fitnessScoreFine;
    alignStrength = alignedStrength;
    return finalPose;

}
