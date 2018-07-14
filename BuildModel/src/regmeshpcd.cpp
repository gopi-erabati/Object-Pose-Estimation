#include "../include/regmeshpcd.h"

RegMeshPcd::RegMeshPcd()
{
}

//function to get ICP aligned pointcloud using single ICP
pcl::PointCloud<PointTReg>::Ptr RegMeshPcd::getIcp ( pcl::PointCloud<PointTReg>::Ptr p_cloudSource,
                                                                  pcl::PointCloud<PointTReg>::Ptr p_cloudTarget,
                                                                  float p_maxCorrDist, float p_ransacStatOutThresh, int p_maxIterations ){

    //cloud to store the result
    pcl::PointCloud<PointTReg>::Ptr cloudAligned (new pcl::PointCloud<PointTReg> );

    //create icp object
    pcl::IterativeClosestPoint<PointTReg, PointTReg> icp;

    // Set the input source and target
    icp.setInputSource ( p_cloudSource );
    icp.setInputTarget ( p_cloudTarget );

    // Set the max correspondence distance , correspondences with higher distances will be ignored
    icp.setMaxCorrespondenceDistance ( p_maxCorrDist );

    // set the ransac outlier rejection threshold
    icp.setRANSACOutlierRejectionThreshold( p_ransacStatOutThresh );

    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations ( p_maxIterations );

    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-16);

    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);


    // Perform the alignment
    icp.align ( *cloudAligned );

    std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;

    return cloudAligned;
}

pcl::PointCloud<PointTReg>::Ptr RegMeshPcd::getIcp2 ( pcl::PointCloud<PointTReg>::Ptr p_cloudSource,
                                                                  pcl::PointCloud<PointTReg>::Ptr p_cloudTarget,
                                                                  float p_maxCorrDist1, float p_ransacStatOutThresh1, int p_maxIterations1,
                                                                   float p_maxCorrDist2, float p_ransacStatOutThresh2, int p_maxIterations2){

    //cloud to store outputs
    pcl::PointCloud<PointTReg>::Ptr cloudAlign1 ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudAlign2 ( new pcl::PointCloud<PointTReg> );

    cloudAlign1 = getIcp( p_cloudSource, p_cloudTarget, p_maxCorrDist1, p_ransacStatOutThresh1, p_maxIterations1);
    cloudAlign2 = getIcp( cloudAlign1, p_cloudTarget, p_maxCorrDist2, p_ransacStatOutThresh2, p_maxIterations2);

    return cloudAlign2;
}


//function to get ICP aligned pointcloud using ICPwithNormals
pcl::PointCloud<PointTReg>::Ptr RegMeshPcd::getIcpNormal ( pcl::PointCloud<PointTReg>::Ptr p_cloudSource,
                                                            pcl::PointCloud<PointTReg>::Ptr p_cloudTarget,
                                                            float p_maxCorrDist, float p_ransacStatOutThresh, int p_maxIterations ){

    //clouds to store intermediate results
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSourceWithNormal ( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTargetWithNormal ( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudIcpNormal ( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    pcl::PointCloud<PointTReg>::Ptr cloudAligned ( new pcl::PointCloud<PointTReg> );

    //Normal Estimation
    pcl::NormalEstimation<PointTReg, pcl::PointXYZRGBNormal> normEst;
    //create kd tree for search
    pcl::search::KdTree<PointTReg>::Ptr kdtree ( new pcl::search::KdTree<PointTReg> );

    //set parameters
    normEst.setSearchMethod( kdtree );
    normEst.setKSearch( 12 );

    //computer noramls for Source
    normEst.setInputCloud( p_cloudSource );
    normEst.compute( *cloudSourceWithNormal );
    pcl::copyPointCloud( *p_cloudSource, *cloudSourceWithNormal );

    //compute normals for target
    normEst.setInputCloud( p_cloudTarget );
    normEst.compute( *cloudTargetWithNormal );
    pcl::copyPointCloud( *p_cloudTarget, *cloudTargetWithNormal );

//    //Initial alignmernt

//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceCloudFeatures;
//    // FPFH Descriptor for Source
//    pcl::FPFHEstimation<PointTReg, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfhEstimation;
//    fpfhEstimation.setInputCloud( p_cloudSource );
//    fpfhEstimation.setRadiusSearch( 0.02 );
//    fpfhEstimation.setInputNormals( cloudSourceWithNormal );
//    sourceCloudFeatures = pcl::PointCloud<pcl::FPFHSignature33>::Ptr ( new pcl::PointCloud<pcl::FPFHSignature33> );
//    fpfhEstimation.compute( *sourceCloudFeatures );

//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetCloudFeatures;
//    // FPFH Descriptor for target
//    fpfhEstimation.setInputCloud( p_cloudTarget );
//    fpfhEstimation.setRadiusSearch( 0.02 );
//    fpfhEstimation.setInputNormals( cloudTargetWithNormal );
//    targetCloudFeatures = pcl::PointCloud<pcl::FPFHSignature33>::Ptr ( new pcl::PointCloud<pcl::FPFHSignature33> );
//    fpfhEstimation.compute( *targetCloudFeatures );

//    //Sample Consensus Initail Alignment
//    pcl::PointCloud<PointTReg>::Ptr result ( new pcl::PointCloud<PointTReg> );
//    pcl::SampleConsensusInitialAlignment<PointTReg, PointTReg, pcl::FPFHSignature33> sacia;
//    sacia.setInputSource( p_cloudSource );
//    sacia.setInputTarget( p_cloudTarget );
//    sacia.setSourceFeatures( sourceCloudFeatures );
//    sacia.setTargetFeatures( targetCloudFeatures );
//    sacia.setMaximumIterations( 1000 );
//    sacia.setMaxCorrespondenceDistance( 0.1 );
//    sacia.setMinSampleDistance( 0.001f );
//    sacia.align( *result );
//    Eigen::Matrix4f pose = sacia.getFinalTransformation();
//    std::cout << "SCA_IA converged with score: " << sacia.getFitnessScore(0.05) << std::endl;

//    pcl::PointCloud<PointTReg>::Ptr alignedCloud ( new pcl::PointCloud<PointTReg> );
//    pcl::transformPointCloud( *p_cloudSource, *alignedCloud, pose);

//    //computer noramls for New Source
//    cloudSourceWithNormal.reset( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
//    normEst.setInputCloud( alignedCloud );
//    normEst.compute( *cloudSourceWithNormal );
//    pcl::copyPointCloud( *alignedCloud, *cloudSourceWithNormal );


    //Final alignment

    // Correspodndence estimation
    pcl::CorrespondencesPtr correspondences ( new pcl::Correspondences );
    //correspondence_estimation_noraml_shooting
    pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr corrEstNormShoot ( new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> );
    corrEstNormShoot->setInputSource( cloudSourceWithNormal );
    corrEstNormShoot->setSourceNormals( cloudSourceWithNormal );
    corrEstNormShoot->setInputTarget( cloudTargetWithNormal );
    corrEstNormShoot->setKSearch( 20 );
    corrEstNormShoot->determineCorrespondences( *correspondences, p_maxCorrDist );

    // Correspondence Rejection
    pcl::CorrespondencesPtr correspondencesFiltered ( new pcl::Correspondences );
    //correpondence_rejection_surface_normal
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr corrRejSurNorm (new pcl::registration::CorrespondenceRejectorSurfaceNormal );
    corrRejSurNorm->initializeDataContainer<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>();
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSourceWithNormalConstPtr = cloudSourceWithNormal;
    corrRejSurNorm->setInputSource<pcl::PointXYZRGBNormal>(cloudSourceWithNormalConstPtr);
    corrRejSurNorm->setInputNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloudSourceWithNormalConstPtr);
    const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTargetWithNormalConstPtr = cloudTargetWithNormal;
    corrRejSurNorm->setInputTarget<pcl::PointXYZRGBNormal>(cloudTargetWithNormalConstPtr);
    corrRejSurNorm->setTargetNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloudTargetWithNormalConstPtr);
    corrRejSurNorm->setThreshold( corrRejThreshNormAngle );
    corrRejSurNorm->getRemainingCorrespondences( *correspondences, *correspondencesFiltered );

    //transformation_estimation_point_to-plane
    pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr transfEstpointToPlane ( new pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> );


    //ICP With Normals
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icpNorm;

    // Set the input source and target
    icpNorm.setInputSource ( cloudSourceWithNormal );
    icpNorm.setInputTarget ( cloudTargetWithNormal );

    // Set the max correspondence distance , correspondences with higher distances will be ignored
    //icpNorm.setMaxCorrespondenceDistance ( p_maxCorrDist );

    // set the ransac outlier rejection threshold
    //icpNorm.setRANSACOutlierRejectionThreshold( p_ransacStatOutThresh );

    // Set the maximum number of iterations (criterion 1)
    icpNorm.setMaximumIterations ( p_maxIterations );

    // Set the transformation epsilon (criterion 2)
    icpNorm.setTransformationEpsilon (1e-8);

    icpNorm.setEuclideanFitnessEpsilon(1e-8);

    //set correspondence estimation
    icpNorm.setCorrespondenceEstimation( corrEstNormShoot );

    //set Rejectors
    icpNorm.addCorrespondenceRejector( corrRejSurNorm );

    //set transofrmation estimation
    icpNorm.setTransformationEstimation( transfEstpointToPlane );

    // Perform the alignment
    icpNorm.align ( *cloudIcpNormal );

    std::cout << "ICP converged with score: " << icpNorm.getFitnessScore() << std::endl;

    //get the final transformation and transform the source cloud
    Eigen::Matrix4f transformIcpNormal = icpNorm.getFinalTransformation();

    pcl::transformPointCloud( *p_cloudSource, *cloudAligned, transformIcpNormal );

    return cloudAligned;
}


//function to register pointclouds using ICPs
pcl::PointCloud<PointTReg>::Ptr RegMeshPcd::registerPointClouds ( std::vector<pcl::PointCloud<PointTReg>::Ptr> & cloudVector, float maxCorrDist, float corrRejThresh, int maxIter ){

    //clouds to store intermediate results
    pcl::PointCloud<PointTReg>::Ptr cloudSource ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudTarget ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudSourceDs ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudTargetDs ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudTemp ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudAlignedIcp ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudAlignedIcpOutRem ( new pcl::PointCloud<PointTReg> );
    pcl::PointCloud<PointTReg>::Ptr cloudAlignedIcpOutRemSmooth ( new pcl::PointCloud<PointTReg> );

    //set parameters for ICP
    float maxCorrDist1 = 0.05, ransacStatOutThresh1 = 0.01; int maxIterations1 = 80;
    float maxCorrDist2 = 0.005, ransacStatOutThresh2 = 0.02; int maxIterations2 = 500;
    float voxelGridLeafSize = 0.001, outlierRemovalThresh = 5.5;
    corrRejThreshNormAngle = corrRejThresh;

    //start the ICP between pointclouds
    cloudTemp = cloudVector[0]; //make first cloud as cloud source using a temp cloud

    //create objet for ProcessingPcd class to use its functions
    ProcessingPcd processingPcd;

    //loop for all clouds
    for (int i = 0; i < cloudVector.size()-1; i++){

        std::cout << "ICP between frame " << i << " and " << i+1 << std::endl;

        //assign source and target clouds
        cloudSource = cloudTemp;
        cloudTarget = cloudVector[i+1];


        //downsample both the clouds
        //cloudSourceDs = processingPcd.getDownSampled( cloudSource, voxelGridLeafSize );
        //cloudTargetDs = processingPcd.getDownSampled( cloudTarget, voxelGridLeafSize );

        //invoke ICP and align two clouds //three variants // choose the best one
        //cloudAlignedIcp = getIcp( cloudSourceDs, cloudTargetDs, maxCorrDist1, ransacStatOutThresh1, maxIterations1 );
        //cloudAlignedIcp = getIcp2( cloudSourceDs, cloudTargetDs, maxCorrDist1, ransacStatOutThresh1, maxIterations1, maxCorrDist2, ransacStatOutThresh2, maxIterations2 );
        cloudAlignedIcp = getIcpNormal( cloudSource, cloudTarget, maxCorrDist, ransacStatOutThresh2, maxIter );

        //combine two aligned pointclouds
        *cloudAlignedIcp += *cloudTarget;

        //remove outliers if any in aligned cloud

        *cloudTemp = *cloudAlignedIcp;

        //cloudTemp = processingPcd.getOutlierRemove( cloudAlignedIcp, outlierRemovalThresh );

    }

    //smooth overall aligned pointcloud
    *cloudAlignedIcpOutRem = *cloudTemp;
    //cloudAlignedIcpOutRemSmooth = processingPcd.getSmooth( cloudAlignedIcpOutRem, 0.02 );

    //cloudAlignedIcpOutRem = processingPcd.getOutlierRemove( cloudTemp, outlierRemovalThresh );
    return cloudAlignedIcpOutRem;

}

pcl::PolygonMesh RegMeshPcd::generateMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud )
{
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; //upsampling object

    //UpSampling using MLS VoxelGrid Dilation
    mls.setInputCloud (p_cloud);
    mls.setSearchRadius (0.03);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (4);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
    //mls.setUpsamplingRadius (0.005);
    //mls.setUpsamplingStepSize (0.003);
    mls.setDilationVoxelSize(0.002);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSmoothed (new pcl::PointCloud<pcl::PointXYZ>);
    mls.process (*cloudSmoothed);

    //Normal Estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloudSmoothed);
    normEst.setInputCloud (cloudSmoothed);
    normEst.setSearchMethod (tree);
    normEst.setKSearch (20);
    normEst.compute (*cloudNormals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloudSmoothed, *cloudNormals, *cloudWithNormals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloudWithNormals);

    boost::shared_ptr<pcl::PolygonMesh> triangles (new pcl::PolygonMesh);
    //Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    // Set typical values for the parameters
    gp3.setMu (3);//3
    gp3.setSearchRadius (0.025);
    gp3.setMaximumNearestNeighbors (1200);
    gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees  //2
    gp3.setMinimumAngle(M_PI/36); // 10 degrees //5deg
    gp3.setMaximumAngle(2*M_PI/3 + M_PI/6); // 120 degrees  // 150 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud (cloudWithNormals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*triangles);
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    //Laplacian Smoothing of mesh
    pcl::PolygonMesh output;
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(triangles);
    vtk.setNumIter(20000);
    vtk.setConvergence(0.0001);
    vtk.setRelaxationFactor(0.0001);
    vtk.setFeatureEdgeSmoothing(true);
    vtk.setFeatureAngle(M_PI/5);
    vtk.setBoundarySmoothing(true);
    vtk.process(output);
    //pcl::io::saveVTKFile ("../3DModel/brickModelAxisAlignedOrigin.vtk", output);

    return output;
}
