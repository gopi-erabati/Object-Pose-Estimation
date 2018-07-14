#include "../include/processingpcd.h"

ProcessingPcd::ProcessingPcd()
{
}

//function to apply pass through filter to pointclouds in X, Y and Z limits
pcl::PointCloud<PointTProc>::Ptr ProcessingPcd::getPassThrough( pcl::PointCloud<PointTProc>::Ptr p_cloud,
                                                                            float p_minX, float p_maxX, float p_minY, float p_maxY, float p_minZ, float p_maxZ ){


    //create temp clouds to store intermediate results
    pcl::PointCloud<PointTProc>::Ptr cloudFilteredZ (new pcl::PointCloud<PointTProc> );
    pcl::PointCloud<PointTProc>::Ptr cloudFilteredZY (new pcl::PointCloud<PointTProc> );
    pcl::PointCloud<PointTProc>::Ptr cloudFilteredZYX (new pcl::PointCloud<PointTProc> );

    //create pass through filter object
    pcl::PassThrough<PointTProc> passThrough;

    //filter along Z direction with limits specified
    passThrough.setInputCloud( p_cloud );
    passThrough.setFilterFieldName( "z" );
    passThrough.setFilterLimits( p_minZ, p_maxZ );
    passThrough.filter( *cloudFilteredZ );

    //filter along Y direction with limits specified
    passThrough.setInputCloud( cloudFilteredZ );
    passThrough.setFilterFieldName( "y" );
    passThrough.setFilterLimits( p_minY, p_maxY );
    passThrough.filter( *cloudFilteredZY );

    //filter along X direction with limits specified
    passThrough.setInputCloud( cloudFilteredZY );
    passThrough.setFilterFieldName( "x" );
    passThrough.setFilterLimits( p_minX, p_maxX );
    passThrough.filter( *cloudFilteredZYX );

    return cloudFilteredZYX;


}


//function to apply voxel grid downsampling if required
pcl::PointCloud<PointTProc>::Ptr ProcessingPcd::getDownSampled( pcl::PointCloud<PointTProc>::Ptr p_cloud, float p_leafSize){

    //temporary cloud to store result
    pcl::PointCloud<PointTProc>::Ptr cloudDownSampled ( new pcl::PointCloud<PointTProc> );

    //create vocelgridDS object
    pcl::VoxelGrid<PointTProc> voxGrid;

    //apply voxelDS
    voxGrid.setInputCloud( p_cloud );
    voxGrid.setLeafSize( p_leafSize, p_leafSize, p_leafSize );
    voxGrid.filter( *cloudDownSampled );

    return cloudDownSampled;
}

//function to remove outliers
pcl::PointCloud<PointTProc>::Ptr ProcessingPcd::getOutlierRemove( pcl::PointCloud<PointTProc>::Ptr p_cloud, float p_threshold){

    //temporary cloud to store result
    pcl::PointCloud<PointTProc>::Ptr cloudSor ( new pcl::PointCloud<PointTProc> );

    //create SOR object
    pcl::StatisticalOutlierRemoval<PointTProc> sor;

    //apply SOR
    sor.setInputCloud( p_cloud );
    sor.setMeanK( 30 );
    sor.setStddevMulThresh( p_threshold );
    sor.filter( *cloudSor );

    return cloudSor;

}

//function to perform smoothing of pc
pcl::PointCloud<PointTProc>::Ptr ProcessingPcd::getSmooth( pcl::PointCloud<PointTProc>::Ptr p_cloud, float p_searchRadius){

    //temporary cloud to store result
    pcl::PointCloud<PointTProc>::Ptr cloudSmooth ( new pcl::PointCloud<PointTProc> );

    //MLS required kdtree, so create kdtree
    pcl::search::KdTree<PointTProc>::Ptr kdtree ( new pcl::search::KdTree<PointTProc> );

    //mls outputs a cloud with normals
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloudNormal;

    //create mls object
    pcl::MovingLeastSquares<PointTProc, pcl::PointXYZRGBNormal> mls;

    mls.setInputCloud( p_cloud );
    mls.setComputeNormals( false );
    mls.setPolynomialFit( true );
    mls.setSearchMethod( kdtree );
    mls.setSearchRadius( p_searchRadius );
    mls.process( cloudNormal );

    pcl::copyPointCloud( cloudNormal, *cloudSmooth );

    return cloudSmooth;



}


//function to filter based on color
pcl::PointCloud<PointTProc>::Ptr ProcessingPcd::getFilterRgb( pcl::PointCloud<PointTProc>::Ptr p_cloud, int p_minR, int p_maxR, int p_minG, int p_maxG, int p_minB, int p_maxB){

    pcl::PointCloud<PointTProc>::Ptr cloudFiltered (new pcl::PointCloud<PointTProc> );

    pcl::ConditionAnd<PointTProc>::Ptr colorCond ( new pcl::ConditionAnd<PointTProc> ());
    colorCond->addComparison(pcl::PackedRGBComparison<PointTProc>::Ptr (new pcl::PackedRGBComparison<PointTProc> ("r", pcl::ComparisonOps::LT, p_maxR)));
    colorCond->addComparison(pcl::PackedRGBComparison<PointTProc>::Ptr (new pcl::PackedRGBComparison<PointTProc> ("r", pcl::ComparisonOps::GT, p_minR)));
    colorCond->addComparison(pcl::PackedRGBComparison<PointTProc>::Ptr (new pcl::PackedRGBComparison<PointTProc> ("g", pcl::ComparisonOps::LT, p_maxG)));
    colorCond->addComparison(pcl::PackedRGBComparison<PointTProc>::Ptr (new pcl::PackedRGBComparison<PointTProc> ("g", pcl::ComparisonOps::GT, p_minG)));
    colorCond->addComparison(pcl::PackedRGBComparison<PointTProc>::Ptr (new pcl::PackedRGBComparison<PointTProc> ("b", pcl::ComparisonOps::LT, p_maxB)));
    colorCond->addComparison(pcl::PackedRGBComparison<PointTProc>::Ptr (new pcl::PackedRGBComparison<PointTProc> ("b", pcl::ComparisonOps::GT, p_minB)));

//    colorCond->addComparison (pcl::FieldComparison<PointTProc>::Ptr (new pcl::FieldComparison<PointTProc>("r", pcl::ComparisonOps::LT, p_maxR)));
//    colorCond->addComparison (pcl::FieldComparison<PointTProc>::Ptr (new pcl::FieldComparison<PointTProc>("r", pcl::ComparisonOps::GT, p_minR)));
//    colorCond->addComparison (pcl::FieldComparison<PointTProc>::Ptr (new pcl::FieldComparison<PointTProc>("g", pcl::ComparisonOps::LT, p_maxG)));
//    colorCond->addComparison (pcl::FieldComparison<PointTProc>::Ptr (new pcl::FieldComparison<PointTProc>("g", pcl::ComparisonOps::GT, p_minG)));
//    colorCond->addComparison (pcl::FieldComparison<PointTProc>::Ptr (new pcl::FieldComparison<PointTProc>("b", pcl::ComparisonOps::LT, p_maxB)));
//    colorCond->addComparison (pcl::FieldComparison<PointTProc>::Ptr (new pcl::FieldComparison<PointTProc>("b", pcl::ComparisonOps::GT, p_minB)));

    //build filter
    pcl::ConditionalRemoval<PointTProc> condRem( colorCond );

    condRem.setInputCloud( p_cloud );
    condRem.setKeepOrganized( true );

    condRem.filter( *cloudFiltered );

    return cloudFiltered;

}
