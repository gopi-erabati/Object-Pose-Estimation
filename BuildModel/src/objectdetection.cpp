#include "objectdetection.h"

ObjectDetection::ObjectDetection()
{


}

void ObjectDetection::getVfhFeature ( pcl::PointCloud<PointTDet>::Ptr p_cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr &p_cloudVfh){

    //Normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> );
    kdTree = pcl::search::KdTree<PointTDet>::Ptr ( new pcl::search::KdTree<PointTDet> );

    normEst.setInputCloud( p_cloud );
    normEst.setSearchMethod( kdTree );
    normEst.setKSearch( 30 );
    normEst.compute( *cloudWithNormals );
    pcl::copyPointCloud( *p_cloud, *cloudWithNormals );

    kdTree.reset( new pcl::search::KdTree<PointTDet> );
    p_cloudVfh = pcl::PointCloud<pcl::VFHSignature308>::Ptr ( new pcl::PointCloud<pcl::VFHSignature308> );

    vfhEst.setInputCloud( p_cloud );
    vfhEst.setInputNormals( cloudWithNormals );
    vfhEst.setSearchMethod( kdTree );
    vfhEst.compute( *p_cloudVfh);

}

// training stage to get kdTree representation of VFH descriptors
void ObjectDetection::getkdTreeRepresentation (const boost::filesystem::path &baseDir){


    std::string extension (".pcd");
    std::transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    std::string kdtree_idx_file_name = "../3DModel/TrainData/kdtree.idx";
    std::string training_data_h5_file_name = "../3DModel/TrainData/training_data.h5";
    std::string training_data_list_file_name = "../3DModel/TrainData/training_data.list";

    std::vector<vfhModel> models;

    // Load the model histograms
    loadFeatureModels (baseDir, extension, models);
    pcl::console::print_error ("Loaded %d VFH models. Creating training data %s/%s.\n",
                               (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

    for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
            data[i][j] = models[i].second[j];

    // Save data to disk (list of models)
    flann::save_to_file (data, training_data_h5_file_name, "training_data");
    std::ofstream fs;
    fs.open (training_data_list_file_name.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
    fs.close ();

    // Build the tree index and save it to disk
    pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
    //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (1));
    //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::AutotunedIndexParams(1.0, 0.01, 0, 1.0) );

    index.buildIndex ();
    index.save (kdtree_idx_file_name);
    delete[] data.ptr ();

    std::cout << "Done with kdTree" << std::endl;

}

void
ObjectDetection::loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                                    std::vector<vfhModel> &models)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
        return;

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
            std::stringstream ss;
            ss << it->path ();
            pcl::console::print_error ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
            loadFeatureModels (it->path (), extension, models);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
            vfhModel m;
            if (loadHist (base_dir / it->path ().filename (), m))
                models.push_back (m);
        }
    }
}

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
ObjectDetection::loadHist (const boost::filesystem::path &path, vfhModel &vfh)
{
    int vfh_idx;
    // Load the file as a PCD
    try
    {
        pcl::PCLPointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        pcl::PCDReader r;
        int type; unsigned int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

        vfh_idx = pcl::getFieldIndex (cloud, "vfh");
        if (vfh_idx == -1)
            return (false);
        if ((int)cloud.width * cloud.height != 1)
            return (false);
    }
    catch (const pcl::InvalidConversionException&)
    {
        return (false);
    }

    // Treat the VFH signature as a single Point Cloud
    pcl::PointCloud <pcl::VFHSignature308> point;
    pcl::io::loadPCDFile (path.string (), point);
    vfh.second.resize (308);

    std::vector <pcl::PCLPointField> fields;
    pcl::getFieldIndex (point, "vfh", fields);

    for (size_t i = 0; i < fields[vfh_idx].count; ++i)
    {
        vfh.second[i] = point.points[0].histogram[i];
    }
    std::string objName =  path.filename().string();
    vfh.first = objName;
    return (true);
}

bool ObjectDetection::getObjectName(pcl::PointCloud<PointTDet>::Ptr p_cloud, std::string &p_objName){

    //get VFH Signature
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloudVfh ( new pcl::PointCloud<pcl::VFHSignature308> );
    getVfhFeature( p_cloud, cloudVfh);

    int k = 15;
    double thresh = 120;
    std::string extension (".pcd");
    std::transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

    //load Model
    vfhModel objVfh;

    std::vector<pcl::PCLPointField> fields;
    int vfhIndex = pcl::getFieldIndex(*cloudVfh, "vfh", fields);

    objVfh.second.resize( 308 );
    for (size_t i = 0; i < fields[vfhIndex].count; ++i)
    {
        objVfh.second[i] = cloudVfh->points[0].histogram[i];
    }
    objVfh.first = "testCloud";

    //KNN
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    nearestKSearch (*index, objVfh, k, k_indices, k_distances);

    std::cout << "The distance of 1-NN is : " << k_distances[0][1] << std::endl;

    //Check for first nieighbor
    if (k_distances[0][1] < thresh){

        p_objName = models.at (k_indices[0][1]).first.c_str ();
        std::vector<std::string> fieldString;
        boost::split( fieldString, p_objName, boost::is_any_of( "_" ) );
        p_objName = fieldString.at(0);
        return true;
    }
    else{
        p_objName = "ObjectNotFound";
        return false;
    }


//    std::vector<std::string> objNames;
//    //store names in vector
//    for (int i = 0 ; i < 5; i++){
//        std::string objNameTemp = models.at(k_indices[0][i]).first.c_str();
//        std::vector<std::string> fieldString;
//        boost::split( fieldString, objNameTemp, boost::is_any_of( "_" ) );
//        objNameTemp = fieldString.at(0);
//        std::cout << objNameTemp << "  " << k_distances[0][i] << ";";
//        objNames.push_back( objNameTemp );
//    }
//    std::cout << "\n" << std::endl;

//    //find Mode
//    //Finding the mode of the  words
//    int _count = 0;
//    int _max = 0;
//    std::string mode;
//    for (unsigned int test = 0, j = 1; test < objNames.size()-1; test++, j++) {
//        if (objNames[test] == objNames[j]) {
//            _count++;
//        }
//        else if (objNames[test] != objNames[j]) {
//            if (_count>_max) {
//                _max = _count;
//                mode = objNames[test];
//            }

//            _count = 0;

//        }
//    }
//    p_objName = mode;
//    return true;
}


void ObjectDetection::loadTrainData(){

    std::string training_data_list_file_name = "../3DModel/TrainData/training_data.list";
    std::string training_data_h5_file_name = "../3DModel/TrainData/training_data.h5";
    flann::Matrix<float> data;

    // Check if the data has already been saved to disk
    if (!boost::filesystem::exists ("../3DModel/TrainData/training_data.list") || !boost::filesystem::exists ("../3DModel/TrainData/training_data.h5"))
    {
        pcl::console::print_error ("Could not find training data models files %s!\n",
                                   training_data_list_file_name.c_str ());
    }
    else
    {
        loadFileList (models, training_data_list_file_name);
        flann::load_from_file (data, training_data_h5_file_name, "training_data");
        pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s.\n",
                                      (int)data.rows, training_data_list_file_name.c_str ());
    }

    std::string kdtree_idx_file_name = "../3DModel/TrainData/kdtree.idx";
    // Check if the tree index has already been saved to disk
    if (!boost::filesystem::exists (kdtree_idx_file_name))
    {
        pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
    }
    else
    {
        index = new flann::Index<flann::ChiSquareDistance<float> > (data, flann::SavedIndexParams ("../3DModel/TrainData/kdtree.idx"));
        index->buildIndex();
        pcl::console::print_highlight( "Loaded kdTree and built! \n");
    }
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool ObjectDetection::loadFileList (std::vector<vfhModel> &models, const std::string &filename)
{
  std::ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfhModel m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
ObjectDetection::nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfhModel &model,
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}
