#include <boost/lexical_cast.hpp>
#include <pcl/common/common.h>

#include "global.h"
#include "rosinterface.h"


int
main(int argc, char** argv)
{

    // ******************************** Command line parser for arguments *************************************
    cv::CommandLineParser parser(argc, argv,
                             #if CV_MAJOR_VERSION == 3
                                 "{ h help     | false   | print this message   }"
                                 "{ o object   | brick   | Object to get pose [brick(default); drill ; yellow]}"
                                 "{ sc scenario | 0       | Scenario table -> 0 ; nottable -> 1   }"
                                 "{ l limits   | -0.5,0.5,-0.5,0.3,0.6,1.7        | X, Y and Z limits from camera(xmin,xmax,ymin,ymax,zmin,zmax)(no spaces!)  }"
                                 "{ p path     | empty        | Path to new object model (pointcloud (.pcd, .ply) format)}"
                                 "{ s sensor   | astra        | Sensor to choose [kinect, astra, euclid]}"
                             #else
                                 "{ h | help     | false   | print this message   }"
                                 "{ o | object   | brick   | Object to get pose [brick(default); drill ; yellow]}"
                                 "{ sc | scenario | 0       | Scenario table -> 0 ; nottable -> 1   }"
                                 "{ l | limits   | -0.5,0.5,-0.5,0.3,0.6,1.7        | X, Y and Z limits from camera(xmin,xmax,ymin,ymax,zmin,zmax)(no spaces!)  }"
                                 "{ p | path     | empty        | Path to new object model (pointcloud (.pcd, .ply) format)}"
                                 "{ s | sensor   | astra        | Sensor to choose [kinect, astra, euclid]}"
                             #endif
                                 );

    // ******************************** Get Values of Different Arguments *************************************
    //print help
    bool helpReq = parser.get<bool>("h");
    if ( helpReq )
    {
#if CV_MAJOR_VERSION == 3
        parser.printMessage();
#else
        parser.printParams();
#endif
        //parser.printMessage();
        return 0;
    }

    //to get object name to estimate and scenario
    std::string objName = parser.get<std::string>("o");
    int scenario = parser.get<int>("sc");

    // to get limits of the vieweing space of sensor
    std::string limits = parser.get<std::string> ("l");
    std::vector<std::string> fieldString;
    boost::split( fieldString, limits, boost::is_any_of( "," ) );
    //std::cout << boost::lexical_cast<float>( fieldString.at(0)) << " , " << fieldString.at(5) << std::endl;
    std::vector<float> limitsD;
    for(int i = 0; i < fieldString.size(); ++i){
        limitsD.push_back(boost::lexical_cast<float>( fieldString.at(i)));
    }

    //to get object path and check its validity
    std::string path = parser.get<std::string>("p");
    //Check path given as new object model is valid
    if(path != "empty"){
        boost::filesystem::path pathToObj(path);
        if( !boost::filesystem::exists(pathToObj)){
            std::cout << "Path to new object doesn't exists!";
            return 0;

        }
        if ( (pathToObj.extension() != ".pcd") && (pathToObj.extension() != ".ply")){
            std::cout << "Only .pcd and .ply extension files possible" << std::endl;
            return 0;
        }
    }

    // to get Sensor model type
    std::string sensor = parser.get<std::string>("s");
    bool kinect(false), astra(false), euclid(false);
    if (sensor == "kinect"){
        kinect = true;
    }
    else if (sensor == "astra"){
        astra = true;
    }
    else if (sensor == "euclid"){
        euclid = true;
    }
    else{
        std::cerr << "Invalid Sensor Type" ;
        return 0;
    }


    // ********************************** Connection to ROS Interface and start Pose Estimation ********************************************************

    RosInterface rosInterface( euclid, kinect, astra);
    rosInterface.startRosInterface( argc, argv, objName, scenario, limitsD, path);

    return 0;
}


