/**
 * main function for the wholeBodyHPIDControl of the biped humanoid robot icub
 */


#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <limits>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Time.h>

// #include <wbi/wholeBodyInterface.h>
// #include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
// #include <wbi/wbiUtil.h>
// #include <codyco/ModelParsing.h>
// #include <codyco/Utils.h>

#include <yarp/os/RFModule.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

//#include "wbhpidcUtilityFunctions.hpp"

#include "WholeBodyTasksControlModule.h"   //
// #include "WBTasksQPControllerQTLw4.h"   // to go into the Module class
// #include <yarpWholeBodyInterface/yarpWbiUtil.h>
// #include <codyco/Utils.h>
// #include <paramHelp/paramHelperServer.h>
// #include <codyco/ModelParsing.h>
// #include <codyco/Utils.h>
// #include <codyco/PIDList.h>
// #include <yarp/os/Port.h>
// #include <yarp/os/LogStream.h>
// #include <yarp/os/LockGuard.h>
// #include <yarp/dev/ControlBoardPid.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::math;
// using namespace codyco;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;



// =========================================================================
// main function
// =========================================================================


int main(int argc, char **argv)
{
    //initialize the network
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5)) 
    {
        std::cerr << "YARP network is not available" << std::endl;
        return -1;
    }
    //
    if( argc != 4 ){
        std::cerr << "wholeBodyControl usage: ./WBTasksQPControllerNewV1 --from ../config/wholeBodyHPIDControl.ini dataID" << std::endl;
        return EXIT_FAILURE;
    }

    std::string n_data     = argv[3];

    // creation of a ressource finder object
    yarp::os::ResourceFinder rf; // = yarp::os::ResourceFinder::getResourceFinderSingleton();
    
    rf.setVerbose(true);										//logs searched directories
    rf.setDefaultConfigFile("wholeBodyHPIDControl.ini");      	//default config file name.
    //rf.setDefaultContext("wholeBodyHPIDControl"); 			//when no parameters are given to the module this is the default context
    rf.configure(argc, argv);
    
    if (rf.check("help")) 
    {
        std::cout<< "Possible parameters" << std::endl << std::endl;
        std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /wbtasksqpcontrollersqtlw4/conf" << std::endl;
        std::cout<< "\t--rate             :Period used by the module. Default set to 10ms." << std::endl;
        std::cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default." << std::endl;
        std::cout<< "\t--moduleName       :name of the module ." << std::endl;
        std::cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. wholeBodyHPIDControl." << std::endl;
//        codyco::iCubPartVersionOptionsPrint();
        return 0;
    }
    // /* create your module */
    WholeBodyTasksControlModule wbtask_module(0.04, n_data); 

    cout << "Configuring and starting module. \n";
    wbtask_module.runModule(rf);   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()

    //
    // wbtask_module.wbhpdi_Controller->stop();

    cout<<"Main returning..."<<endl;

	
    return 0;

}
