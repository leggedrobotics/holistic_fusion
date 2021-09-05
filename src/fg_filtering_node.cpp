// C++
#include <Python.h>
#include <string.h>

#include <boost/filesystem.hpp>
// ROS
#include <ros/ros.h>
// Local packages
#include "fg_filtering/FactorGraphFiltering.h"

// Main node entry point
int main(int argc, char** argv) {
  // ROS related
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  fg_filtering::FactorGraphFiltering fgFiltering;
  // Setup Instance
  if (fgFiltering.setup(node, privateNode)) {
    ROS_INFO("Node is set up completely.");
  }
  spinner.spin();

  // Cleanup and log signals
  fgFiltering.logSignals();

  // Create plots
  if (fgFiltering.getLogPlots()) {
    std::cout << "------------------------" << std::endl << "Plotting the logs..." << std::endl;
    Py_Initialize();

    boost::filesystem::path globalFileName(__FILE__);
    char pythonFileName[99];
    std::strcpy(pythonFileName, globalFileName.parent_path().c_str());
    std::cout << "Plotting: " << strcat(pythonFileName, "/../python/plot.py") << std::endl;
    FILE* pythonFile = _Py_fopen(pythonFileName, "r");
    PyRun_SimpleFile(pythonFile, pythonFileName);

    Py_Finalize();
    std::cout << "...done." << std::endl << "------------------------" << std::endl;
  } else {
    std::cout << "Logging of the plots is disabled." << std::endl;
  }

  // Destruct
  fgFiltering.~FactorGraphFiltering();

  return 0;
}
