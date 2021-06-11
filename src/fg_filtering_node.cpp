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
  ros::MultiThreadedSpinner spinner(3);

  // Create Instance
  fg_filtering::FactorGraphFiltering fgFilter(0.1);
  // Setup Instance
  if (fgFilter.setup(node, privateNode)) ROS_INFO("Node is set up completely.");
  spinner.spin();

  // Create plots
  std::cout << "------------------------" << std::endl << "Plotting the logs..." << std::endl;
  Py_Initialize();

  boost::filesystem::path globalFileName(__FILE__);
  char pythonFileName[99];
  std::strcpy(pythonFileName, globalFileName.parent_path().c_str());
  std::cout << "Test: " << strcat(pythonFileName, "/../python/plot.py") << std::endl;
  FILE* pythonFile = _Py_fopen(pythonFileName, "r");
  PyRun_SimpleFile(pythonFile, pythonFileName);

  Py_Finalize();
  std::cout << "...done." << std::endl << "------------------------" << std::endl;

  return 0;
}
