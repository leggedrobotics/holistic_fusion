#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "loam/Angle.h"
#include "loam/Twist.h"
#include "math_utils.h"
#include "loam/GraphManager.hpp"
#include "loam/ImuManager.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace fg_filtering {

/** \brief Implementation of the LOAM laser odometry component.
 *
 */
class FactorGraphFiltering {
 public:
  explicit FactorGraphFiltering(float scanPeriod = 0.1);

  /** \brief Setup component.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /** \brief Try to process buffered data. */
  void process();

  //CUSTOMIZATION
  /** \brief IMU Callback Function for handling incoming IMU messages
     *
     * @param imu_ptr pointer to incoming ROS IMU message
  */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr);

  //CUSTOMIZATION
  // Set frames and parameters for integration of external motion priors
  void setLidarFrame(const std::string& s) { _lidarFrame = s; }
  void setImuFrame(const std::string& s) { _imuFrame = s; }
  void setImuTimeOffset(const double d) { _imuTimeOffset = d; }
  void setZeroMotionDetection(const bool b) { _zeroMotionDetection = b; }
  void setVerboseLevel(int verbose) { _verboseLevel = verbose; }
  auto const& transformSum() { return _transformSum; }
  auto const& graphIMUBias() const { return _imuBiasMsg; }

 protected:
  /** \brief Publish the current result via the respective topics. */
  void publishResult();
    /** \brief IMU buffer*/
  loam::ImuManager _imuBuffer;
  /** \brief Factor graph*/
  loam::GraphManager _graphMgr;

 private:
  /** \brief Transform the given point to the start of the sweep.
   *
   * @param pi the point to transform
   * @param po the point instance for storing the result
   */
  void transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po);

  void pluginIMURotation(const loam::Angle& bcx, const loam::Angle& bcy, const loam::Angle& bcz,
                         const loam::Angle& blx, const loam::Angle& bly, const loam::Angle& blz,
                         const loam::Angle& alx, const loam::Angle& aly, const loam::Angle& alz,
                         loam::Angle& acx, loam::Angle& acy, loam::Angle& acz);

  void accumulateRotation(loam::Angle cx, loam::Angle cy, loam::Angle cz,
                          loam::Angle lx, loam::Angle ly, loam::Angle lz,
                          loam::Angle& ox, loam::Angle& oy, loam::Angle& oz);

  bool updateFactorGraph();

  //Member variables
  float _scanPeriod;      ///< time per scan
  long _frameCount;       ///< number of processed frames
  size_t _maxIterations;  ///< maximum number of iterations
  size_t _minIterations;  ///< minimum number of iterations
  bool _systemInited;     ///< initialization flag

  float _deltaTAbort;  ///< optimization abort threshold for deltaT
  float _deltaRAbort;  ///< optimization abort threshold for deltaR
  uint16_t _ioRatio;  ///< ratio of input to output frames

  loam::Twist _transform;     ///< optimized pose transformation //smk: also used as motion prior, also adjusted by IMU or VIO(if needed)
  loam::Twist _transformSum;  ///< accumulated optimized pose transformation

  ros::Time _timeImuTrans;               ///< time of current IMU transformation information
  ros::Time _timeUpdate;

  tf::StampedTransform _odometryTrans;  ///< odometry transformation

  ros::Publisher _pubOdometry;         ///< laser odometry publisher
  ros::Publisher _pubLaserImuBias;          ///< laser odometry imu bias publisher
  tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

  ros::Subscriber _subImuTrans;  ///< IMU transformation information message subscriber
  ros::Subscriber _subImu;       ///< IMU subscriber

  std::string _fgFrameId;                                                              // Frame id of camera

  //Graph Parameters
  std::string _lidarFrame = "";              // LiDAR frame name - used to lookup LiDAR-to-ExternalSensor Frame
  std::string _imuFrame = "";                           //IMU frame name - used to lookup LiDAR-to-IMU Frame
  double _imuTimeOffset = 0.0;                          //Offset between IMU and LiDAR Measurements - Depending on LiDAR timestamp first(+0.05) or last(-0.05)
  Eigen::Matrix4d _T_LB = Eigen::Matrix4d::Identity();  //IMU to LiDAR expressed in IMU frame, Read as Transforms LiDAR into IMU
  Eigen::Matrix4d _T_BL = Eigen::Matrix4d::Identity();  //LiDAR to IMU expressed in LiDAR frame, Read as Transforms IMU into LiDAR
  bool _gravityAttitudeInit = false;                    //Flag if attitude from gravity were initialized
  tf::Transform _gravityAttitude;                       //Attitude from initial gravity alignment
  sensor_msgs::Imu _imuBiasMsg;                         //IMU bias publishing ROS message
  bool _zeroMotionDetection = false;                    //Detect and Add Zero Motion Factors(Zero delta Pose and Velocity)
  
  int _verboseLevel = 0;
};

}  // end namespace fg_filtering

#endif  //LOAM_LASERODOMETRY_H
