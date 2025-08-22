#ifndef GRAPHMSFROS_H
#define GRAPHMSFROS_H

// std
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

// ROS2
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Workspace
#include "graph_msf/interface/GraphMsf.h"
#include "graph_msf/interface/GraphMsfClassic.h"
#include "graph_msf/interface/GraphMsfHolistic.h"

// Macros
#define ROS_QUEUE_SIZE 1

namespace graph_msf {

// Structure to hold non-time-critical data for worker thread processing
struct NonTimeCriticalData {
  Eigen::Matrix<double, 6, 6> poseCovarianceRos;
  Eigen::Matrix<double, 6, 6> twistCovarianceRos;
  Eigen::Vector3d positionVarianceRos;
  Eigen::Vector3d orientationVarianceRos;
  std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr;
  std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr;

  NonTimeCriticalData(const Eigen::Matrix<double, 6, 6>& poseCovariance,
                      const Eigen::Matrix<double, 6, 6>& twistCovariance,
                      const Eigen::Vector3d& positionVariance,
                      const Eigen::Vector3d& orientationVariance,
                      std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedState,
                      std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedState)
      : poseCovarianceRos(poseCovariance),
        twistCovarianceRos(twistCovariance),
        positionVarianceRos(positionVariance),
        orientationVarianceRos(orientationVariance),
        integratedNavStatePtr(integratedState),
        optimizedStateWithCovarianceAndBiasPtr(optimizedState) {}
};

class GraphMsfRos2 : public GraphMsfClassic, public GraphMsfHolistic, public rclcpp::Node {
 public:
  GraphMsfRos2(const std::string& nodeName, const rclcpp::NodeOptions& options);
  // Destructor
  ~GraphMsfRos2() override;

  // Setup
  void setup(std::shared_ptr<StaticTransforms> staticTransformsPtr);

 protected:
  // Functions that need implementation
  virtual void initializePublishers();
  virtual void initializeSubscribers();
  virtual void initializeMessages();
  virtual void initializeServices(rclcpp::Node& node);

  // Commodity Functions to be shared -----------------------------------
  // Static
  static void addToPathMsg(const nav_msgs::msg::Path::SharedPtr& pathPtr, const std::string& frameName,
                           const rclcpp::Time& stamp, const Eigen::Vector3d& t, int maxBufferLength);
  static void addToOdometryMsg(
      const nav_msgs::msg::Odometry::SharedPtr& msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
      const rclcpp::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& W_v_W_F,
      const Eigen::Vector3d& W_w_W_F,
      const Eigen::Matrix<double, 6, 6>& poseCovariance = Eigen::Matrix<double, 6, 6>::Zero(),
      const Eigen::Matrix<double, 6, 6>& twistCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  static void addToPoseWithCovarianceStampedMsg(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msgPtr, const std::string& frameName,
      const rclcpp::Time& stamp, const Eigen::Isometry3d& T,
      const Eigen::Matrix<double, 6, 6>& transformCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  static void extractCovariancesFromOptimizedState(
      Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  static void createVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                                   const Eigen::Vector3d& velocity, visualization_msgs::msg::Marker& marker);
  void createContactMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                           const Eigen::Vector3d& position, const std::string& nameSpace, const int id,
                           visualization_msgs::msg::Marker& marker);

  // Parameter Loading -----------------------------------
  virtual void declareRosParams();
  virtual void readParams();

  // Callbacks
  virtual void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr);

  // Services
  bool srvOfflineSmootherOptimizeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Publishing -----------------------------------
  virtual void publishState(
      const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  void publishNonTimeCriticalData(
      const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
      const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
      const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr);
  void publishOptimizedStateAndBias(
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
      const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos, const double timeStamp);

  void publishTfTreeTransform(const std::string& frameName, const std::string& childFrameName, double timeStamp,
                              const Eigen::Isometry3d& T_frame_childFrame) const;
  void publishImuOdoms(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                       const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                       const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const;
  void publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                  const double timeStamp) const;
  void publishVelocityMarkers(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishImuPaths(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishAddedImuMeas(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const rclcpp::Time& stamp) const;

  // Measure time
  long secondsSinceStart();

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;

  // Publishers
  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  // Members
  std::string referenceFrameAlignedNameId = "_graph_msf_aligned";
  std::string sensorFrameCorrectedNameId = "_graph_msf_corrected";
  std::string optimizationResultLoggingPath = "";

 private:
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubEstOdomImu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubEstWorldImu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOptWorldImu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubEstWorldPosVariance_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubEstWorldRotVariance_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubVelocityMarker_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubEstOdomImuPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubEstWorldImuPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOptWorldImuPath_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubAccelBias_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubGyroBias_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubAddedImuMeas_;

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      pubPoseStampedByTopicMap_ = {};

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;

  // Messages
  nav_msgs::msg::Odometry::SharedPtr estOdomImuMsgPtr_;
  nav_msgs::msg::Odometry::SharedPtr estWorldImuMsgPtr_;
  nav_msgs::msg::Odometry::SharedPtr optWorldImuMsgPtr_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr estWorldPosVarianceMsgPtr_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr estWorldRotVarianceMsgPtr_;
  nav_msgs::msg::Path::SharedPtr estOdomImuPathPtr_;
  nav_msgs::msg::Path::SharedPtr estWorldImuPathPtr_;
  nav_msgs::msg::Path::SharedPtr optWorldImuPathPtr_;
  nav_msgs::msg::Path::SharedPtr measWorldLidarPathPtr_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr accelBiasMsgPtr_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr gyroBiasMsgPtr_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvSmootherOptimize_;

  // Last Optimized State Timestamp
  double lastOptimizedStateTimestamp_ = 0.0;
  double lastIntegratedStateTimestamp_ = 0.0;

  // Worker thread pool for non-time-critical publishing
  std::queue<NonTimeCriticalData> nonTimeCriticalQueue_;
  std::mutex queueMutex_;
  std::condition_variable queueCondition_;
  std::thread workerThread_;
  std::atomic<bool> shutdownRequested_{false};
  
  void workerThreadFunction();

  // Mutex
  std::mutex rosPublisherMutex_;
};
}  // namespace graph_msf

#endif  // end GRAPHMSFROS_H
