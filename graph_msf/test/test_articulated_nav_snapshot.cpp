#include <gtest/gtest.h>
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/interface/NavState.h"

namespace graph_msf {
namespace {
TEST(ArticulatedNavigationSnapshot, ReturnedCopyRetainsItsOptimizerRevision) {
  SafeIntegratedNavState original(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
                                  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0, false);
  ArticulatedTurnState first;
  first.graph_key=10; first.position_rad=.1;
  original.setArticulatedTurnState(std::make_shared<const ArticulatedTurnState>(first));
  const SafeIntegratedNavState returned(original);
  ArticulatedTurnState second;
  second.graph_key=11; second.position_rad=.2;
  original.setArticulatedTurnState(std::make_shared<const ArticulatedTurnState>(second));
  ASSERT_TRUE(returned.getArticulatedTurnState());
  EXPECT_EQ(returned.getArticulatedTurnState()->graph_key,10u);
  EXPECT_DOUBLE_EQ(returned.getArticulatedTurnState()->position_rad,.1);
  EXPECT_EQ(original.getArticulatedTurnState()->graph_key,11u);
}

TEST(ArticulatedNavigationSnapshot, OptimizationBetweenGraphKeysPreservesImuTailAndSnapshot) {
  auto config=std::make_shared<GraphConfig>();
  config->useImuSignalLowPassFilter_=false;
  config->use2ndOrderCoriolisFlag_=false;
  config->optimizeReferenceFramePosesWrtWorldFlag_=false;
  config->initialGyroBiasNoiseDensity_=1e-5;
  config->realTimeSmootherLag_=1.0;
  config->relinearizeSkip_=1;
  GraphManager manager(config,"IMU","world");
  ArticulatedTurnConfig turn_config;
  turn_config.enable=true;
  manager.configureArticulatedTurn(turn_config);
  ASSERT_TRUE(manager.initImuIntegrators(9.81));
  ASSERT_TRUE(manager.initPoseVelocityBiasGraph(1.0,gtsam::Pose3(),gtsam::Pose3()));
  auto imu=std::make_shared<ImuBuffer>(config);
  imu->addToImuBuffer(.99,Eigen::Vector3d(0,0,9.81),Eigen::Vector3d(0,0,.2));
  imu->addToImuBuffer(1.0,Eigen::Vector3d(0,0,9.81),Eigen::Vector3d(0,0,.2));
  SafeIntegratedNavState returned;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimized;
  std::shared_ptr<const ArticulatedTurnState> first_returned;
  for(int i=1;i<=8;++i) {
    const double t=1.0+.01*i;
    ArticulatedTurnSample sample;
    sample.stamp_sec=t;
    sample.initial_turn_reference_rad=.2*(t-1.0);
    sample.parked=true;
    for(uint32_t sensor=0;sensor<2;++sensor) {
      ArticulatedTurnLegSample leg;
      leg.sensor_id=sensor; leg.stamp_sec=t;
      leg.vertical_rate_rad_s=0; leg.gravity_sensor=Eigen::Vector3d::UnitZ();
      sample.legs.push_back(leg);
    }
    ASSERT_TRUE(manager.addArticulatedTurnSample(sample));
    imu->addToImuBuffer(t,Eigen::Vector3d(0,0,9.81),Eigen::Vector3d(0,0,.2));
    manager.addImuFactorAndGetState(returned,optimized,imu,t,i%2==0);
    EXPECT_NEAR(gtsam::Rot3(returned.getT_W_Ik().rotation()).yaw(),.2*(t-1.0),1e-7);
    // Optimize between graph keys. The just-returned state must retain the
    // previous joint posterior, even though the manager gets a new revision.
    if(i==3 || i==5 || i==7) {
      const auto before=returned.getArticulatedTurnState();
      manager.updateGraph();
      EXPECT_EQ(returned.getArticulatedTurnState(),before);
      const auto current=manager.getArticulatedTurnState();
      EXPECT_FALSE(current.fault_latched) << current.reason;
      EXPECT_NEAR(current.stamp_sec,t-.01,1e-10);
      if(i==5) first_returned=before;
    }
  }
  ASSERT_TRUE(returned.getArticulatedTurnState());
  EXPECT_EQ(returned.getArticulatedTurnState()->graph_key,3u);
  ASSERT_TRUE(first_returned);
  EXPECT_EQ(first_returned->graph_key,1u);
}
} // namespace
} // namespace graph_msf
