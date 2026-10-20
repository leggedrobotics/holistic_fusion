#include <memory>

#include <gtest/gtest.h>

#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/core/TimeGraphKeyBuffer.h"
#include "graph_msf/imu/ImuBuffer.hpp"

namespace graph_msf {
namespace {

class TimeGraphKeyBufferLookupTest : public ::testing::Test {
 protected:
  void SetUp() override {
    buffer_.addToBuffer(10.0, 100);
    buffer_.addToBuffer(20.0, 200);
  }

  TimeGraphKeyBuffer buffer_{8, 0};
};

TEST(TimeGraphKeyBufferLookup, EmptyReturnsFalse) {
  TimeGraphKeyBuffer buffer(8, 0);
  double timestamp = -1.0;
  gtsam::Key key = 999;

  EXPECT_FALSE(buffer.getClosestKeyAndTimestamp(timestamp, key, "test", 1.0, 10.0));
  EXPECT_DOUBLE_EQ(timestamp, -1.0);
  EXPECT_EQ(key, 999);
}

TEST_F(TimeGraphKeyBufferLookupTest, BeforeOldestSelectsOldest) {
  double timestamp = 0.0;
  gtsam::Key key = 0;

  EXPECT_TRUE(buffer_.getClosestKeyAndTimestamp(timestamp, key, "test", 1.0, 9.0));
  EXPECT_DOUBLE_EQ(timestamp, 10.0);
  EXPECT_EQ(key, 100);
}

TEST_F(TimeGraphKeyBufferLookupTest, ExactNewestSelectsNewest) {
  double timestamp = 0.0;
  gtsam::Key key = 0;

  EXPECT_TRUE(buffer_.getClosestKeyAndTimestamp(timestamp, key, "test", 0.0, 20.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  EXPECT_EQ(key, 200);
}

TEST_F(TimeGraphKeyBufferLookupTest, MidpointPreservesNewerTieBreak) {
  double timestamp = 0.0;
  gtsam::Key key = 0;

  EXPECT_TRUE(buffer_.getClosestKeyAndTimestamp(timestamp, key, "test", 5.0, 15.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  EXPECT_EQ(key, 200);
}

TEST_F(TimeGraphKeyBufferLookupTest, AfterNewestSelectsNewest) {
  double timestamp = 0.0;
  gtsam::Key key = 0;

  EXPECT_TRUE(buffer_.getClosestKeyAndTimestamp(timestamp, key, "test", 1.0, 21.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  EXPECT_EQ(key, 200);
}

TEST(TimeGraphKeyBufferLookup, PruningAndBoundaryLookupUseOldestRetainedKey) {
  TimeGraphKeyBuffer buffer(2, 0);
  buffer.addToBuffer(10.0, 100);
  buffer.addToBuffer(20.0, 200);
  buffer.addToBuffer(30.0, 300);
  double timestamp = 0.0;
  gtsam::Key key = 0;

  EXPECT_TRUE(buffer.getClosestKeyAndTimestamp(timestamp, key, "test", 11.0, 9.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  EXPECT_EQ(key, 200);
}

std::shared_ptr<GraphConfig> makeImuBufferConfig(int bufferLength = 8) {
  auto config = std::make_shared<GraphConfig>();
  config->imuBufferLength_ = bufferLength;
  config->imuRate_ = 100.0;
  config->useImuSignalLowPassFilter_ = false;
  config->verboseLevel_ = 0;
  return config;
}

void addImuSample(ImuBuffer& buffer, double timestamp) {
  buffer.addToImuBuffer(timestamp, Eigen::Vector3d(timestamp, 2.0 * timestamp, 3.0 * timestamp),
                       Eigen::Vector3d(-timestamp, -2.0 * timestamp, -3.0 * timestamp));
}

void expectImuSample(const ImuMeasurement& measurement, double timestamp) {
  EXPECT_DOUBLE_EQ(measurement.timestamp, timestamp);
  EXPECT_TRUE(measurement.acceleration.isApprox(Eigen::Vector3d(timestamp, 2.0 * timestamp, 3.0 * timestamp)));
  EXPECT_TRUE(measurement.angularVelocity.isApprox(Eigen::Vector3d(-timestamp, -2.0 * timestamp, -3.0 * timestamp)));
}

class ImuBufferLookupTest : public ::testing::Test {
 protected:
  ImuBufferLookupTest() : config_(makeImuBufferConfig()), buffer_(config_) {}

  void SetUp() override {
    addImuSample(buffer_, 10.0);
    addImuSample(buffer_, 20.0);
  }

  std::shared_ptr<GraphConfig> config_;
  ImuBuffer buffer_;
};

TEST(ImuBufferLookup, EmptyReturnsFalse) {
  ImuBuffer buffer(makeImuBufferConfig());
  double timestamp = -1.0;
  ImuMeasurement measurement;
  measurement.timestamp = -1.0;

  EXPECT_FALSE(buffer.getClosestImuMeasurement(timestamp, measurement, 1.0, 10.0));
  EXPECT_DOUBLE_EQ(timestamp, -1.0);
  EXPECT_DOUBLE_EQ(measurement.timestamp, -1.0);
}

TEST_F(ImuBufferLookupTest, BeforeOldestSelectsOldest) {
  double timestamp = 0.0;
  ImuMeasurement measurement;

  EXPECT_TRUE(buffer_.getClosestImuMeasurement(timestamp, measurement, 1.0, 9.0));
  EXPECT_DOUBLE_EQ(timestamp, 10.0);
  expectImuSample(measurement, 10.0);
}

TEST_F(ImuBufferLookupTest, ExactNewestSelectsNewest) {
  double timestamp = 0.0;
  ImuMeasurement measurement;

  EXPECT_TRUE(buffer_.getClosestImuMeasurement(timestamp, measurement, 0.0, 20.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  expectImuSample(measurement, 20.0);
}

TEST_F(ImuBufferLookupTest, MidpointPreservesNewerTieBreak) {
  double timestamp = 0.0;
  ImuMeasurement measurement;

  EXPECT_TRUE(buffer_.getClosestImuMeasurement(timestamp, measurement, 5.0, 15.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  expectImuSample(measurement, 20.0);
}

TEST_F(ImuBufferLookupTest, AfterNewestSelectsNewest) {
  double timestamp = 0.0;
  ImuMeasurement measurement;

  EXPECT_TRUE(buffer_.getClosestImuMeasurement(timestamp, measurement, 1.0, 21.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  expectImuSample(measurement, 20.0);
}

TEST(ImuBufferLookup, PruningAndBoundaryLookupUseOldestRetainedSample) {
  ImuBuffer buffer(makeImuBufferConfig(2));
  addImuSample(buffer, 10.0);
  addImuSample(buffer, 20.0);
  addImuSample(buffer, 30.0);
  double timestamp = 0.0;
  ImuMeasurement measurement;

  EXPECT_TRUE(buffer.getClosestImuMeasurement(timestamp, measurement, 11.0, 9.0));
  EXPECT_DOUBLE_EQ(timestamp, 20.0);
  expectImuSample(measurement, 20.0);
}

}  // namespace
}  // namespace graph_msf
