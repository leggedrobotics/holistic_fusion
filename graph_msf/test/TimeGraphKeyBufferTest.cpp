#include <gtest/gtest.h>

#include "graph_msf/core/TimeGraphKeyBuffer.h"

namespace graph_msf {
namespace {

TEST(TimeGraphKeyBufferTest, ReturnsSafeBoundaryKeys) {
  TimeGraphKeyBuffer buffer(3, 0);
  buffer.addToBuffer(1.0, 10);
  buffer.addToBuffer(2.0, 20);

  double graphTime = 0.0;
  gtsam::Key key = 0;

  EXPECT_FALSE(buffer.getClosestKeyAndTimestamp(graphTime, key, "before", 0.1, 0.5));
  EXPECT_DOUBLE_EQ(graphTime, 1.0);
  EXPECT_EQ(key, 10);

  EXPECT_TRUE(buffer.getClosestKeyAndTimestamp(graphTime, key, "near-latest", 0.1, 1.95));
  EXPECT_DOUBLE_EQ(graphTime, 2.0);
  EXPECT_EQ(key, 20);

  EXPECT_FALSE(buffer.getClosestKeyAndTimestamp(graphTime, key, "after", 0.1, 2.5));
  EXPECT_DOUBLE_EQ(graphTime, 2.0);
  EXPECT_EQ(key, 20);
}

TEST(TimeGraphKeyBufferTest, EvictionAndReplacementKeepMapsConsistent) {
  TimeGraphKeyBuffer buffer(2, 0);
  buffer.addToBuffer(1.0, 30);
  buffer.addToBuffer(2.0, 10);
  buffer.addToBuffer(3.0, 20);

  auto timeToKey = buffer.getTimeToKeyBuffer();
  auto keyToTime = buffer.getKeyToTimeBuffer();
  EXPECT_EQ(timeToKey, (TimeToKeyMap{{2.0, 10}, {3.0, 20}}));
  EXPECT_EQ(keyToTime, (KeyToTimeMap{{10, 2.0}, {20, 3.0}}));

  buffer.addToBuffer(4.0, 20);
  buffer.addToBuffer(2.0, 99);

  timeToKey = buffer.getTimeToKeyBuffer();
  keyToTime = buffer.getKeyToTimeBuffer();
  EXPECT_EQ(timeToKey, (TimeToKeyMap{{2.0, 99}, {4.0, 20}}));
  EXPECT_EQ(keyToTime, (KeyToTimeMap{{20, 4.0}, {99, 2.0}}));
  EXPECT_DOUBLE_EQ(buffer.getLatestTimestampInBuffer(), 4.0);
}

TEST(TimeGraphKeyBufferTest, RejectsNonPositiveCapacity) {
  EXPECT_THROW(TimeGraphKeyBuffer(0, 0), std::invalid_argument);
}

}  // namespace
}  // namespace graph_msf
