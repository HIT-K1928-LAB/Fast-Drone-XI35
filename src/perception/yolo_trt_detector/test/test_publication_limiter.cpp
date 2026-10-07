#include <gtest/gtest.h>

#include "yolo_trt_detector/publication_limiter.h"

TEST(PublicationLimiter, EnforcesTenHertzWithDeterministicTimestamps) {
  yolo_trt_detector::PublicationLimiter limiter(10.0);
  EXPECT_TRUE(limiter.shouldPublish(0.0));
  EXPECT_FALSE(limiter.shouldPublish(0.050));
  EXPECT_FALSE(limiter.shouldPublish(0.099));
  EXPECT_TRUE(limiter.shouldPublish(0.100));
  EXPECT_FALSE(limiter.shouldPublish(0.199));
  EXPECT_TRUE(limiter.shouldPublish(0.200));
}

TEST(PublicationLimiter, RecoversFromClockReset) {
  yolo_trt_detector::PublicationLimiter limiter(10.0);
  EXPECT_TRUE(limiter.shouldPublish(5.0));
  EXPECT_TRUE(limiter.shouldPublish(1.0));
  EXPECT_FALSE(limiter.shouldPublish(1.05));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
