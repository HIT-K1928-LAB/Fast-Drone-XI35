#include <gtest/gtest.h>

#include <yolo_trt_detector/standoff_goal.h>

namespace {

TEST(StandoffGoal, PlacesGoalBetweenVehicleAndTarget) {
    tf2::Vector3 goal;
    ASSERT_TRUE(yolo_trt_detector::calculateStandoffGoal(
        tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(10.0, 0.0, 0.0), 2.0, goal));
    EXPECT_NEAR(goal.x(), 8.0, 1e-9);
    EXPECT_NEAR(goal.y(), 0.0, 1e-9);
    EXPECT_NEAR(goal.z(), 0.0, 1e-9);
}

TEST(StandoffGoal, UsesFullThreeDimensionalDistance) {
    const tf2::Vector3 vehicle(1.0, 2.0, 3.0);
    const tf2::Vector3 target(4.0, 6.0, 15.0);
    tf2::Vector3 goal;
    ASSERT_TRUE(
        yolo_trt_detector::calculateStandoffGoal(vehicle, target, 2.5, goal));
    EXPECT_NEAR((target - goal).length(), 2.5, 1e-9);
    EXPECT_NEAR((goal - vehicle).cross(target - vehicle).length(), 0.0, 1e-9);
}

TEST(StandoffGoal, HoldsVehiclePositionWhenTargetIsTooClose) {
    const tf2::Vector3 vehicle(1.0, 2.0, 3.0);
    tf2::Vector3 goal;
    ASSERT_TRUE(yolo_trt_detector::calculateStandoffGoal(
        vehicle, tf2::Vector3(1.5, 2.0, 3.0), 2.0, goal));
    EXPECT_NEAR((goal - vehicle).length(), 0.0, 1e-9);
}

TEST(StandoffGoal, RejectsInvalidDistance) {
    tf2::Vector3 goal;
    EXPECT_FALSE(yolo_trt_detector::calculateStandoffGoal(
        tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0), -1.0, goal));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
