#include <gtest/gtest.h>
#include <mavros_msgs/ExtendedState.h>
#include <px4ctrl/offboard_fsm/config.h>
#include <px4ctrl/offboard_fsm/freshness.h>
#include <px4ctrl/offboard_fsm/fsm_core.h>
#include <stdexcept>

namespace px4ctrl {
namespace {

InputSnapshot healthyInput(const double now) {
    InputSnapshot input;
    input.now                     = now;
    input.enabled                 = true;
    input.state_fresh             = true;
    input.extended_state_fresh    = true;
    input.odometry_fresh          = true;
    input.livo_odometry_fresh     = true;
    input.connected               = true;
    input.armed                   = false;
    input.mode                    = "POSCTL";
    input.landed_state            = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    input.position.z              = 0.0;
    input.livo_position           = input.position;
    input.planner.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    return input;
}

void advance(FsmCore& core, InputSnapshot& input, const double seconds, const double dt = 0.02) {
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        input.now += dt;
        input.events = Events();
        core.step(input, dt);
    }
}

TEST(OffboardFsmFreshness, AcceptsReceiptThatRacesAheadOfSampledNow) {
    EXPECT_TRUE(receiptFresh(10.001, 0.30, 10.000))
        << "A callback completed after the control cycle sampled now; "
           "the data is newest, not stale";
}

TEST(OffboardFsmFreshness, RejectsMissingInvalidAndExpiredReceipts) {
    EXPECT_FALSE(receiptFresh(0.0, 0.30, 10.0));
    EXPECT_FALSE(receiptFresh(9.0, 0.30, 10.0));
    EXPECT_FALSE(receiptFresh(9.9, -0.1, 10.0));
    EXPECT_FALSE(receiptFresh(NAN, 0.30, 10.0));
    EXPECT_TRUE(receiptFresh(9.8, 0.30, 10.0));
}

TEST(OffboardFsmConfig, RejectsNonPositiveRcTimeout) {
    Config config;
    config.timeouts.rc_input = 0.0;
    EXPECT_THROW(config.validate(), std::runtime_error);
}

TEST(OffboardFsmCore, GroundTakeoffHoverExternalTimeoutAndLanding) {
    Config config;
    config.node.prestream_min_time        = 0.2;
    config.takeoff.relative_height        = 0.2;
    config.takeoff.max_velocity           = 0.5;
    config.takeoff.max_acceleration       = 1.0;
    config.takeoff.max_jerk               = 2.0;
    config.takeoff.stable_time            = 0.1;
    config.planner.activation_messages    = 2;
    config.planner.brake_stable_time      = 0.1;
    config.landing.touchdown_confirm_time = 0.1;
    config.landing.disarm_delay           = 0.0;

    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::PRESTREAM, core.state());
    advance(core, input, 0.3);

    input.mode = "OFFBOARD";
    input.now += 0.02;
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::GROUND, core.state());

    input.events.takeoff = true;
    input.now += 0.02;
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::ARMING, core.state());

    input.events = Events();
    input.armed  = true;
    input.now += 0.02;
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::TAKEOFF, core.state());

    input.planner_fresh    = true;
    input.planner_sequence = 5;
    advance(core, input, 0.2);
    EXPECT_EQ(FsmState::TAKEOFF, core.state()) << "Planner commands must be ignored during takeoff";

    input.position.z                  = 0.2;
    input.velocity_world.z            = 0.0;
    bool published_traj_start_trigger = false;
    for (int i = 0; i < 10; ++i) {
        input.now += 0.02;
        input.events                 = Events();
        const FsmOutput output       = core.step(input, 0.02);
        published_traj_start_trigger = published_traj_start_trigger ||
                                       output.publish_traj_start_trigger;
    }
    EXPECT_EQ(FsmState::HOVER, core.state());
    EXPECT_TRUE(published_traj_start_trigger);

    input.now += 0.02;
    EXPECT_FALSE(core.step(input, 0.02).publish_traj_start_trigger)
        << "The trajectory start trigger must be emitted only on the "
           "TAKEOFF-to-HOVER transition";

    input.planner_received_at = input.now + 0.01;
    input.planner_fresh       = true;
    input.planner.position    = input.position;
    input.planner.yaw         = input.yaw;
    input.planner_sequence++;
    input.now += 0.02;
    core.step(input, 0.02);
    input.planner_sequence++;
    input.planner_received_at = input.now;
    input.now += 0.02;
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::EXTERNAL, core.state());

    input.planner_fresh = false;
    input.now += 0.02;
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::BRAKE, core.state());
    input.velocity_world = geometry_msgs::Vector3();
    advance(core, input, 0.2);
    EXPECT_EQ(FsmState::HOVER, core.state());

    input.events.land = true;
    input.now += 0.02;
    core.step(input, 0.02);
    EXPECT_EQ(FsmState::LANDING, core.state());

    input.events       = Events();
    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    advance(core, input, 0.2);
    EXPECT_EQ(FsmState::DISARMING, core.state());
}

TEST(OffboardFsmCore, OdometryLossStopsSetpoints) {
    Config config;
    config.node.prestream_min_time = 0.1;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);
    input.mode = "OFFBOARD";
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::GROUND, core.state());

    input.odometry_fresh = false;
    input.now += 0.02;
    const FsmOutput output = core.step(input, 0.02);
    EXPECT_EQ(FsmState::FAILSAFE, core.state());
    EXPECT_FALSE(output.publish_setpoint);
}

TEST(OffboardFsmCore, LandingSurvivesOdometryLossAndDisarmsOnGround) {
    Config config;
    config.node.prestream_min_time        = 0.1;
    config.landing.touchdown_confirm_time = 0.1;
    config.landing.disarm_delay           = 0.0;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);
    input.mode         = "OFFBOARD";
    input.armed        = true;
    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::HOVER, core.state());

    input.events.land = true;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::LANDING, core.state());

    input.events         = Events();
    input.odometry_fresh = false;
    input.now += 0.02;
    const FsmOutput stale_output = core.step(input, 0.02);
    EXPECT_EQ(FsmState::LANDING, core.state());
    EXPECT_FALSE(stale_output.publish_setpoint);

    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    advance(core, input, 0.2);
    ASSERT_EQ(FsmState::DISARMING, core.state());
    input.now += 0.02;
    const FsmOutput disarm_output = core.step(input, 0.02);
    EXPECT_TRUE(disarm_output.request_disarm);
}

TEST(OffboardFsmCore, LandingSurvivesOffboardExitAndPrestreamsFrozenReference) {
    Config config;
    config.node.prestream_min_time        = 0.1;
    config.landing.touchdown_confirm_time = 0.1;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);
    input.mode         = "OFFBOARD";
    input.armed        = true;
    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::HOVER, core.state());

    input.events.land = true;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::LANDING, core.state());

    input.events = Events();
    input.mode   = "POSCTL";
    input.now += 0.02;
    const FsmOutput prestream_output = core.step(input, 0.02);
    EXPECT_EQ(FsmState::LANDING, core.state());
    EXPECT_TRUE(prestream_output.publish_setpoint);
    EXPECT_FALSE(prestream_output.request_offboard);

    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
    advance(core, input, 0.2);
    EXPECT_EQ(FsmState::DISARMING, core.state());
}

TEST(OffboardFsmCore, LandingKeepsDownwardIntentAtPositionFloor) {
    Config config;
    config.node.prestream_min_time      = 0.1;
    config.landing.max_velocity         = 0.2;
    config.landing.max_acceleration     = 1.0;
    config.landing.max_descent_distance = 0.05;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);

    input.mode         = "OFFBOARD";
    input.armed        = true;
    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::HOVER, core.state());

    input.events.land = true;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::LANDING, core.state());

    input.events = Events();
    FsmOutput output;
    for (int i = 0; i < 150; ++i) {
        input.now += 0.02;
        output = core.step(input, 0.02);
    }

    EXPECT_TRUE(output.publish_setpoint);
    EXPECT_NEAR(-0.05, output.reference.position.z, 1e-6);
    EXPECT_NEAR(-config.landing.max_velocity, output.reference.velocity.z, 1e-6)
        << "PX4 requires a downward trajectory velocity to confirm touchdown";
}

TEST(OffboardFsmCore, PlannerBeforeHoverEntryIsRejected) {
    Config config;
    config.node.prestream_min_time = 0.1;
    FsmCore core(config);
    InputSnapshot input       = healthyInput(1.0);
    input.planner_fresh       = true;
    input.planner_sequence    = 10;
    input.planner_received_at = 0.5;
    core.step(input, 0.02);
    advance(core, input, 0.2);
    input.mode         = "OFFBOARD";
    input.armed        = true;
    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::HOVER, core.state());

    advance(core, input, 0.2);
    EXPECT_EQ(FsmState::HOVER, core.state());
}

TEST(OffboardFsmCore, RejectsTakeoffAboveConfiguredHeightLimit) {
    Config config;
    config.node.prestream_min_time        = 0.1;
    config.limits.max_height_above_origin = 1.0;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);
    input.mode = "OFFBOARD";
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::GROUND, core.state());

    input.events.takeoff        = true;
    input.events.takeoff_height = 1.5;
    input.now += 0.02;
    const FsmOutput output = core.step(input, 0.02);
    EXPECT_EQ(FsmState::GROUND, core.state());
    EXPECT_FALSE(output.request_arm);
    EXPECT_EQ("takeoff request rejected by height limit", core.transitionReason());
}

TEST(OffboardFsmCore, PlannerOutsideGeofenceFallsBackToBrake) {
    Config config;
    config.node.prestream_min_time        = 0.1;
    config.planner.activation_messages    = 1;
    config.limits.max_horizontal_distance = 1.0;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);
    input.mode         = "OFFBOARD";
    input.armed        = true;
    input.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::HOVER, core.state());

    input.planner_fresh       = true;
    input.planner.position    = input.position;
    input.planner_received_at = input.now + 0.01;
    ++input.planner_sequence;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::EXTERNAL, core.state());

    input.planner.position.x  = 1.5;
    input.planner_received_at = input.now;
    ++input.planner_sequence;
    input.now += 0.02;
    const FsmOutput brake_output = core.step(input, 0.02);
    EXPECT_EQ(FsmState::BRAKE, core.state());
    EXPECT_TRUE(brake_output.publish_setpoint)
        << "The setpoint stream must not have a transition-cycle gap";
}

TEST(OffboardFsmCore, AutomaticOffboardRequestRetriesUntilModeChanges) {
    Config config;
    config.node.prestream_min_time             = 0.1;
    config.node.request_offboard_automatically = true;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);

    input.events.takeoff = true;
    input.now += 0.02;
    EXPECT_TRUE(core.step(input, 0.02).request_offboard);
    input.events = Events();
    input.now += 0.02;
    EXPECT_TRUE(core.step(input, 0.02).request_offboard)
        << "A rejected/lost mode service request must be retried";

    input.events.land = true;
    input.now += 0.02;
    EXPECT_FALSE(core.step(input, 0.02).request_offboard)
        << "Landing must cancel a pending automatic takeoff";
}

TEST(OffboardFsmCore, FcuLossCancelsPendingAutomaticTakeoff) {
    Config config;
    config.node.prestream_min_time             = 0.1;
    config.node.request_offboard_automatically = true;
    FsmCore core(config);
    InputSnapshot input = healthyInput(1.0);
    core.step(input, 0.02);
    advance(core, input, 0.2);

    input.events.takeoff = true;
    input.now += 0.02;
    ASSERT_TRUE(core.step(input, 0.02).request_offboard);

    input.events         = Events();
    input.odometry_fresh = false;
    input.now += 0.02;
    core.step(input, 0.02);
    ASSERT_EQ(FsmState::WAIT_FCU, core.state());

    input.odometry_fresh = true;
    input.now += 0.02;
    EXPECT_FALSE(core.step(input, 0.02).request_offboard)
        << "A takeoff requested before FCU feedback loss must not resume automatically";
    EXPECT_EQ(FsmState::PRESTREAM, core.state());
}

TEST(OffboardFsmCore, RcSingleSwitchMapsStateToSafeEvent) {
    const uint8_t on_ground = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;

    EXPECT_TRUE(rcTakeoffLandEvent(FsmState::GROUND, true, on_ground).takeoff);
    EXPECT_FALSE(rcTakeoffLandEvent(FsmState::GROUND, false, on_ground).takeoff);
    EXPECT_TRUE(rcTakeoffLandEvent(FsmState::ARMING, true, on_ground).land);
    EXPECT_TRUE(rcTakeoffLandEvent(FsmState::TAKEOFF, true, on_ground).land);
    EXPECT_TRUE(rcTakeoffLandEvent(FsmState::HOVER, true, on_ground).land);
    EXPECT_TRUE(rcTakeoffLandEvent(FsmState::EXTERNAL, true, on_ground).land);
    EXPECT_TRUE(rcTakeoffLandEvent(FsmState::BRAKE, true, on_ground).land);

    EXPECT_FALSE(rcTakeoffLandEvent(FsmState::PRESTREAM, true, on_ground).takeoff);
    EXPECT_FALSE(rcTakeoffLandEvent(FsmState::LANDING, true, on_ground).land);
    EXPECT_FALSE(rcTakeoffLandEvent(FsmState::DISARMING, true, on_ground).land);
    EXPECT_FALSE(rcTakeoffLandEvent(FsmState::FAILSAFE, true, on_ground).land);
}

}  // namespace
}  // namespace px4ctrl

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
