#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_)
    : param(param_),
      controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
    state = MANUAL_CTRL;
    hover_pose.setZero();
    imu_acc_lpf.setZero();
    flag_init_imu_acc_lpf = false;
}

/*
        Finite State Machine

              system start
                    |
                    v
              MANUAL_CTRL
              /    |    ^
             /     |    |
     TAKEOFF     HOVER  | exit offboard / unsafe
           /       |    |
          v        v    |
    AUTO_TAKEOFF -> AUTO_HOVER <-> CMD_CTRL
          ^          |   ^
          |          |   |
          +----------+   |
       ground TAKEOFF    |
                     \   |
                      v  |
                    AUTO_LAND

        Notes:
        - AUTO_HOVER can still be an on-ground offboard state before takeoff.
        - AUTO_HOVER accepts TAKEOFF only when the vehicle is considered on ground.
        - AUTO_TAKEOFF reuses an existing OFFBOARD session when entered from AUTO_HOVER.
*/

// ---- state handlers ----
void PX4CtrlFSM::handleManualCtrl(const ros::Time &now_time, Desired_State_t &des) {
    if (kf_fusion_fail) {
        ROS_ERROR("[px4ctrl] kf fusion is failed, please take land manually!");
        return;
    }
    if (tryEnterAutoTakeoff(now_time)) return;
    if (tryEnterAutoHover(now_time)) return;
    if (tryRebootFcu()) return;
}

void PX4CtrlFSM::handleAutoHover(const ros::Time &now_time, Desired_State_t &des) {
    if (tryFallbackToManual(now_time)) return;
    if (canAutoTakeoffFromHover()) {
        if (tryEnterAutoTakeoff(now_time)) return;
    }
    if (tryEnterAutoLand(now_time)) return;
    if (tryEnterCmdCtrl(now_time, des)) return;

    set_hov_with_rc();
    des = get_hover_des();

    const bool command_switch_triggered = commandSwitchTriggered();
    const bool delayed_trigger_due      = takeoff_land_ctx.delayed_trigger.first &&
                                     now_time > takeoff_land_ctx.delayed_trigger.second;
    if (command_switch_triggered || delayed_trigger_due) {
        if (canUsePlannerCommand(now_time)) {
            takeoff_land_ctx.delayed_trigger.first = false;
            publish_trigger(odom_data.msg);
            ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow planner command.\033[32m");
        } else if (command_switch_triggered) {
            ROS_WARN("[px4ctrl] Reject TRIGGER. Drone is not confirmed airborne.");
        } else {
            ROS_WARN_THROTTLE(
                1.0, "[px4ctrl] Hold delayed TRIGGER until drone is confirmed airborne.");
        }
    }
}

void PX4CtrlFSM::handleCmdCtrl(const ros::Time &now_time, Desired_State_t &des) {
    if (tryFallbackToManual(now_time)) return;
    if (tryFallbackCmdToHover(now_time, des)) return;

    des = get_cmd_des();

    if (landRequested()) {
        ROS_ERROR(
            "[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. "
            "Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER "
            "first.",
            param.msg_timeout.cmd);
    }
}

void PX4CtrlFSM::handleAutoTakeoff(const ros::Time &now_time, Desired_State_t &des) {
    if (tryFallbackToManual(now_time)) return;

    if ((now_time - takeoff_land_ctx.command_time).toSec() <
        TakeoffLandContext::MOTORS_SPEEDUP_TIME) {
        des = get_rotor_speed_up_des(now_time);
        return;
    }

    if (odom_data.p(2) >= takeoff_land_ctx.start_pose(2) + param.takeoff_land.height) {
        state = AUTO_HOVER;
        set_hov_with_odom();

        ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");
        ROS_INFO("odom_data.p(2): %f", odom_data.p(2));
        ROS_INFO("takeoff_land_ctx.start_pose(2): %f", takeoff_land_ctx.start_pose(2));
        ROS_INFO("takeoff_height: %f", param.takeoff_land.height);

        takeoff_land_ctx.delayed_trigger.first = true;
        takeoff_land_ctx.delayed_trigger.second =
            now_time + ros::Duration(TakeoffLandContext::DELAY_TRIGGER_TIME);
        return;
    }

    des = get_takeoff_land_des(param.takeoff_land.speed);
}

void PX4CtrlFSM::handleAutoLand(
    const ros::Time &now_time, Desired_State_t &des, bool &rotor_low_speed_during_land) {
    if (tryFallbackToManual(now_time)) return;

    if (!commandSwitchEnabled()) {
        state = AUTO_HOVER;
        set_hov_with_odom();
        des = get_hover_des();

        ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
        return;
    }

    if (!get_landed()) {
        des = get_takeoff_land_des(-param.takeoff_land.speed);
        return;
    }

    rotor_low_speed_during_land = true;

    static bool print_once_flag = true;
    if (print_once_flag) {
        ROS_INFO("\033[32m[px4ctrl] Wait for about 10s to let the drone disarm.\033[32m");
        print_once_flag = false;
    }

    if (extended_state_data.current_extended_state.landed_state ==
        mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
        static double last_trial_time = 0.0;
        if (now_time.toSec() - last_trial_time > 1.0) {
            if (toggle_arm_disarm(false)) {
                print_once_flag = true;
                state           = MANUAL_CTRL;
                toggle_offboard_mode(false);

                ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
            }

            last_trial_time = now_time.toSec();
        }
    }
}

// ---- event predicates ----
bool PX4CtrlFSM::takeoffRequested() const {
    return param.takeoff_land.enable && takeoff_land_data.triggered &&
           takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF;
}

bool PX4CtrlFSM::landRequested() const {
    return takeoff_land_data.triggered &&
           takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND;
}

bool PX4CtrlFSM::hoverSwitchTriggered() const { return rc_data.enter_hover_mode; }

bool PX4CtrlFSM::commandSwitchEnabled() const { return rc_data.is_command_mode; }

bool PX4CtrlFSM::commandSwitchTriggered() const { return rc_data.enter_command_mode; }

// ---- guards ----
bool PX4CtrlFSM::canEnterAutoHover(const ros::Time &now_time) const {
    if (!odom_is_received(now_time)) {
        ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
        return false;
    }

    if (cmd_is_received(now_time)) {
        ROS_ERROR(
            "[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into "
            "AUTO_HOVER.");
        return false;
    }

    if (odom_data.v.norm() > 3.0) {
        ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s.", odom_data.v.norm());
        return false;
    }

    return true;
}

bool PX4CtrlFSM::canEnterAutoTakeoff(const ros::Time &now_time) const {
    if (!odom_is_received(now_time)) {
        ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
        return false;
    }

    if (cmd_is_received(now_time)) {
        ROS_ERROR(
            "[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into "
            "AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
        return false;
    }

    if (odom_data.v.norm() > 0.1) {
        ROS_ERROR(
            "[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!",
            odom_data.v.norm());
        return false;
    }

    if (!get_landed() && !isOnGroundForTakeoff()) {
        ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Drone is not on ground.");
        return false;
    }

    if (!bat_is_received(now_time)) {
        ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No battery data.");
        return false;
    }

    if (rc_is_received(now_time)) {
        if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered()) {
            ROS_ERROR(
                "[px4ctrl] Reject AUTO_TAKEOFF. Keep RC switches at auto hover and command "
                "control, and sticks centered.");
            return false;
        }
    }

    return true;
}

bool PX4CtrlFSM::canEnterCmdCtrl(const ros::Time &now_time) const {
    return commandSwitchEnabled() && cmd_is_received(now_time) && !emergency_hover &&
           !search_hover && state_data.current_state.mode == "OFFBOARD" &&
           canUsePlannerCommand(now_time);
}

bool PX4CtrlFSM::canUsePlannerCommand(const ros::Time &now_time) const {
    if (!odom_is_received(now_time)) {
        ROS_WARN_THROTTLE(1.0, "[px4ctrl] Reject planner command. No recent odom.");
        return false;
    }

    if (!state_data.current_state.armed) {
        ROS_WARN_THROTTLE(1.0, "[px4ctrl] Reject planner command. Drone is not armed.");
        return false;
    }

    if (isOnGroundForTakeoff()) {
        ROS_WARN_THROTTLE(
            1.0, "[px4ctrl] Reject planner command. Drone is still reported on ground.");
        return false;
    }

    if (get_landed()) {
        ROS_WARN_THROTTLE(
            1.0, "[px4ctrl] Reject planner command. Land detector still reports landed.");
        return false;
    }

    return true;
}

bool PX4CtrlFSM::isOnGroundForTakeoff() const {
    return extended_state_data.current_extended_state.landed_state ==
           mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
}

bool PX4CtrlFSM::canAutoTakeoffFromHover() const {
    return state == AUTO_HOVER &&
           (isOnGroundForTakeoff() || !state_data.current_state.armed || get_landed());
}

// ---- transition attempts ----
bool PX4CtrlFSM::tryEnterAutoTakeoff(const ros::Time &now_time) {
    if (!takeoffRequested()) return false;

    if (!canEnterAutoTakeoff(now_time)) return true;

    enterAutoTakeoff(now_time);
    return true;
}

bool PX4CtrlFSM::tryEnterAutoHover(const ros::Time &now_time) {
    if (!hoverSwitchTriggered()) return false;

    if (!canEnterAutoHover(now_time)) {
        return true;
    }

    enterAutoHover();
    return true;
}

bool PX4CtrlFSM::tryEnterCmdCtrl(const ros::Time &now_time, Desired_State_t &des) {
    if (!canEnterCmdCtrl(now_time)) return false;

    enterCmdCtrl(des);
    return true;
}

bool PX4CtrlFSM::tryEnterAutoLand(const ros::Time &now_time) {
    if (!(landRequested() || emergency_hover)) return false;

    enterAutoLand();
    if (emergency_hover) {
        ROS_WARN("Switch to AUTO_LAND due to Emergency!");
    }
    return true;
}

bool PX4CtrlFSM::tryFallbackToManual(const ros::Time &now_time) {
    const bool rc_ok = param.takeoff_land.no_RC || rc_is_received(now_time);
    if (rc_ok && rc_data.is_hover_mode && odom_is_received(now_time) && !kf_fusion_fail) {
        return false;
    }

    const State_t old_state = state;
    if (!enterManualFromOffboard()) {
        state = old_state;
        ROS_ERROR_THROTTLE(1.0, "[px4ctrl] Failed to exit OFFBOARD, keep current FSM state!");
        return true;
    }

    if (old_state == AUTO_HOVER) {
        ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
    } else if (old_state == CMD_CTRL) {
        ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
    } else if (old_state == AUTO_LAND) {
        ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
    } else if (old_state == AUTO_TAKEOFF) {
        ROS_WARN("[px4ctrl] AUTO_TAKEOFF --> MANUAL_CTRL(L1)");
    }

    return true;
}

bool PX4CtrlFSM::tryFallbackCmdToHover(const ros::Time &now_time, Desired_State_t &des) {
    if (commandSwitchEnabled() && cmd_is_received(now_time) && !emergency_hover && !search_hover) {
        return false;
    }

    state = AUTO_HOVER;
    set_hov_with_odom();
    des = get_hover_des();

    ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
    if (emergency_hover) ROS_WARN("Switch to AUTO_HOVER due to Emergency!");
    if (search_hover) ROS_WARN("Switch from CMD to Hover due to search_hover");

    return true;
}

bool PX4CtrlFSM::tryRebootFcu() {
    if (!rc_data.toggle_reboot) return false;

    if (state_data.current_state.armed) {
        ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
        return true;
    }

    reboot_FCU();
    return true;
}

// ---- transition actions ----
bool PX4CtrlFSM::enterManualFromOffboard() {
    if (state_data.current_state.mode == "OFFBOARD") {
        if (!toggle_offboard_mode(false)) {
            return false;
        }
    }

    state = MANUAL_CTRL;
    return true;
}

bool PX4CtrlFSM::enterAutoHover() {
    controller.resetThrustMapping(bat_data);
    set_hov_with_odom();

    if (!toggle_offboard_mode(true)) {
        state = MANUAL_CTRL;
        return false;
    }

    state = AUTO_HOVER;
    ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
    return true;
}

bool PX4CtrlFSM::enterAutoTakeoff(const ros::Time &now_time) {
    controller.resetThrustMapping(bat_data);
    set_start_pose_for_takeoff_land(odom_data);

    if (state_data.current_state.mode != "OFFBOARD") {
        if (!toggle_offboard_mode(true)) {
            state = MANUAL_CTRL;
            return false;
        }
    }

    state = AUTO_TAKEOFF;

    ros::Duration(0.1).sleep();
    ros::spinOnce();

    if (param.takeoff_land.enable_auto_arm && !toggle_arm_disarm(true)) {
        enterManualFromOffboard();
        return false;
    }

    takeoff_land_ctx.command_time = now_time;
    ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
    return true;
}

void PX4CtrlFSM::enterCmdCtrl(Desired_State_t &des) {
    state = CMD_CTRL;
    des   = get_cmd_des();

    ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
}

void PX4CtrlFSM::enterAutoLand() {
    state = AUTO_LAND;
    set_start_pose_for_takeoff_land(odom_data);

    ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
}

void PX4CtrlFSM::process() {
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data);
    bool rotor_low_speed_during_land = false;

    // STEP1: state machine runs
    switch (state) {
        case MANUAL_CTRL:
            handleManualCtrl(now_time, des);
            break;
        case AUTO_HOVER:
            handleAutoHover(now_time, des);
            break;
        case CMD_CTRL:
            handleCmdCtrl(now_time, des);
            break;
        case AUTO_TAKEOFF:
            handleAutoTakeoff(now_time, des);
            break;
        case AUTO_LAND:
            handleAutoLand(now_time, des, rotor_low_speed_during_land);
            break;
        default:
            break;
    }

    // STEP1.5: low pass filter for imu acc data
    if (state == AUTO_TAKEOFF || state == AUTO_HOVER || state == CMD_CTRL) {
        LPF_imu_a(imu_data.a);
    }

    // STEP2: estimate thrust model
    if (state == AUTO_TAKEOFF) {
        ros::Time now = ros::Time::now();
        double delta_t =
            (now - takeoff_land_ctx.command_time).toSec() - TakeoffLandContext::MOTORS_SPEEDUP_TIME;
        if (delta_t > 0.2) controller.estimateThrustModel(imu_acc_lpf, param, bat_data);
    }

    if (state == AUTO_HOVER || state == CMD_CTRL) {
        // controller.estimateThrustModel(imu_data.a, bat_data.volt, param);
        // controller.estimateThrustModel(imu_data.a, param);
        // controller.estimateThrustModelUsingVelFB(odom_data.v, param);
        controller.estimateThrustModel(imu_acc_lpf, param, bat_data);
        // controller.estimateThrustModel(imu_acc_lpf, param, bat_data);
    }

    // STEP3: solve and update new control commands
    if (rotor_low_speed_during_land)  // used at the start of auto land
    {
        motors_idling(imu_data, u);
    } else {
        debug_msg              = controller.calculateControl(des, odom_data, imu_data, u);
        debug_msg.header.stamp = now_time;
        debug_pub.publish(debug_msg);
    }

    // STEP4: publish control commands to mavros
    if (param.use_bodyrate_ctrl) {
        publish_bodyrate_ctrl(u, now_time);
    } else {
        publish_attitude_ctrl(u, now_time);
    }

    // STEP5: Detect if the drone has landed
    land_detector(state, des, odom_data);
    // cout << takeoff_land_ctx.landed << " ";
    // fflush(stdout);

    // STEP6: Clear flags beyound their lifetime
    rc_data.enter_hover_mode    = false;
    rc_data.enter_command_mode  = false;
    rc_data.toggle_reboot       = false;
    takeoff_land_data.triggered = false;
}

void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u) {
    u.q         = imu.q;
    u.bodyrates = Eigen::Vector3d::Zero();
    u.thrust    = 0.04;
}

void PX4CtrlFSM::land_detector(
    const State_t state, const Desired_State_t &des, const Odom_Data_t &odom) {
    static State_t last_state = State_t::MANUAL_CTRL;
    if (last_state == State_t::MANUAL_CTRL &&
        (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF)) {
        takeoff_land_ctx.landed = false;  // Always holds
    }
    last_state = state;

    if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed) {
        takeoff_land_ctx.landed = true;
        return;  // No need of other decisions
    }

    // land_detector parameters
    constexpr double POSITION_DEVIATION_C =
        -0.5;  // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
    constexpr double VELOCITY_THR_C = 0.1;  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    constexpr double TIME_KEEP_C = 3.0;  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

    static ros::Time time_C12_reached;  // time_Constraints12_reached
    static bool is_last_C12_satisfy;
    if (takeoff_land_ctx.landed) {
        time_C12_reached    = ros::Time::now();
        is_last_C12_satisfy = false;
    } else {
        bool C12_satisfy =
            (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
        if (C12_satisfy && !is_last_C12_satisfy) {
            time_C12_reached = ros::Time::now();
        } else if (C12_satisfy && is_last_C12_satisfy) {
            if ((ros::Time::now() - time_C12_reached).toSec() >
                TIME_KEEP_C)  // Constraint 3 reached
            {
                takeoff_land_ctx.landed = true;
            }
        }

        is_last_C12_satisfy = C12_satisfy;
    }
}

Desired_State_t PX4CtrlFSM::get_hover_des() {
    Desired_State_t des;
    des.p        = hover_pose.head<3>();
    des.v        = Eigen::Vector3d::Zero();
    des.a        = Eigen::Vector3d::Zero();
    des.j        = Eigen::Vector3d::Zero();
    des.yaw      = hover_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des() {
    Desired_State_t des;
    des.p        = cmd_data.p;
    des.v        = cmd_data.v;
    des.a        = cmd_data.a;
    des.j        = cmd_data.j;
    des.yaw      = cmd_data.yaw;
    des.yaw_rate = cmd_data.yaw_rate;

    return des;
}

Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now) {
    double delta_t = (now - takeoff_land_ctx.command_time).toSec();
    double des_a_z = exp((delta_t - TakeoffLandContext::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 -
                     7.0;  // Parameters 6.0 and 7.0 are just heuristic values which result in a
                           // saticfactory curve.
    if (des_a_z > 0.1) {
        ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
        des_a_z = 0.0;
    }

    Desired_State_t des;
    des.p        = takeoff_land_ctx.start_pose.head<3>();
    des.v        = Eigen::Vector3d::Zero();
    des.a        = Eigen::Vector3d(0, 0, des_a_z);
    des.j        = Eigen::Vector3d::Zero();
    des.yaw      = takeoff_land_ctx.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed) {
    ros::Time now = ros::Time::now();
    double delta_t =
        (now - takeoff_land_ctx.command_time).toSec() -
        (speed > 0 ? TakeoffLandContext::MOTORS_SPEEDUP_TIME : 0);  // speed > 0 means takeoff
    // takeoff_land.last_set_cmd_time = now;

    // takeoff_land_ctx.start_pose(2) += speed * delta_t;

    Desired_State_t des;
    des.p = takeoff_land_ctx.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
    des.v = Eigen::Vector3d(0, 0, speed);
    if (speed > 0) {
        double des_a_z =
            (delta_t < TakeoffLandContext::TAKEOFF_SPEEDUP_TIME)
                ? (0.01 * 9.81 * sin(M_PI / TakeoffLandContext::TAKEOFF_SPEEDUP_TIME * delta_t))
                : 0;
        des.a = Eigen::Vector3d(0, 0, des_a_z);
    } else {
        des.a = Eigen::Vector3d::Zero();
    }
    des.j        = Eigen::Vector3d::Zero();
    des.yaw      = takeoff_land_ctx.start_pose(3);
    des.yaw_rate = 0.0;

    return des;
}

void PX4CtrlFSM::set_hov_with_odom() {
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3)        = get_yaw_from_quaternion(odom_data.q);

    last_set_hover_pose_time = ros::Time::now();
}

void PX4CtrlFSM::set_hov_with_rc() {
    ros::Time now            = ros::Time::now();
    double delta_t           = (now - last_set_hover_pose_time).toSec();
    last_set_hover_pose_time = now;

    hover_pose(0) +=
        rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? -1 : 1);
    hover_pose(1) +=
        rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? -1 : 1);
    hover_pose(2) +=
        rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? -1 : 1);
    hover_pose(3) +=
        rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? -1 : 1);

    if (hover_pose(2) < -0.3) hover_pose(2) = -0.3;

    // if (param.print_dbg)
    // {
    // 	static unsigned int count = 0;
    // 	if (count++ % 100 == 0)
    // 	{
    // 		cout << "hover_pose=" << hover_pose.transpose() << endl;
    // 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << "
    // "
    // << rc_data.ch[3] << endl;
    // 	}
    // }
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom) {
    takeoff_land_ctx.start_pose.head<3>() = odom_data.p;
    takeoff_land_ctx.start_pose(3)        = get_yaw_from_quaternion(odom_data.q);

    takeoff_land_ctx.command_time = ros::Time::now();
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time) const {
    return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time) const {
    return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time) const {
    return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time) const {
    return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time) const {
    return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom() {
    if (odom_data.recv_new_msg) {
        odom_data.recv_new_msg = false;
        return true;
    }

    return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp    = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();

    msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp    = stamp;
    msg.header.frame_id = std::string("FCU");

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    msg.orientation.x = u.q.x();
    msg.orientation.y = u.q.y();
    msg.orientation.z = u.q.z();
    msg.orientation.w = u.q.w();

    msg.thrust = u.thrust;

    ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose            = odom_msg.pose.pose;

    traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off) {
    mavros_msgs::SetMode offb_set_mode;

    if (on_off) {
        state_data.state_before_offboard = state_data.current_state;
        if (state_data.state_before_offboard.mode == "OFFBOARD") {  // Not allowed
            state_data.state_before_offboard.mode = "MANUAL";
        }

        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROS_ERROR("Enter OFFBOARD rejected by PX4!");
            return false;
        }
    } else {
        offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
        if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)) {
            ROS_ERROR("Exit OFFBOARD rejected by PX4!");
            return false;
        }
    }

    return true;

    // if (param.print_dbg)
    // 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success)) {
        if (arm)
            ROS_ERROR("ARM rejected by PX4!");
        else
            ROS_ERROR("DISARM rejected by PX4!");

        return false;
    }

    return true;
}

void PX4CtrlFSM::reboot_FCU() {
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast    = false;
    reboot_srv.request.command      = 246;  // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1       = 1;    // Reboot autopilot
    reboot_srv.request.param2       = 0;    // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;

    reboot_FCU_srv.call(reboot_srv);

    ROS_INFO("Reboot FCU");

    // if (param.print_dbg)
    // 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result,
    // reboot_srv.response.success);
}

void PX4CtrlFSM::LPF_imu_a(Eigen::Vector3d &imu_data_acc) {
    if (flag_init_imu_acc_lpf == false) {
        // LPF：
        // b = 2 * PI * fc * dt, dt = 1 / fs
        // a = b / (1 + b)
        // y[n] = a * x[n] + (1-a) * y[n-1]

        b_lpf = 2 * 3.1415926 * param.thr_map.imu_acc_lpf_freq_cutoff / param.ctrl_freq_max;
        a_lpf = b_lpf / (1 + b_lpf);
        printf(
            "lpf_freq_cutoff: %6.3f, a_lpf: %6.3f\n", param.thr_map.imu_acc_lpf_freq_cutoff, a_lpf);

        imu_acc_lpf           = imu_data_acc;
        flag_init_imu_acc_lpf = true;
    }
    imu_acc_lpf = a_lpf * imu_data_acc + (1 - a_lpf) * imu_acc_lpf;
    // printf("%6.3f,%6.3f\n", imu_acc_lpf(2), imu_data_acc(2));
    // fflush(stdout);
}
