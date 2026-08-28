#include "PX4CtrlFSM.h"
#include <dynamic_reconfigure/server.h>
#include <mavros_msgs/MessageInterval.h>
#include <px4ctrl/PIDConfig.h>
#include <ros/ros.h>
#include <signal.h>

void mySigintHandler(int sig) {
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

void pidReconfigureCallback(px4ctrl::PIDConfig &config, uint32_t level, Parameter_t *param) {
    param->gain.Kp0 = config.Kp0;
    param->gain.Kp1 = config.Kp1;
    param->gain.Kp2 = config.Kp2;

    param->gain.Ki0 = config.Ki0;
    param->gain.Ki1 = config.Ki1;
    param->gain.Ki2 = config.Ki2;

    param->gain.Kd0 = config.Kd0;
    param->gain.Kd1 = config.Kd1;
    param->gain.Kd2 = config.Kd2;

    ROS_INFO(
        "[PX4Ctrl][PID] "
        "Kp=[%.3f %.3f %.3f] "
        "Ki=[%.3f %.3f %.3f] "
        "Kd=[%.3f %.3f %.3f]",
        config.Kp0, config.Kp1, config.Kp2, config.Ki0, config.Ki1, config.Ki2, config.Kd0,
        config.Kd1, config.Kd2);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    /***************************************************
     * Dynamic PID reconfigure
     ***************************************************/
    dynamic_reconfigure::Server<px4ctrl::PIDConfig> pid_server(nh);

    px4ctrl::PIDConfig pid_config;

    /* Use YAML values as initial values */
    pid_config.Kp0 = param.gain.Kp0;
    pid_config.Kp1 = param.gain.Kp1;
    pid_config.Kp2 = param.gain.Kp2;

    pid_config.Ki0 = param.gain.Ki0;
    pid_config.Ki1 = param.gain.Ki1;
    pid_config.Ki2 = param.gain.Ki2;

    pid_config.Kd0 = param.gain.Kd0;
    pid_config.Kd1 = param.gain.Kd1;
    pid_config.Kd2 = param.gain.Kd2;

    /*
     * Very important:
     * initialize dynamic_reconfigure using the YAML values.
     */
    pid_server.updateConfig(pid_config);

    dynamic_reconfigure::Server<px4ctrl::PIDConfig>::CallbackType pid_callback;

    pid_callback = boost::bind(&pidReconfigureCallback, _1, _2, &param);

    pid_server.setCallback(pid_callback);

    /***************************************************
     * Controller
     ***************************************************/
    LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        param.mavros_ns + "/state", 10, boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>(
        param.mavros_ns + "/extended_state", 10,
        boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "odom", 100, boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1), ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "cmd", 100, boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1), ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        param.mavros_ns + "/imu/data",  // Note: do NOT change it to /mavros/imu/data_raw !!!
        100, boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1), ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub;
    if (!param.takeoff_land
             .no_RC)  // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>(
            param.mavros_ns + "/rc/in", 10, boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub;
    if (param.thr_map.use_battery_feedback) {
        bat_sub = nh.subscribe<sensor_msgs::BatteryState>(
            param.mavros_ns + "/battery", 100,
            boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1), ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay());
    }

    ros::Subscriber takeoff_land_sub = nh.subscribe<quadrotor_msgs::TakeoffLand>(
        "takeoff_land", 100, boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    fsm.ctrl_FCU_pub =
        nh.advertise<mavros_msgs::AttitudeTarget>(param.mavros_ns + "/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10);  // debug
    if (param.tuning_debug.enable) {
        fsm.tune_debug_pub =
            nh.advertise<quadrotor_msgs::Px4ctrlTuneDebug>(param.tuning_debug.topic, 10);
        ROS_INFO_STREAM("[px4ctrl] PID tuning debug enabled: " << param.tuning_debug.topic);
    } else {
        ROS_INFO("[px4ctrl] PID tuning debug disabled.");
    }

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>(param.mavros_ns + "/set_mode");
    fsm.arming_client_srv =
        nh.serviceClient<mavros_msgs::CommandBool>(param.mavros_ns + "/cmd/arming");
    fsm.reboot_FCU_srv =
        nh.serviceClient<mavros_msgs::CommandLong>(param.mavros_ns + "/cmd/command");

    fsm.set_bat_freq =
        nh.serviceClient<mavros_msgs::MessageInterval>(param.mavros_ns + "/set_message_interval");
    mavros_msgs::MessageInterval srv;
    if (param.thr_map.use_battery_feedback) {
        srv.request.message_id   = param.mavros_battery_id;    // /mavros/battery ID
        srv.request.message_rate = param.mavros_bat_msg_freq;  // 10Hz
        if (fsm.set_bat_freq.call(srv)) {
            ROS_INFO(
                "set bat message frequent %fHz result: %d", srv.request.message_rate,
                srv.response.success);
        } else {
            ROS_ERROR("Failed to call %s/set_message_interval", param.mavros_ns.c_str());
        }
    } else {
        ROS_INFO("[px4ctrl] Battery feedback disabled; using hover_percentage for initialization.");
    }

    srv.request.message_id   = param.mavros_attitude_id;        // ATTITUDE
    srv.request.message_rate = param.mavros_attitude_msg_freq;  // 200Hz
    if (fsm.set_bat_freq.call(srv)) {
        ROS_INFO(
            "set attitude message frequent %fHz result: %d", srv.request.message_rate,
            srv.response.success);
    } else {
        ROS_ERROR("Failed to call %s/set_message_interval", param.mavros_ns.c_str());
    }

    srv.request.message_id   = param.mavros_attitude_quaternion_id;        // ATTITUDE_QUATERNION
    srv.request.message_rate = param.mavros_attitude_quaternion_msg_freq;  // 250Hz
    if (fsm.set_bat_freq.call(srv)) {
        ROS_INFO(
            "set attitude quaternion message frequent %fHz result: %d", srv.request.message_rate,
            srv.response.success);
    } else {
        ROS_ERROR("Failed to call %s/set_message_interval", param.mavros_ns.c_str());
    }

    srv.request.message_id   = param.mavros_highres_imu_id;        // HIGHRES_IMU
    srv.request.message_rate = param.mavros_highres_imu_msg_freq;  // 1000Hz
    if (fsm.set_bat_freq.call(srv)) {
        ROS_INFO(
            "set highres imu message frequent %fHz result: %d", srv.request.message_rate,
            srv.response.success);
    } else {
        ROS_ERROR("Failed to call %s/set_message_interval", param.mavros_ns.c_str());
    }

    // add by bk
    ros::Subscriber emergency_sub = nh.subscribe<std_msgs::Bool>(
        "/planning/Emergency_hover", 1, boost::bind(&PX4CtrlFSM::emergency_callback, &fsm, _1));

    ros::Subscriber search_hover_sub = nh.subscribe<std_msgs::Bool>(
        "/Search_plan/search_hover", 1, boost::bind(&PX4CtrlFSM::search_hover_callback, &fsm, _1));

    ros::Subscriber kf_fusion_fail = nh.subscribe<std_msgs::Bool>(
        "/vins_fusion/vins_fail", 1, boost::bind(&PX4CtrlFSM::kf_fail_callback, &fsm, _1));

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC) {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    } else {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok()) {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now())) {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5) ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        fsm.process();  // We DO NOT rely on feedback as trigger, since there is no significant
                        // performance difference through our test.
    }

    return 0;
}
