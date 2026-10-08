#!/usr/bin/env bash

# orin-lidar-01 可跨飞机复用的 PX4 参数写入脚本。
# 脚本会临时启动 MAVROS，在确认飞控已连接且未解锁后写入参数，最后重启飞控。
# 传感器校准值、RC 通道校准值等每架飞机独有的参数不在本脚本中管理。

set -eo pipefail

# 加载 ROS 与当前工作空间环境。
source /opt/ros/noetic/setup.bash
source /root/Fast-Drone-XI35/devel/setup.bash
set -u

# 避免同时运行两个 MAVROS 实例争用同一个飞控串口。
if rosnode list 2>/dev/null | grep -qx /mavros; then
    echo "已有 /mavros 节点，请先关闭后再运行此脚本。" >&2
    exit 1
fi

MAVROS_LOG="$(mktemp /tmp/apply_px4_params_mavros.XXXXXX.log)"
roslaunch mavros px4.launch >"$MAVROS_LOG" 2>&1 &
MAVROS_PID=$!

cleanup() {
    kill -INT "$MAVROS_PID" 2>/dev/null || true
    wait "$MAVROS_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "正在启动 MAVROS，日志：$MAVROS_LOG"
connected=0
for _ in $(seq 1 60); do
    if ! kill -0 "$MAVROS_PID" 2>/dev/null; then
        echo "MAVROS 启动失败：" >&2
        tail -30 "$MAVROS_LOG" >&2
        exit 1
    fi

    state="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
    if grep -q '^connected: True$' <<<"$state" && \
       rosrun mavros mavparam get MAV_COMP_ID >/dev/null 2>&1; then
        connected=1
        break
    fi
    sleep 1
done

if ((connected == 0)); then
    echo "60 秒内未连接飞控或参数尚未加载完成。" >&2
    tail -30 "$MAVROS_LOG" >&2
    exit 1
fi

state="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
if ! grep -q '^armed: False$' <<<"$state"; then
    echo "飞机已解锁或无法确认解锁状态，拒绝写入参数。" >&2
    exit 1
fi

echo "MAVROS 已连接，开始写入 PX4 参数。"

# MAVLink 0：机载计算机链路。
rosrun mavros mavparam set MAV_0_CONFIG 102
rosrun mavros mavparam set MAV_0_FLOW_CTRL 0
rosrun mavros mavparam set MAV_0_FORWARD 0
rosrun mavros mavparam set MAV_0_MODE 1
rosrun mavros mavparam set MAV_0_RADIO_CTL 0
rosrun mavros mavparam set MAV_0_RATE 92160

# MAVLink 1：遥测链路。
rosrun mavros mavparam set MAV_1_CONFIG 101
rosrun mavros mavparam set MAV_1_FLOW_CTRL 2
rosrun mavros mavparam set MAV_1_FORWARD 0
rosrun mavros mavparam set MAV_1_MODE 0
rosrun mavros mavparam set MAV_1_RADIO_CTL 1
rosrun mavros mavparam set MAV_1_RATE 0

# MAVLink 2 / GPS1 端口：当前不接 GPS，保留该端口的 MAVLink 配置。
rosrun mavros mavparam set GPS_1_CONFIG 0
rosrun mavros mavparam set MAV_2_CONFIG 201
rosrun mavros mavparam set MAV_2_FLOW_CTRL 2
rosrun mavros mavparam set MAV_2_FORWARD 0
rosrun mavros mavparam set MAV_2_MODE 0
rosrun mavros mavparam set MAV_2_RADIO_CTL 1
rosrun mavros mavparam set MAV_2_RATE 0
rosrun mavros mavparam set SER_GPS1_BAUD 115200

# MAVLink 公共设置、串口波特率和 USB MAVLink 模式。
rosrun mavros mavparam set MAV_COMP_ID 1
rosrun mavros mavparam set MAV_FWDEXTSP 1
rosrun mavros mavparam set MAV_HASH_CHK_EN 1
rosrun mavros mavparam set MAV_HB_FORW_EN 1
rosrun mavros mavparam set MAV_PROTO_VER 0
rosrun mavros mavparam set MAV_RADIO_TOUT 5
rosrun mavros mavparam set MAV_TYPE 2
rosrun mavros mavparam set MAV_USEHILGPS 0
rosrun mavros mavparam set SER_TEL1_BAUD 57600
rosrun mavros mavparam set SER_TEL2_BAUD 921600
rosrun mavros mavparam set SYS_USB_AUTO 2
rosrun mavros mavparam set USB_MAV_MODE 2

# 飞控 IMU 输出与积分频率。
rosrun mavros mavparam set IMU_GYRO_RATEMAX 800
rosrun mavros mavparam set IMU_INTEG_RATE 400

# 手动 Position 模式及水平运动包线。
# Direct velocity 模式中，摇杆直接映射为水平速度，回中后由 PX4 锁定位置。
# MPC_ACC_HOR_MAX 主要限制直接速度模式的刹车减速度；起步加速度还会受速度环
# 和倾角限制。46 deg 对应的理想水平加速度上限约为 tan(46 deg)*g=10.2 m/s^2。
rosrun mavros mavparam set MPC_POS_MODE 0
rosrun mavros mavparam set MPC_XY_VEL_MAX 20.0
rosrun mavros mavparam set MPC_VEL_MANUAL 20.0
rosrun mavros mavparam set MPC_VEL_MAN_SIDE 20.0
rosrun mavros mavparam set MPC_VEL_MAN_BACK 20.0
rosrun mavros mavparam set MPC_ACC_HOR_MAX 10.0
# MPC_ACC_HOR/MPC_JERK_MAX 在当前直接速度模式中不参与手动摇杆整形，
# 但分别供自动模式/切回加速度模式以及手动加速度模式使用。
rosrun mavros mavparam set MPC_ACC_HOR 10.0
rosrun mavros mavparam set MPC_JERK_MAX 20.0
rosrun mavros mavparam set MPC_TILTMAX_AIR 46.0
rosrun mavros mavparam set MPC_TILTMAX_LND 12.0
rosrun mavros mavparam set MPC_HOLD_MAX_XY 0.8
rosrun mavros mavparam set MAN_DEADZONE 0.05

# 垂直运动包线保持保守值，不随水平 20 m/s、10 m/s^2 的目标一起提高。
rosrun mavros mavparam set MPC_ALT_MODE 2
rosrun mavros mavparam set MPC_Z_VEL_MAX_UP 3.0
rosrun mavros mavparam set MPC_Z_VEL_MAX_DN 1.5
rosrun mavros mavparam set MPC_ACC_UP_MAX 4.0
rosrun mavros mavparam set MPC_ACC_DOWN_MAX 3.0
rosrun mavros mavparam set MPC_HOLD_MAX_Z 0.6
rosrun mavros mavparam set MPC_TKO_SPEED 1.5
rosrun mavros mavparam set MPC_LAND_SPEED 0.7
rosrun mavros mavparam set MPC_LAND_CRWL 0.3

# EKF2 外部视觉融合：FAST-LIVO2 是唯一的位置、速度和高度辅助源。
# FAST-LIVO2 通过 /mavros/odometry/out 发送 MAVLink ODOMETRY；
# 即使地面测试时仍连接光流或测距硬件，也不参与 EKF2 融合。
rosrun mavros mavparam set EKF2_EV_CTRL 15
rosrun mavros mavparam set EKF2_HGT_REF 3
rosrun mavros mavparam set EKF2_GPS_CTRL 0
rosrun mavros mavparam set EKF2_OF_CTRL 0
rosrun mavros mavparam set EKF2_RNG_CTRL 0
rosrun mavros mavparam set EKF2_EV_DELAY 0
rosrun mavros mavparam set EKF2_EV_NOISE_MD 0
rosrun mavros mavparam set EKF2_EVP_NOISE 0.05
rosrun mavros mavparam set EKF2_EVV_NOISE 0.10
rosrun mavros mavparam set EKF2_EVA_NOISE 0.05
# FAST-LIVO2 已经输出标定到飞控 IMU/机体原点的位姿，
# 因此 EKF2 中不能再次应用杆臂偏移，否则会引入随姿态变化的位置误差。
rosrun mavros mavparam set EKF2_EV_POS_X 0
rosrun mavros mavparam set EKF2_EV_POS_Y 0
rosrun mavros mavparam set EKF2_EV_POS_Z 0

# 通信、遥控器及 OFFBOARD 丢失保护。
rosrun mavros mavparam set COM_ARM_WO_GPS 1
rosrun mavros mavparam set COM_DL_LOSS_T 10
rosrun mavros mavparam set COM_DLL_EXCEPT 0
rosrun mavros mavparam set COM_FAIL_ACT_T 5.0
rosrun mavros mavparam set COM_OBL_RC_ACT 0
rosrun mavros mavparam set COM_OF_LOSS_T 1.0
rosrun mavros mavparam set COM_RCL_EXCEPT 0
rosrun mavros mavparam set COM_RC_IN_MODE 3
rosrun mavros mavparam set COM_RC_LOSS_T 0.5
# 新版 PX4 已将 COM_RC_OVERRIDE/COM_RC_STICK_OV 合并为该参数；
# 正值允许飞手以足够快的摇杆动作从自动控制中接管，-1 表示禁用。
rosrun mavros mavparam set MAN_OVERRIDE_SPD 1.0
rosrun mavros mavparam set NAV_DLL_ACT 0
rosrun mavros mavparam set NAV_RCL_ACT 2

# 6S 电池容量估算与低电保护。
# PX4 的 BAT1_V_EMPTY/BAT1_V_CHARGED 使用单节电压：
# 16.0 V / 6 = 2.6667 V，25.2 V / 6 = 4.2 V。
# 25% 仅警告并禁止解锁；15% 进入严重低电状态并触发 Land。
rosrun mavros mavparam set BAT1_N_CELLS 6
rosrun mavros mavparam set BAT1_V_EMPTY 2.6667
rosrun mavros mavparam set BAT1_V_CHARGED 4.2
rosrun mavros mavparam set BAT_LOW_THR 0.25
rosrun mavros mavparam set BAT_CRIT_THR 0.15
rosrun mavros mavparam set COM_ARM_BAT_MIN 0.25
rosrun mavros mavparam set COM_LOW_BAT_ACT 2

# 解锁、落地自动上锁、预解锁和遥控器飞行模式档位。
rosrun mavros mavparam set COM_DISARM_LAND 2.0
# OFFBOARD 近地下降速度为 0.15 m/s。PX4 以 1.1 倍该阈值判断
# “仍有下降意图”，因此设为 0.10 m/s，确保触地后能够进入
# ground_contact -> maybe_landed -> landed，而不必提高触地速度。
rosrun mavros mavparam set LNDMC_Z_VEL_MAX 0.10
rosrun mavros mavparam set COM_DISARM_PRFLT 10.0
rosrun mavros mavparam set COM_FLTMODE1 8
rosrun mavros mavparam set COM_FLTMODE2 -1
rosrun mavros mavparam set COM_FLTMODE3 -1
rosrun mavros mavparam set COM_FLTMODE4 1
rosrun mavros mavparam set COM_FLTMODE5 -1
rosrun mavros mavparam set COM_FLTMODE6 2
rosrun mavros mavparam set COM_PREARM_MODE 0
rosrun mavros mavparam set COM_SPOOLUP_TIME 1.0

# 遥控器协议、通道映射及开关阈值。
rosrun mavros mavparam set RC_CHAN_CNT 18
rosrun mavros mavparam set RC_INPUT_PROTO 2
rosrun mavros mavparam set RC_PORT_CONFIG 300
rosrun mavros mavparam set RC_MAP_ROLL 1
rosrun mavros mavparam set RC_MAP_PITCH 2
rosrun mavros mavparam set RC_MAP_THROTTLE 3
rosrun mavros mavparam set RC_MAP_YAW 4
rosrun mavros mavparam set RC_MAP_FLTMODE 7
rosrun mavros mavparam set RC_MAP_KILL_SW 8
rosrun mavros mavparam set RC_MAP_ARM_SW 0
rosrun mavros mavparam set RC_MAP_MODE_SW 0
rosrun mavros mavparam set RC_MAP_OFFB_SW 0
rosrun mavros mavparam set RC_MAP_RETURN_SW 0
rosrun mavros mavparam set RC_ARMSWITCH_TH 0.75
rosrun mavros mavparam set RC_KILLSWITCH_TH 0.75
rosrun mavros mavparam set RC_OFFB_TH 0.75
rosrun mavros mavparam set RC_RETURN_TH 0.75

# 飞行日志记录设置。
rosrun mavros mavparam set SDLOG_BACKEND 3
rosrun mavros mavparam set SDLOG_BOOT_BAT 0
rosrun mavros mavparam set SDLOG_DIRS_MAX 0
rosrun mavros mavparam set SDLOG_MISSION 0
rosrun mavros mavparam set SDLOG_MODE 0
rosrun mavros mavparam set SDLOG_PROFILE 1
rosrun mavros mavparam set SDLOG_UTC_OFFSET 0
rosrun mavros mavparam set SDLOG_UUID 1

# 部分参数需要重启后生效。
echo "参数全部写入成功，正在重启飞控。"
rosrun mavros mavcmd long 246 1 0 0 0 0 0 0
echo "飞控重启命令已发送。"
