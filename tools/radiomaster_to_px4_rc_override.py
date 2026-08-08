import argparse
import sys
import time
from typing import List, Optional, Tuple

import pygame
from pymavlink import mavutil


# ==========================
# 需要按你的环境修改
# ==========================
SERVER_IP = "192.168.31.13"
SERVER_PORT = 18570

TARGET_SYSTEM = 1
TARGET_COMPONENT = 1

# ==========================
# RadioMaster axis 映射
# 你需要根据 joy.cpl / --debug 的结果修改
# ==========================
AXIS_ROLL = 0
AXIS_PITCH = 1
AXIS_THROTTLE = 2
AXIS_YAW = 3

AXIS_CH5 = 4
AXIS_CH6 = 5
AXIS_CH7 = 7
AXIS_CH8 = 6

# 如果方向反了，就改 True / False
REV_ROLL = False
REV_PITCH = True
REV_THROTTLE = True
REV_YAW = False

REV_CH5 = False
REV_CH6 = False
REV_CH7 = False
REV_CH8 = False

SEND_HZ = 50
RECONNECT_INTERVAL = 1.0

# RadioMaster 未连接时持续发布的安全默认值：
# 横滚/俯仰/偏航居中，油门最低，辅助通道置低。
DEFAULT_CHANNELS: Tuple[int, ...] = (
    1500, 1500, 1000, 1500,
    1000, 1000, 1000, 1000,
)

# 摇杆设备名包含这个字符串时优先选择
# 不确定的话可以留空：PREFERRED_NAME = ""
PREFERRED_NAME = "RadioMaster"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def axis_to_pwm(v: float, reverse: bool = False, center: int = 1500, scale: int = 500) -> int:
    """
    pygame axis: -1.0 ~ +1.0
    RC pwm: 1000 ~ 2000
    """
    if reverse:
        v = -v
    v = clamp(v, -1.0, 1.0)
    return int(clamp(center + v * scale, 1000, 2000))


def throttle_axis_to_pwm(v: float, reverse: bool = True) -> int:
    """
    油门通道单独处理。
    最终目标：
      油门最低 -> 1000
      油门最高 -> 2000
    如果方向反了，改 REV_THROTTLE。
    """
    if reverse:
        v = -v
    v = clamp(v, -1.0, 1.0)
    return int((v + 1.0) * 0.5 * 1000 + 1000)


def switch_axis_to_pwm(v: float, reverse: bool = False) -> int:
    """
    三段开关：
      一端 -> 1000
      中间 -> 1500
      另一端 -> 2000
    """
    if reverse:
        v = -v

    if v < -0.5:
        return 1000
    elif v > 0.5:
        return 2000
    else:
        return 1500


def init_joystick_system():
    pygame.init()
    pygame.joystick.init()


def find_joystick(preferred_name: str = "") -> Optional[object]:
    """查找指定的 joystick；找不到时返回 None，等待后续热插拔。"""
    count = pygame.joystick.get_count()
    for i in range(count):
        try:
            joy = pygame.joystick.Joystick(i)
            joy.init()
            name = joy.get_name()
        except pygame.error:
            continue

        # 指定名称时只使用匹配设备，避免 RadioMaster 缺席时误用其他手柄。
        if preferred_name and preferred_name.lower() not in name.lower():
            joy.quit()
            continue

        print(f"\nUsing joystick [{i}]: {name}")
        print(f"Axes={joy.get_numaxes()}, Buttons={joy.get_numbuttons()}, Hats={joy.get_numhats()}")
        return joy

    return None


def get_instance_id(joy) -> Optional[int]:
    try:
        return joy.get_instance_id()
    except AttributeError:
        # 兼容较旧的 pygame
        return None


def get_axes(joy) -> List[float]:
    pygame.event.pump()
    return [joy.get_axis(i) for i in range(joy.get_numaxes())]


def get_buttons(joy) -> List[int]:
    pygame.event.pump()
    return [joy.get_button(i) for i in range(joy.get_numbuttons())]


def debug_loop(preferred_name: str):
    print("\nDebug mode. Move sticks/switches and observe axis numbers.")
    print("Press Ctrl+C to exit.\n")

    joy = None
    joystick_instance_id = None
    next_scan_time = 0.0
    waiting_announced = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                next_scan_time = 0.0
            elif event.type == pygame.JOYDEVICEREMOVED and joy is not None:
                removed_instance_id = getattr(event, "instance_id", None)
                if (
                    joystick_instance_id is None
                    or removed_instance_id is None
                    or removed_instance_id == joystick_instance_id
                ):
                    print("\nRadioMaster disconnected. Waiting for reconnection...")
                    joy = None
                    joystick_instance_id = None
                    next_scan_time = 0.0
                    waiting_announced = True

        now = time.monotonic()
        if joy is None and now >= next_scan_time:
            joy = find_joystick(preferred_name)
            next_scan_time = now + RECONNECT_INTERVAL
            if joy is not None:
                joystick_instance_id = get_instance_id(joy)
                waiting_announced = False
            elif not waiting_announced:
                print("RadioMaster not found. Waiting for reconnection...")
                waiting_announced = True

        if joy is not None:
            try:
                if not joy.get_init():
                    raise pygame.error("joystick is no longer initialized")
                axes = [round(x, 3) for x in get_axes(joy)]
                buttons = get_buttons(joy)
                print(f"axes={axes} buttons={buttons}")
            except pygame.error as e:
                print(f"\nJoystick access failed: {e}")
                print("Waiting for reconnection...")
                joy = None
                joystick_instance_id = None
                next_scan_time = now + RECONNECT_INTERVAL
                waiting_announced = True

        time.sleep(0.2)


def safe_get_axis(joy, axis_index: int) -> float:
    if axis_index < 0 or axis_index >= joy.get_numaxes():
        raise RuntimeError(
            f"Axis index {axis_index} 超出范围。当前 joystick 只有 {joy.get_numaxes()} 个 axes。"
        )
    return joy.get_axis(axis_index)


def read_rc_channels(joy) -> Tuple[int, ...]:
    roll_axis = safe_get_axis(joy, AXIS_ROLL)
    pitch_axis = safe_get_axis(joy, AXIS_PITCH)
    throttle_axis = safe_get_axis(joy, AXIS_THROTTLE)
    yaw_axis = safe_get_axis(joy, AXIS_YAW)

    return (
        axis_to_pwm(roll_axis, REV_ROLL),
        axis_to_pwm(pitch_axis, REV_PITCH),
        throttle_axis_to_pwm(throttle_axis, REV_THROTTLE),
        axis_to_pwm(yaw_axis, REV_YAW),
        switch_axis_to_pwm(safe_get_axis(joy, AXIS_CH5), REV_CH5),
        switch_axis_to_pwm(safe_get_axis(joy, AXIS_CH6), REV_CH6),
        switch_axis_to_pwm(safe_get_axis(joy, AXIS_CH7), REV_CH7),
        switch_axis_to_pwm(safe_get_axis(joy, AXIS_CH8), REV_CH8),
    )


def send_release(mav):
    """
    释放 CH1~CH8 的 RC override。

    对 MAVLink RC_CHANNELS_OVERRIDE 的 CH1~CH8：
      0 表示释放该通道的 override，
      重新交还给正常 RC 输入或 PX4 failsafe 逻辑。
    """
    for _ in range(20):
        try:
            mav.mav.rc_channels_override_send(
                TARGET_SYSTEM,
                TARGET_COMPONENT,
                0, 0, 0, 0,
                0, 0, 0, 0
            )
        except Exception as e:
            print(f"Failed to send release packet: {e}")
            break

        time.sleep(0.02)


def forward_loop(preferred_name: str):
    mav = mavutil.mavlink_connection(
        f"udpout:{SERVER_IP}:{SERVER_PORT}",
        source_system=250,
        source_component=190,
        dialect="common"
    )

    print("\nForwarding RadioMaster to PX4 SITL")
    print(f"UDP target: {SERVER_IP}:{SERVER_PORT}")
    print(f"Target PX4: system={TARGET_SYSTEM}, component={TARGET_COMPONENT}")
    print("Press Ctrl+C to stop.\n")

    dt = 1.0 / SEND_HZ
    i = 0
    joy = None
    joystick_instance_id = None
    next_scan_time = 0.0
    waiting_announced = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                # 新设备加入后立即扫描，不必等到下一次定时轮询。
                next_scan_time = 0.0
            elif event.type == pygame.JOYDEVICEREMOVED and joy is not None:
                removed_instance_id = getattr(event, "instance_id", None)
                if (
                    joystick_instance_id is None
                    or removed_instance_id is None
                    or removed_instance_id == joystick_instance_id
                ):
                    print("\nRadioMaster disconnected.")
                    print("Switching to default RC values and waiting for reconnection...")
                    joy = None
                    joystick_instance_id = None
                    next_scan_time = 0.0
                    waiting_announced = True

        now = time.monotonic()
        if joy is None and now >= next_scan_time:
            joy = find_joystick(preferred_name)
            next_scan_time = now + RECONNECT_INTERVAL
            if joy is not None:
                joystick_instance_id = get_instance_id(joy)
                print(f"Joystick instance ID: {joystick_instance_id}")
                print("RadioMaster connected. Publishing live RC values.")
                waiting_announced = False
            elif not waiting_announced:
                print("RadioMaster not found.")
                print("Publishing default RC values and waiting for reconnection...")
                waiting_announced = True

        source = "DEFAULT"
        channels = DEFAULT_CHANNELS
        if joy is not None:
            try:
                if not joy.get_init():
                    raise pygame.error("joystick is no longer initialized")
                channels = read_rc_channels(joy)
                source = "RadioMaster"
            except (pygame.error, RuntimeError) as e:
                # 部分平台不会及时产生 JOYDEVICEREMOVED，读取失败时同样切换默认值。
                print(f"\nJoystick access failed: {e}")
                print("Switching to default RC values and waiting for reconnection...")
                joy = None
                joystick_instance_id = None
                next_scan_time = now + RECONNECT_INTERVAL
                waiting_announced = True

        if i % SEND_HZ == 0:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )

            print(
                f"source={source:<11} "
                f"CH1={channels[0]:4d} "
                f"CH2={channels[1]:4d} "
                f"CH3={channels[2]:4d} "
                f"CH4={channels[3]:4d} "
                f"CH5={channels[4]:4d} "
                f"CH6={channels[5]:4d} "
                f"CH7={channels[6]:4d} "
                f"CH8={channels[7]:4d}"
            )

        mav.mav.rc_channels_override_send(
            TARGET_SYSTEM,
            TARGET_COMPONENT,
            *channels
        )

        i += 1
        time.sleep(dt)


def main():
    parser = argparse.ArgumentParser(description="RadioMaster USB Joystick to PX4 SITL RC override")
    parser.add_argument("--debug", action="store_true", help="只打印 joystick axes/buttons，不发送 MAVLink")
    parser.add_argument("--name", default=PREFERRED_NAME, help="只使用名称包含该字符串的 joystick；留空则使用第一个")
    args = parser.parse_args()

    init_joystick_system()

    try:
        if args.debug:
            debug_loop(args.name)
        else:
            forward_loop(args.name)
    except KeyboardInterrupt:
        print("\nStopping...")
        if not args.debug:
            try:
                mav = mavutil.mavlink_connection(
                    f"udpout:{SERVER_IP}:{SERVER_PORT}",
                    source_system=250,
                    source_component=190,
                    dialect="common"
                )
                send_release(mav)
                print("RC override released.")
            except Exception as e:
                print(f"Failed to release RC override: {e}")
        sys.exit(0)


if __name__ == "__main__":
    main()
