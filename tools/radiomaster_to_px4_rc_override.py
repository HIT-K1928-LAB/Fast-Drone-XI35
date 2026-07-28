import argparse
import sys
import time
from typing import List

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


def init_joystick(preferred_name: str = ""):
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        raise RuntimeError("没有检测到 joystick。请确认 RadioMaster 已选择 USB Joystick/HID 模式。")

    selected_index = 0

    print("Detected joysticks:")
    for i in range(count):
        j = pygame.joystick.Joystick(i)
        j.init()
        name = j.get_name()
        print(f"  [{i}] {name}, axes={j.get_numaxes()}, buttons={j.get_numbuttons()}, hats={j.get_numhats()}")

        if preferred_name and preferred_name.lower() in name.lower():
            selected_index = i

    joy = pygame.joystick.Joystick(selected_index)
    joy.init()

    print(f"\nUsing joystick [{selected_index}]: {joy.get_name()}")
    print(f"Axes={joy.get_numaxes()}, Buttons={joy.get_numbuttons()}, Hats={joy.get_numhats()}")
    return joy


def get_axes(joy) -> List[float]:
    pygame.event.pump()
    return [joy.get_axis(i) for i in range(joy.get_numaxes())]


def get_buttons(joy) -> List[int]:
    pygame.event.pump()
    return [joy.get_button(i) for i in range(joy.get_numbuttons())]


def debug_loop(joy):
    print("\nDebug mode. Move sticks/switches and observe axis numbers.")
    print("Press Ctrl+C to exit.\n")

    while True:
        axes = [round(x, 3) for x in get_axes(joy)]
        buttons = get_buttons(joy)
        print(f"axes={axes} buttons={buttons}")
        time.sleep(0.2)


def safe_get_axis(joy, axis_index: int) -> float:
    if axis_index < 0 or axis_index >= joy.get_numaxes():
        raise RuntimeError(
            f"Axis index {axis_index} 超出范围。当前 joystick 只有 {joy.get_numaxes()} 个 axes。"
        )
    return joy.get_axis(axis_index)


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


def forward_loop(joy):
    mav = mavutil.mavlink_connection(
        f"udpout:{SERVER_IP}:{SERVER_PORT}",
        source_system=250,
        source_component=190,
        dialect="common"
    )

    # Pygame 2 中用于识别当前 joystick
    try:
        joystick_instance_id = joy.get_instance_id()
    except AttributeError:
        # 兼容较旧的 pygame
        joystick_instance_id = None

    print("\nForwarding RadioMaster to PX4 SITL")
    print(f"UDP target: {SERVER_IP}:{SERVER_PORT}")
    print(f"Target PX4: system={TARGET_SYSTEM}, component={TARGET_COMPONENT}")
    print(f"Joystick instance ID: {joystick_instance_id}")
    print("Press Ctrl+C to stop.\n")

    dt = 1.0 / SEND_HZ
    i = 0

    while True:
        # 不要再只使用 pygame.event.pump()
        # event.get() 会泵送并读取断联事件
        for event in pygame.event.get():

            if event.type == pygame.JOYDEVICEREMOVED:
                removed_instance_id = getattr(event, "instance_id", None)

                # 如果无法获取 instance ID，也按当前设备断联处理
                if (
                    joystick_instance_id is None
                    or removed_instance_id is None
                    or removed_instance_id == joystick_instance_id
                ):
                    print("\nRadioMaster disconnected.")
                    print("Releasing RC override...")

                    send_release(mav)

                    print("RC override released.")
                    print("RC_CHANNELS_OVERRIDE publishing stopped.")
                    return

        # 再增加一层保险：
        # 某些平台可能没有及时产生 JOYDEVICEREMOVED 事件
        try:
            if not joy.get_init():
                print("\nJoystick is no longer initialized.")
                print("Releasing RC override...")

                send_release(mav)

                print("RC override released.")
                print("RC_CHANNELS_OVERRIDE publishing stopped.")
                return

            roll_axis = safe_get_axis(joy, AXIS_ROLL)
            pitch_axis = safe_get_axis(joy, AXIS_PITCH)
            throttle_axis = safe_get_axis(joy, AXIS_THROTTLE)
            yaw_axis = safe_get_axis(joy, AXIS_YAW)

            ch1 = axis_to_pwm(roll_axis, REV_ROLL)
            ch2 = axis_to_pwm(pitch_axis, REV_PITCH)
            ch3 = throttle_axis_to_pwm(
                throttle_axis,
                REV_THROTTLE
            )
            ch4 = axis_to_pwm(yaw_axis, REV_YAW)

            ch5 = switch_axis_to_pwm(
                safe_get_axis(joy, AXIS_CH5),
                REV_CH5
            )
            ch6 = switch_axis_to_pwm(
                safe_get_axis(joy, AXIS_CH6),
                REV_CH6
            )
            ch7 = switch_axis_to_pwm(
                safe_get_axis(joy, AXIS_CH7),
                REV_CH7
            )
            ch8 = switch_axis_to_pwm(
                safe_get_axis(joy, AXIS_CH8),
                REV_CH8
            )

        except pygame.error as e:
            # USB 断开后，get_axis() 在部分系统上会直接抛出 pygame.error
            print(f"\nJoystick access failed: {e}")
            print("Releasing RC override...")

            send_release(mav)

            print("RC override released.")
            print("RC_CHANNELS_OVERRIDE publishing stopped.")
            return

        if i % SEND_HZ == 0:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )

            print(
                f"CH1={ch1:4d} "
                f"CH2={ch2:4d} "
                f"CH3={ch3:4d} "
                f"CH4={ch4:4d} "
                f"CH5={ch5:4d} "
                f"CH6={ch6:4d} "
                f"CH7={ch7:4d} "
                f"CH8={ch8:4d}"
            )

        mav.mav.rc_channels_override_send(
            TARGET_SYSTEM,
            TARGET_COMPONENT,
            ch1,
            ch2,
            ch3,
            ch4,
            ch5,
            ch6,
            ch7,
            ch8
        )

        i += 1
        time.sleep(dt)


def main():
    parser = argparse.ArgumentParser(description="RadioMaster USB Joystick to PX4 SITL RC override")
    parser.add_argument("--debug", action="store_true", help="只打印 joystick axes/buttons，不发送 MAVLink")
    parser.add_argument("--name", default=PREFERRED_NAME, help="优先选择名称包含该字符串的 joystick")
    args = parser.parse_args()

    joy = init_joystick(args.name)

    try:
        if args.debug:
            debug_loop(joy)
        else:
            forward_loop(joy)
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
