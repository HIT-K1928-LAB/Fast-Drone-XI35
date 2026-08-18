#!/usr/bin/env python3
"""Publish a Hikrobot MVS camera as a ROS1 Bayer image topic."""

import ctypes
import os
import sys

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


MVS_ROOT = os.environ.get("MVCAM_SDK_PATH", "/opt/MVS")
os.environ.setdefault("MVCAM_SDK_PATH", MVS_ROOT)
os.environ.setdefault("MVCAM_COMMON_RUNENV", os.path.join(MVS_ROOT, "lib"))
MVS_PYTHON_IMPORT = os.path.join(MVS_ROOT, "Samples", "aarch64", "Python", "MvImport")
if MVS_PYTHON_IMPORT not in sys.path:
    sys.path.insert(0, MVS_PYTHON_IMPORT)

try:
    from MvCameraControl_class import (  # pylint: disable=wrong-import-position
        MV_ACCESS_Exclusive, MV_CC_DEVICE_INFO, MV_CC_DEVICE_INFO_LIST,
        MV_FRAME_OUT, MV_GIGE_DEVICE, MV_TRIGGER_MODE_OFF, MV_USB_DEVICE, MVCC_FLOATVALUE,
        MvCamera, POINTER, byref, cast, memset, sizeof,
    )
except (ImportError, OSError, TypeError) as exc:
    raise RuntimeError(
        "Cannot load Hikrobot MVS ARM64 Python SDK from {}. Install it on the "
        "host at /opt/MVS and recreate the runtime container.".format(MVS_ROOT)
    ) from exc


def decode_c_string(value):
    """Decode an SDK ctypes character array without assuming its encoding."""
    return memoryview(value).tobytes().split(b"\0", 1)[0].decode("utf-8", errors="replace")


class HikrobotMvsPublisher:
    def __init__(self):
        self.camera = None
        self.opened = False
        self.grabbing = False
        self.serial = rospy.get_param("~device_serial", "")
        self.frame_id = rospy.get_param("~frame_id", "hikrobot_camera")
        self.topic = rospy.get_param("~image_topic", "/hikrobot_camera/image_raw")
        self.width = int(rospy.get_param("~width", 1624))
        self.height = int(rospy.get_param("~height", 1240))
        self.offset_x = int(rospy.get_param("~offset_x", 0))
        self.offset_y = int(rospy.get_param("~offset_y", 0))
        self.binning_x = int(rospy.get_param("~binning_x", 1))
        self.binning_y = int(rospy.get_param("~binning_y", 1))
        self.frame_rate_hz = float(rospy.get_param("~frame_rate_hz", 10.0))
        self.exposure_time_us = float(rospy.get_param("~exposure_time_us", 20000.0))
        self.gain_db = float(rospy.get_param("~gain_db", 6.0))
        self.exposure_auto = str(rospy.get_param("~exposure_auto", "Off"))
        self.auto_exposure_target_gray = float(rospy.get_param("~auto_exposure_target_gray", 100.0))
        self.auto_exposure_min_us = float(rospy.get_param("~auto_exposure_min_us", 1000.0))
        self.auto_exposure_max_us = float(rospy.get_param("~auto_exposure_max_us", 90000.0))
        self.auto_exposure_period_s = float(rospy.get_param("~auto_exposure_period_s", 0.5))
        self.next_auto_exposure_time = rospy.Time(0)
        self.status_period_s = float(rospy.get_param("~status_period_s", 0.5))
        self.next_status_time = rospy.Time(0)
        self.publisher = rospy.Publisher(self.topic, Image, queue_size=2)
        self.exposure_publisher = rospy.Publisher(
            "/hikrobot_camera/exposure_time_us", Float64, queue_size=2)
        self.gain_publisher = rospy.Publisher("/hikrobot_camera/gain_db", Float64, queue_size=2)

    def _select_device(self):
        devices = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, devices)
        if ret != 0:
            raise RuntimeError("MVS device enumeration failed: 0x{:x}".format(ret))
        if devices.nDeviceNum == 0:
            raise RuntimeError("No Hikrobot GigE/USB3Vision device found")

        available = []
        for index in range(devices.nDeviceNum):
            info = cast(devices.pDeviceInfo[index], POINTER(MV_CC_DEVICE_INFO)).contents
            if info.nTLayerType == MV_USB_DEVICE:
                serial = decode_c_string(info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                model = decode_c_string(info.SpecialInfo.stUsb3VInfo.chModelName)
            elif info.nTLayerType == MV_GIGE_DEVICE:
                serial = decode_c_string(info.SpecialInfo.stGigEInfo.chSerialNumber)
                model = decode_c_string(info.SpecialInfo.stGigEInfo.chModelName)
            else:
                continue
            available.append("{} ({})".format(serial, model))
            if not self.serial or self.serial == serial:
                rospy.loginfo("Using Hikrobot camera %s (%s)", serial, model)
                return info
        raise RuntimeError("Hikrobot serial '{}' was not found; available: {}".format(
            self.serial, ", ".join(available) or "none"))

    @staticmethod
    def _check(ret, operation):
        if ret != 0:
            raise RuntimeError("MVS {} failed: 0x{:x}".format(operation, ret))

    def _set_optional_int(self, name, value):
        """Set a GenICam integer feature when this camera exposes it."""
        ret = self.camera.MV_CC_SetIntValue(name, value)
        if ret != 0:
            rospy.loginfo("MVS camera does not expose optional feature %s (0x%x)", name, ret)

    def start(self):
        MvCamera.MV_CC_Initialize()
        try:
            device = self._select_device()
            self.camera = MvCamera()
            self._check(self.camera.MV_CC_CreateHandle(device), "create handle")
            self._check(self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), "open device")
            self.opened = True
            # This node has no software crop: it publishes the camera's complete ROI.
            # Binning is optional in GenICam; this USB camera reports 1x1 by default.
            self._set_optional_int("BinningHorizontal", self.binning_x)
            self._set_optional_int("BinningVertical", self.binning_y)
            self._check(self.camera.MV_CC_SetIntValue("OffsetX", self.offset_x), "set OffsetX")
            self._check(self.camera.MV_CC_SetIntValue("OffsetY", self.offset_y), "set OffsetY")
            self._check(self.camera.MV_CC_SetIntValue("Width", self.width), "set Width")
            self._check(self.camera.MV_CC_SetIntValue("Height", self.height), "set Height")
            if device.nTLayerType == MV_GIGE_DEVICE:
                packet_size = self.camera.MV_CC_GetOptimalPacketSize()
                if packet_size > 0:
                    self._check(self.camera.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size), "set packet size")
            self._check(self.camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF), "disable trigger")
            if self.exposure_auto not in ("Off", "Once", "Continuous", "Software"):
                raise RuntimeError("exposure_auto must be Off, Once, Continuous, or Software")
            # ExposureTime is read-only while ExposureAuto is active.  Disable it
            # first so the supplied value becomes the initial/manual value.
            self._check(self.camera.MV_CC_SetEnumValueByString("ExposureAuto", "Off"),
                        "disable ExposureAuto before setting exposure")
            self._check(self.camera.MV_CC_SetEnumValueByString("GainAuto", "Off"),
                        "disable GainAuto before setting gain")
            # A value <= 0 deliberately keeps the value stored in the camera.
            if self.exposure_time_us > 0:
                self._check(self.camera.MV_CC_SetFloatValue("ExposureTime", self.exposure_time_us),
                            "set exposure time")
            if self.gain_db >= 0:
                self._check(self.camera.MV_CC_SetFloatValue("Gain", self.gain_db), "set gain")
            if self.frame_rate_hz > 0:
                self._check(self.camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True),
                            "enable frame rate control")
                self._check(self.camera.MV_CC_SetFloatValue("AcquisitionFrameRate", self.frame_rate_hz),
                            "set frame rate")
            hardware_auto = "Off" if self.exposure_auto == "Software" else self.exposure_auto
            self._check(self.camera.MV_CC_SetEnumValueByString("ExposureAuto", hardware_auto),
                        "set ExposureAuto={}".format(hardware_auto))
            rospy.loginfo("MVS geometry: %dx%d, Offset=(%d,%d), Binning=%dx%d, software crop=OFF",
                          self.width, self.height, self.offset_x, self.offset_y,
                          self.binning_x, self.binning_y)
            rospy.loginfo("MVS manual image settings: ExposureTime=%.1f us, Gain=%.2f dB",
                          self.exposure_time_us, self.gain_db)
            rospy.loginfo("MVS ExposureAuto=%s", self.exposure_auto)
            rospy.loginfo("MVS frame rate: %.2f Hz", self.frame_rate_hz)
            self._check(self.camera.MV_CC_StartGrabbing(), "start grabbing")
            self.grabbing = True
        except Exception:
            self.close()
            raise

    def spin(self):
        while not rospy.is_shutdown():
            frame = MV_FRAME_OUT()
            memset(byref(frame), 0, sizeof(frame))
            ret = self.camera.MV_CC_GetImageBuffer(frame, 1000)
            if ret != 0 or not frame.pBufAddr:
                rospy.logwarn_throttle(5.0, "MVS did not return an image: 0x%x", ret)
                continue
            try:
                info = frame.stFrameInfo
                width, height = info.nWidth, info.nHeight
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id
                msg.height, msg.width = height, width
                msg.encoding, msg.is_bigendian, msg.step = "bayer_rggb8", 0, width
                raw = ctypes.string_at(frame.pBufAddr, width * height)
                msg.data = raw
                self.publisher.publish(msg)
                self._publish_current_settings()
                self._update_software_auto_exposure(raw)
            finally:
                self.camera.MV_CC_FreeImageBuffer(frame)

    def _publish_current_settings(self):
        """Publish the values currently applied by MVS, including auto exposure."""
        now = rospy.Time.now()
        if now < self.next_status_time:
            return
        self.next_status_time = now + rospy.Duration(max(self.status_period_s, 0.1))
        exposure = MVCC_FLOATVALUE()
        gain = MVCC_FLOATVALUE()
        exposure_ret = self.camera.MV_CC_GetFloatValue("ExposureTime", exposure)
        gain_ret = self.camera.MV_CC_GetFloatValue("Gain", gain)
        if exposure_ret == 0:
            self.exposure_publisher.publish(exposure.fCurValue)
        else:
            rospy.logwarn_throttle(5.0, "MVS cannot read ExposureTime: 0x%x", exposure_ret)
        if gain_ret == 0:
            self.gain_publisher.publish(gain.fCurValue)
        else:
            rospy.logwarn_throttle(5.0, "MVS cannot read Gain: 0x%x", gain_ret)

    def _update_software_auto_exposure(self, raw):
        """A predictable fallback when the camera's hardware auto mode is unsuitable."""
        if self.exposure_auto != "Software":
            return
        now = rospy.Time.now()
        if now < self.next_auto_exposure_time:
            return
        self.next_auto_exposure_time = now + rospy.Duration(max(self.auto_exposure_period_s, 0.1))
        # Sample roughly 8k Bayer pixels; full-frame statistics are unnecessary here.
        stride = max(1, len(raw) // 8192)
        sample = raw[::stride]
        mean_gray = sum(sample) / float(len(sample))
        current = MVCC_FLOATVALUE()
        if self.camera.MV_CC_GetFloatValue("ExposureTime", current) != 0:
            return
        if mean_gray < 1.0:
            ratio = 2.0
        else:
            ratio = self.auto_exposure_target_gray / mean_gray
        # Limit each correction to avoid visible exposure pumping.
        ratio = min(2.0, max(0.5, ratio))
        new_exposure = min(self.auto_exposure_max_us,
                           max(self.auto_exposure_min_us, current.fCurValue * ratio))
        if abs(new_exposure - current.fCurValue) < max(50.0, current.fCurValue * 0.02):
            return
        ret = self.camera.MV_CC_SetFloatValue("ExposureTime", new_exposure)
        if ret == 0:
            rospy.loginfo("Software AE: mean=%.1f, ExposureTime %.0f -> %.0f us",
                          mean_gray, current.fCurValue, new_exposure)
        else:
            rospy.logwarn_throttle(5.0, "Software AE cannot set ExposureTime: 0x%x", ret)

    def close(self):
        if self.camera is not None:
            if self.grabbing:
                self.camera.MV_CC_StopGrabbing()
                self.grabbing = False
            if self.opened:
                self.camera.MV_CC_CloseDevice()
                self.opened = False
            self.camera.MV_CC_DestroyHandle()
            self.camera = None
        MvCamera.MV_CC_Finalize()


def main():
    rospy.init_node("hikrobot_mvs_camera")
    node = HikrobotMvsPublisher()
    try:
        node.start()
        node.spin()
    except Exception as exc:
        rospy.logfatal("Hikrobot MVS camera failed: %s", exc)
        raise
    finally:
        node.close()


if __name__ == "__main__":
    main()
