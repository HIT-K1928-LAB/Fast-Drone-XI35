#!/usr/bin/env python3
"""Publish a Hikrobot MVS camera as a ROS1 Bayer image topic."""

import ctypes
import os
import sys
import time

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
        self.sdk_initialized = False
        self.handle_created = False
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
        self.image_node_num = int(rospy.get_param("~image_node_num", 5))
        if self.image_node_num < 1:
            raise ValueError("image_node_num must be at least 1")
        self.usb_transfer_size_bytes = int(
            rospy.get_param("~usb_transfer_size_bytes", 0))
        self.usb_transfer_ways = int(rospy.get_param("~usb_transfer_ways", 8))
        if self.usb_transfer_ways < 1 or self.usb_transfer_ways > 10:
            raise ValueError("usb_transfer_ways must be between 1 and 10")
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
        self.capture_failure_recovery_threshold = max(
            1, int(rospy.get_param("~capture_failure_recovery_threshold", 3)))
        self.recovery_warning_threshold = max(
            1, int(rospy.get_param("~recovery_warning_threshold", 10)))
        self.recovery_window_s = max(
            1.0, float(rospy.get_param("~recovery_window_s", 60.0)))
        self.consecutive_capture_failures = 0
        self.recovery_attempt_times = []
        self.total_recoveries = 0
        self.device_layer_type = None
        self.last_device_frame_num = None
        self.last_device_timestamp = None
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

    def _apply_camera_settings(self, configure_transport=False):
        """Apply all settings that may be lost when the USB stream recovers."""
        # This node has no software crop: it publishes the camera's complete ROI.
        # Binning is optional in GenICam; this USB camera reports 1x1 by default.
        self._set_optional_int("BinningHorizontal", self.binning_x)
        self._set_optional_int("BinningVertical", self.binning_y)
        self._check(self.camera.MV_CC_SetIntValue("OffsetX", self.offset_x), "set OffsetX")
        self._check(self.camera.MV_CC_SetIntValue("OffsetY", self.offset_y), "set OffsetY")
        self._check(self.camera.MV_CC_SetIntValue("Width", self.width), "set Width")
        self._check(self.camera.MV_CC_SetIntValue("Height", self.height), "set Height")
        if configure_transport and self.device_layer_type == MV_GIGE_DEVICE:
            packet_size = self.camera.MV_CC_GetOptimalPacketSize()
            if packet_size > 0:
                self._check(self.camera.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size),
                            "set packet size")
        self._check(self.camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF),
                    "disable trigger")
        if self.exposure_auto not in ("Off", "Once", "Continuous", "Software"):
            raise RuntimeError("exposure_auto must be Off, Once, Continuous, or Software")
        # ExposureTime is read-only while ExposureAuto is active. Disable it
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
        hardware_auto = "Off" if self.exposure_auto == "Software" else self.exposure_auto
        self._check(self.camera.MV_CC_SetEnumValueByString("ExposureAuto", hardware_auto),
                    "set ExposureAuto={}".format(hardware_auto))
        # Set the acquisition rate last: exposure/auto-exposure changes can affect
        # the device's resulting frame-rate state.
        if self.frame_rate_hz > 0:
            self._check(self.camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True),
                        "enable frame rate control")
            self._check(self.camera.MV_CC_SetFloatValue("AcquisitionFrameRate", self.frame_rate_hz),
                        "set frame rate")

    def _configure_usb_transport(self):
        """Bound U3V asynchronous transfers for the RK3588 xHCI controller."""
        if self.device_layer_type != MV_USB_DEVICE:
            return
        if self.usb_transfer_size_bytes > 0:
            self._check(
                self.camera.MV_USB_SetTransferSize(self.usb_transfer_size_bytes),
                "set USB transfer size")
        self._check(self.camera.MV_USB_SetTransferWays(self.usb_transfer_ways),
                    "set USB transfer ways")
        transfer_size = ctypes.c_uint(0)
        transfer_ways = ctypes.c_uint(0)
        self._check(self.camera.MV_USB_GetTransferSize(transfer_size),
                    "read USB transfer size")
        self._check(self.camera.MV_USB_GetTransferWays(transfer_ways),
                    "read USB transfer ways")
        rospy.loginfo("MVS USB transport: transfer_size=%d bytes, transfer_ways=%d",
                      transfer_size.value, transfer_ways.value)

    def _verify_frame_rate(self, context):
        """Read back the configured rate; ResultingFrameRate is only an upper bound."""
        if self.frame_rate_hz <= 0:
            return
        enabled = ctypes.c_bool(False)
        configured = MVCC_FLOATVALUE()
        self._check(self.camera.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", enabled),
                    "read frame rate control")
        self._check(self.camera.MV_CC_GetFloatValue("AcquisitionFrameRate", configured),
                    "read configured frame rate")
        tolerance_hz = max(0.05, self.frame_rate_hz * 0.005)
        if not enabled.value or abs(configured.fCurValue - self.frame_rate_hz) > tolerance_hz:
            raise RuntimeError(
                "MVS frame rate verification failed {}: enable={}, requested={:.3f}, "
                "configured={:.3f}".format(
                    context, enabled.value, self.frame_rate_hz, configured.fCurValue))
        resulting = MVCC_FLOATVALUE()
        resulting_ret = self.camera.MV_CC_GetFloatValue("ResultingFrameRate", resulting)
        if resulting_ret == 0:
            rospy.loginfo("MVS frame rate %s: requested=%.2f, configured=%.2f, "
                          "resulting limit=%.2f Hz", context, self.frame_rate_hz,
                          configured.fCurValue, resulting.fCurValue)
        else:
            rospy.loginfo("MVS frame rate %s: requested=%.2f, configured=%.2f Hz "
                          "(ResultingFrameRate unavailable: 0x%x)", context,
                          self.frame_rate_hz, configured.fCurValue, resulting_ret)

    def _device_stream_was_reset(self, frame_num, device_timestamp):
        frame_rollback = False
        if self.last_device_frame_num is not None and frame_num < self.last_device_frame_num:
            # A normal uint32 wrap has a small positive modular delta. A recovery
            # reset (for example 518 -> 0) has a very large modular delta.
            delta = (frame_num - self.last_device_frame_num) & 0xffffffff
            frame_rollback = delta > 0x7fffffff
        timestamp_rollback = (
            device_timestamp != 0 and
            self.last_device_timestamp not in (None, 0) and
            device_timestamp < self.last_device_timestamp
        )
        return frame_rollback or timestamp_rollback

    def _restart_stream_after_reset(self, reason):
        """Rebuild SDK buffers and restore camera settings after stream recovery."""
        now = time.monotonic()
        self.recovery_attempt_times = [
            attempt for attempt in self.recovery_attempt_times
            if now - attempt < self.recovery_window_s
        ]
        self.recovery_attempt_times.append(now)
        self.total_recoveries += 1
        recent_recoveries = len(self.recovery_attempt_times)
        if recent_recoveries >= self.recovery_warning_threshold:
            # The MVS SDK may recover a USB stream successfully many times while
            # the underlying link remains unstable.  Do not kill the camera node:
            # doing so also tears down a REQUIRED roslaunch and FAST-LIVO2.  Keep
            # recovery failures fatal, but make a high recovery rate telemetry.
            rospy.logerr_throttle(
                5.0,
                "MVS stream remains unstable: %d recoveries in %.1f seconds "
                "(%d total); continuing recovery instead of stopping the camera node",
                recent_recoveries, self.recovery_window_s, self.total_recoveries)
        rospy.logwarn("MVS stream recovery (%s); restoring camera configuration "
                      "(%d recent in %.0f s, %d total)", reason,
                      recent_recoveries, self.recovery_window_s,
                      self.total_recoveries)
        self._check(self.camera.MV_CC_StopGrabbing(), "stop grabbing for recovery")
        self.grabbing = False
        self._check(self.camera.MV_CC_SetImageNodeNum(self.image_node_num),
                    "set image node count for recovery")
        self._configure_usb_transport()
        self._apply_camera_settings()
        if rospy.is_shutdown():
            return
        self._check(self.camera.MV_CC_StartGrabbing(), "restart grabbing after recovery")
        self.grabbing = True
        self._verify_frame_rate("after recovery")
        self.consecutive_capture_failures = 0
        self.last_device_frame_num = None
        self.last_device_timestamp = None

    def start(self):
        self._check(MvCamera.MV_CC_Initialize(), "initialize SDK")
        self.sdk_initialized = True
        try:
            device = self._select_device()
            self.camera = MvCamera()
            self._check(self.camera.MV_CC_CreateHandle(device), "create handle")
            self.handle_created = True
            self._check(self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), "open device")
            self.opened = True
            self.device_layer_type = device.nTLayerType
            self._check(self.camera.MV_CC_SetImageNodeNum(self.image_node_num),
                        "set image node count")
            self._configure_usb_transport()
            self._apply_camera_settings(configure_transport=True)
            rospy.loginfo("MVS geometry: %dx%d, Offset=(%d,%d), Binning=%dx%d, software crop=OFF",
                          self.width, self.height, self.offset_x, self.offset_y,
                          self.binning_x, self.binning_y)
            rospy.loginfo("MVS manual image settings: ExposureTime=%.1f us, Gain=%.2f dB",
                          self.exposure_time_us, self.gain_db)
            rospy.loginfo("MVS ExposureAuto=%s", self.exposure_auto)
            rospy.loginfo("MVS SDK image cache nodes: %d", self.image_node_num)
            self._check(self.camera.MV_CC_StartGrabbing(), "start grabbing")
            self.grabbing = True
            self._verify_frame_rate("after start")
        except Exception:
            self.close()
            raise

    def spin(self):
        while not rospy.is_shutdown():
            frame = MV_FRAME_OUT()
            memset(byref(frame), 0, sizeof(frame))
            ret = self.camera.MV_CC_GetImageBuffer(frame, 1000)
            if ret != 0 or not frame.pBufAddr:
                self.consecutive_capture_failures += 1
                rospy.logwarn_throttle(5.0, "MVS did not return an image: 0x%x", ret)
                if (self.consecutive_capture_failures >=
                        self.capture_failure_recovery_threshold):
                    self._restart_stream_after_reset(
                        "{} consecutive capture failures, last error 0x{:x}".format(
                            self.consecutive_capture_failures, ret))
                continue
            self.consecutive_capture_failures = 0
            restart_needed = False
            frame_num = None
            device_timestamp = None
            msg = None
            raw = None
            try:
                info = frame.stFrameInfo
                frame_num = int(info.nFrameNum)
                device_timestamp = (
                    int(info.nDevTimeStampHigh) << 32
                ) | int(info.nDevTimeStampLow)
                restart_needed = self._device_stream_was_reset(frame_num, device_timestamp)
                if not restart_needed:
                    width, height = info.nWidth, info.nHeight
                    msg = Image()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = self.frame_id
                    msg.height, msg.width = height, width
                    msg.encoding, msg.is_bigendian, msg.step = "bayer_rggb8", 0, width
                    raw = ctypes.string_at(frame.pBufAddr, width * height)
                    msg.data = raw
            finally:
                # The bytes above are an owning copy.  Return the SDK buffer
                # before ROS serialization/publishing, which can be delayed by
                # FAST-LIVO2 load and otherwise starve the MVS buffer pool.
                self._check(self.camera.MV_CC_FreeImageBuffer(frame), "free image buffer")
            if restart_needed:
                self._restart_stream_after_reset(
                    "device frame or timestamp counter moved backwards")
                continue
            self.publisher.publish(msg)
            self._publish_current_settings()
            self._update_software_auto_exposure(raw)
            self.last_device_frame_num = frame_num
            self.last_device_timestamp = device_timestamp

    def _publish_current_settings(self):
        """Publish the values currently applied by MVS, including auto exposure."""
        publish_exposure = self.exposure_publisher.get_num_connections() > 0
        publish_gain = self.gain_publisher.get_num_connections() > 0
        if not publish_exposure and not publish_gain:
            # Control-register reads share the USB link with image streaming and
            # can aggravate recovery on some ARM64/xHCI combinations. Do not poll
            # them unless a consumer explicitly requests the status topics.
            return
        now = rospy.Time.now()
        if now < self.next_status_time:
            return
        self.next_status_time = now + rospy.Duration(max(self.status_period_s, 0.1))
        if publish_exposure:
            exposure = MVCC_FLOATVALUE()
            exposure_ret = self.camera.MV_CC_GetFloatValue("ExposureTime", exposure)
            if exposure_ret == 0:
                self.exposure_publisher.publish(exposure.fCurValue)
            else:
                rospy.logwarn_throttle(5.0, "MVS cannot read ExposureTime: 0x%x", exposure_ret)
        if publish_gain:
            gain = MVCC_FLOATVALUE()
            gain_ret = self.camera.MV_CC_GetFloatValue("Gain", gain)
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
            if self.handle_created:
                self.camera.MV_CC_DestroyHandle()
                self.handle_created = False
            self.camera = None
        if self.sdk_initialized:
            MvCamera.MV_CC_Finalize()
            self.sdk_initialized = False


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
