#!/usr/bin/env python3

import subprocess
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
PROFILES_DIR = WORKSPACE / "bringup/profiles"
HARDWARE_PROFILES = ("pc", "xi35_10", "xi35_default")
ROSOUT_LOGGER_LAUNCH = (
    WORKSPACE
    / "src/common/utilities/uav_utils/launch/rosout_file_logger.launch"
)
ORIN_LIDAR_PROFILE = PROFILES_DIR / "orin-lidar-01"
FAST_LIVO_MID360_ORIN_LAUNCH = (
    WORKSPACE / "src/localization/FAST-LIVO2/launch/mid360_orin.launch"
)
YOPO_SHARED_LAUNCH = (
    WORKSPACE / "src/planning/yopo_planner/launch/yopo_planner.launch"
)
PX4CTRL_SHARED_LAUNCH = (
    WORKSPACE / "src/control/px4ctrl/launch/run_ctrl.launch"
)
FSM_CONFIG_PROFILES = (
    "pc",
    "pc_sim",
    "xi35_10",
    "xi35_default",
    "xi35_euroc",
)


class ProfileLaunchLayoutTest(unittest.TestCase):
    def test_rosout_logger_uses_a_stable_nodes_subdirectory(self):
        root = ET.parse(ROSOUT_LOGGER_LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("arg")
        }
        parameters = {
            parameter.attrib["name"]: parameter.attrib["value"]
            for parameter in root.find("node").findall("param")
        }

        self.assertIn("subdirectory", arguments)
        self.assertIn("subdirectory", parameters)
        self.assertEqual(arguments.get("subdirectory"), "nodes")
        self.assertEqual(parameters.get("subdirectory"), "$(arg subdirectory)")

    def test_hardware_profiles_keep_algorithm_files_under_config(self):
        expected = (
            "config/vins/fast_drone_250.yaml",
            "config/px4ctrl/ctrl_param_fpv.yaml",
        )
        for profile_name in HARDWARE_PROFILES:
            profile_dir = PROFILES_DIR / profile_name
            with self.subTest(profile=profile_name):
                missing = [
                    relative_path
                    for relative_path in expected
                    if not (profile_dir / relative_path).is_file()
                ]
                self.assertEqual(missing, [])

    def test_legacy_profiles_use_offboard_fsm_schema(self):
        for profile_name in FSM_CONFIG_PROFILES:
            config_file = (
                PROFILES_DIR
                / profile_name
                / "config/px4ctrl/ctrl_param_fpv.yaml"
            )
            with self.subTest(profile=profile_name):
                config = yaml.safe_load(config_file.read_text())
                self.assertEqual(config.get("schema_version"), 1)
                self.assertIn("topics", config)
                self.assertIn("livo_odometry", config["topics"])

    def test_shared_px4ctrl_launch_overrides_profile_odometry_topic(self):
        root = ET.parse(PX4CTRL_SHARED_LAUNCH).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("arg")
        }
        node = root.find("node")
        parameters = {
            parameter.attrib["name"]: parameter.attrib.get("value")
            for parameter in node.findall("param")
        }

        self.assertEqual(arguments.get("odom_topic"), "/LIVO2/imu_propagate")
        self.assertEqual(
            parameters.get("topics/livo_odometry"), "$(arg odom_topic)"
        )

    def test_profile_env_files_do_not_select_tmux_layout_scripts(self):
        for profile_env in sorted(PROFILES_DIR.glob("*/profile.env")):
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    f"source {profile_env!s}; "
                    "printf '%s' \"${FLIGHT_PROFILE_SCRIPT:-}\"",
                ],
                check=True,
                text=True,
                capture_output=True,
            )
            with self.subTest(profile=profile_env.parent.name):
                self.assertEqual(result.stdout, "")

    def test_each_profile_tmux_script_owns_its_mode_and_layout(self):
        expected = {
            "pc": ("vins_stereo", "launch/vins.launch"),
            "xi35_10": ("vins_stereo", "launch/vins.launch"),
            "xi35_default": ("vins_stereo", "launch/vins.launch"),
            "xi35_euroc": ("euroc", "launch/vins.launch"),
        }
        for profile_name, (mode, relative_launch) in expected.items():
            profile_dir = PROFILES_DIR / profile_name
            result = subprocess.run(
                ["bash", str(profile_dir / "tmux.sh"), "--print-layout"],
                check=False,
                text=True,
                capture_output=True,
            )
            with self.subTest(profile=profile_name):
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f"MODE|{mode}\n", result.stdout)
                self.assertIn(
                    f"roslaunch '{profile_dir / relative_launch}'",
                    result.stdout,
                )
                self.assertNotIn(":=", result.stdout)

    def test_profile_env_files_do_not_define_ros_node_config_paths(self):
        variable_names = (
            "VINS_CONFIG",
            "REALSENSE_CONFIG",
            "PX4CTRL_CONFIG",
            "EUROC_CONFIG",
        )
        shell_names = " ".join(variable_names)
        for profile_env in sorted(PROFILES_DIR.glob("*/profile.env")):
            command = (
                f"WORKSPACE={WORKSPACE!s}; source {profile_env!s}; "
                f"for name in {shell_names}; do "
                "if [[ -v $name ]]; then printf '%s\\n' \"$name\"; fi; "
                "done"
            )
            result = subprocess.run(
                ["bash", "-c", command],
                check=True,
                text=True,
                capture_output=True,
            )
            with self.subTest(profile=profile_env.parent.name):
                self.assertEqual(result.stdout, "")

    def test_every_profile_launch_is_well_formed_xml(self):
        launch_files = sorted(PROFILES_DIR.glob("*/launch/**/*.launch"))
        self.assertTrue(launch_files)
        for launch_file in launch_files:
            with self.subTest(launch=launch_file):
                self.assertEqual(ET.parse(launch_file).getroot().tag, "launch")

    def test_orin_lidar_profile_has_separate_offboard_fsm_modes(self):
        expected_topics = {
            "lidar": "/LIVO2/imu_propagate",
            "mocap": "/motion_capture/motion_capture_odom",
        }
        for mode, odometry_topic in expected_topics.items():
            launch_file = (
                ORIN_LIDAR_PROFILE / "launch" / mode / "offboard_fsm.launch"
            )
            config_file = (
                ORIN_LIDAR_PROFILE / "config" / mode / "offboard_fsm.yaml"
            )
            with self.subTest(mode=mode):
                self.assertTrue(launch_file.is_file())
                self.assertTrue(config_file.is_file())
                config = yaml.safe_load(config_file.read_text())
                self.assertEqual(config["schema_version"], 1)
                self.assertEqual(
                    config["topics"]["livo_odometry"], odometry_topic
                )
                launch_text = launch_file.read_text()
                self.assertIn(
                    f"config/{mode}/offboard_fsm.yaml", launch_text
                )

    def test_orin_lidar_profile_does_not_reference_legacy_px4ctrl_files(self):
        active_files = [
            ORIN_LIDAR_PROFILE / "tmux.sh",
            *sorted((ORIN_LIDAR_PROFILE / "launch").glob("**/*.launch")),
        ]
        for active_file in active_files:
            with self.subTest(file=active_file):
                text = active_file.read_text()
                self.assertNotIn("px4ctrl.launch", text)
                self.assertNotIn("px4_native_control", text)

        self.assertFalse(
            (ORIN_LIDAR_PROFILE / "config/px4_native_control.yaml").exists()
        )
        self.assertFalse((ORIN_LIDAR_PROFILE / "config/px4ctrl").exists())

    def test_orin_lidar_profile_owns_minimal_fastlivo_configuration(self):
        config_directory = (
            ORIN_LIDAR_PROFILE / "config/lidar/fastlivo2"
        )
        self.assertEqual(
            {path.name for path in config_directory.iterdir() if path.is_file()},
            {
                "mid360_d455.yaml",
                "mid360_driver.json",
                "realsense_d455_advanced.json",
            },
        )

        config = yaml.safe_load(
            (config_directory / "mid360_d455.yaml").read_text()
        )
        realsense = config["camera"]["realsense2_camera"]
        self.assertEqual(realsense["device_type"], "d455")
        self.assertTrue(realsense["enable_color"])
        self.assertEqual(realsense["color_width"], 640)
        self.assertEqual(realsense["color_height"], 480)
        self.assertEqual(realsense["color_fps"], 30)
        self.assertTrue(realsense["enable_depth"])
        self.assertEqual(realsense["depth_width"], 480)
        self.assertEqual(realsense["depth_height"], 270)
        self.assertEqual(realsense["depth_fps"], 30)
        self.assertFalse(realsense["enable_infra"])
        self.assertFalse(realsense["enable_infra1"])
        self.assertFalse(realsense["enable_infra2"])
        self.assertFalse(realsense["enable_pointcloud"])
        self.assertEqual(config["image_input"]["max_rate_hz"], 10.0)

    def test_orin_lidar_launch_passes_profile_owned_driver_configs(self):
        profile_launch = (
            ORIN_LIDAR_PROFILE / "launch/lidar/localization.launch"
        ).read_text()
        self.assertIn('name="mid360_config"', profile_launch)
        self.assertIn("fastlivo2/mid360_driver.json", profile_launch)
        self.assertNotIn("realsense_config_file", profile_launch)

        shared_launch = FAST_LIVO_MID360_ORIN_LAUNCH.read_text()
        root = ET.fromstring(shared_launch)
        mid360_argument = next(
            argument
            for argument in root.findall("arg")
            if argument.attrib["name"] == "mid360_config"
        )
        self.assertNotIn("default", mid360_argument.attrib)
        self.assertNotIn("mid360_rk3588", shared_launch)
        self.assertNotIn("realsense_config_file", shared_launch)

    def test_orin_lidar_launch_resolves_profile_camera_parameters(self):
        launch_file = (
            ORIN_LIDAR_PROFILE / "launch/lidar/localization.launch"
        )
        command = (
            "set -eo pipefail; "
            "source /opt/ros/noetic/setup.bash; "
            f"source {WORKSPACE / 'devel/setup.bash'}; "
            f"roslaunch --dump-params {launch_file}"
        )
        result = subprocess.run(
            ["bash", "-lc", command],
            check=True,
            text=True,
            capture_output=True,
        )
        parameters = yaml.safe_load(result.stdout)
        expected = {
            "/camera/realsense2_camera/device_type": "d455",
            "/camera/realsense2_camera/color_width": 640,
            "/camera/realsense2_camera/color_height": 480,
            "/camera/realsense2_camera/color_fps": 30,
            "/camera/realsense2_camera/enable_depth": True,
            "/camera/realsense2_camera/depth_width": 480,
            "/camera/realsense2_camera/depth_height": 270,
            "/camera/realsense2_camera/depth_fps": 30,
            "/camera/realsense2_camera/enable_infra": False,
            "/camera/realsense2_camera/enable_infra1": False,
            "/camera/realsense2_camera/enable_infra2": False,
            "/camera/realsense2_camera/enable_pointcloud": False,
            "/camera/realsense2_camera/publish_tf": False,
            "/image_input/max_rate_hz": 10.0,
            "/mavros/fcu_url": "/dev/ttyACM0:921600",
        }
        for name, value in expected.items():
            with self.subTest(parameter=name):
                self.assertEqual(parameters.get(name), value)

    def test_orin_lidar_profile_owns_complete_yopo_configuration(self):
        config_file = ORIN_LIDAR_PROFILE / "config/lidar/yopo.yaml"
        profile_launch = ORIN_LIDAR_PROFILE / "launch/lidar/yopo.launch"

        self.assertTrue(config_file.is_file())
        config = yaml.safe_load(config_file.read_text())
        self.assertEqual(
            config["onnx_file"], "$(find yopo_planner)/model/yopo.onnx"
        )
        self.assertEqual(
            config["engine_file"],
            "/root/Fast-Drone-XI35/bringup/profiles/orin-lidar-01/models/yopo/yopo.engine",
        )
        self.assertEqual(config["odom_topic"], "/LIVO2/imu_propagate")
        self.assertEqual(
            config["depth_topic"], "/camera/depth/image_rect_raw"
        )
        self.assertEqual(config["ctrl_topic"], "/position_cmd")
        self.assertEqual(config["goal_topic"], "/move_base_simple/goal")
        self.assertEqual(config["traj_start_topic"], "/traj_start_trigger")
        self.assertTrue(config["wait_for_traj_start_trigger"])
        self.assertEqual(config["min_depth"], 0.04)
        self.assertEqual(config["max_depth"], 20.0)

        extrinsic = config["camera_extrinsic"]
        self.assertEqual(len(extrinsic["rotation_body_depth"]), 9)
        self.assertEqual(len(extrinsic["translation_body_depth"]), 3)

        launch_root = ET.parse(profile_launch).getroot()
        profile_config_arg = next(
            argument
            for argument in launch_root.findall("arg")
            if argument.attrib["name"] == "config_file"
        )
        self.assertIn(
            "config/lidar/yopo.yaml", profile_config_arg.attrib["default"]
        )
        include = launch_root.find("include")
        self.assertIsNotNone(include)
        config_arg = next(
            argument
            for argument in include.findall("arg")
            if argument.attrib["name"] == "config_file"
        )
        self.assertEqual(config_arg.attrib["value"], "$(arg config_file)")
        self.assertNotIn('name="odom_topic"', profile_launch.read_text())
        self.assertNotIn(
            'name="camera_extrinsic_config"', YOPO_SHARED_LAUNCH.read_text()
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
