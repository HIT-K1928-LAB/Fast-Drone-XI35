#!/usr/bin/env python3

import subprocess
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
PX4_MODELS = (
    WORKSPACE / "third_party/px4_sitl/Tools/sitl_gazebo/models"
)
PROFILE_DIR = WORKSPACE / "bringup/profiles/pc_sim"


class Indoor1FastLivo2ContractTest(unittest.TestCase):
    def build_pc_sim_layout(self, odometry_source):
        self.assertEqual(odometry_source, "lidar")
        return subprocess.run(
            ["bash", str(PROFILE_DIR / "tmux.sh"), "--print-layout"],
            check=True,
            text=True,
            capture_output=True,
        ).stdout

    def test_pc_sim_algorithm_configs_live_under_config_directory(self):
        expected_configs = (
            "config/vins/fast_drone_250.yaml",
            "config/px4ctrl/ctrl_param_fpv.yaml",
            "config/fastlivo2/gpu_lidar_sim.yaml",
            "config/fastlivo2/camera_stereo_sim.yaml",
        )
        missing = [
            relative_path
            for relative_path in expected_configs
            if not (PROFILE_DIR / relative_path).is_file()
        ]
        self.assertEqual(missing, [])

    def test_gpu_lidar_scan_period_is_ten_hertz(self):
        root = ET.parse(
            PX4_MODELS / "3d_gpu_lidar/3d_gpu_lidar.sdf"
        ).getroot()
        sensor = root.find("./model/link/sensor[@type='gpu_ray']")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.findtext("update_rate"), "10")

    def test_saved_world_states_start_at_zero_simulation_time(self):
        nonzero_states = {}
        for world_path in WORKSPACE.rglob("*.world"):
            contents = world_path.read_text(errors="replace")
            state_blocks = re.findall(
                r"<state\b[^>]*>(.*?)</state>",
                contents,
                flags=re.DOTALL,
            )
            for state_block in state_blocks:
                for seconds, nanoseconds in re.findall(
                    r"<sim_time>\s*(\d+)\s+(\d+)\s*</sim_time>",
                    state_block,
                ):
                    if (seconds, nanoseconds) == ("0", "0"):
                        continue
                    relative_path = world_path.relative_to(WORKSPACE)
                    nonzero_states[str(relative_path)] = (
                        f"{seconds} {nanoseconds}"
                    )

        self.assertEqual(nonzero_states, {})

    def test_pc_sim_lidar_layout_spawns_indoor1_iris_model(self):
        output = self.build_pc_sim_layout("lidar")
        self.assertIn(
            f"roslaunch '{PROFILE_DIR}/launch/simulator.launch'",
            output,
        )
        self.assertIn(
            f"roslaunch '{PROFILE_DIR}/launch/lidar_bridge.launch'",
            output,
        )
        self.assertIn(
            f"roslaunch '{PROFILE_DIR}/launch/fastlivo2.launch'",
            output,
        )
        self.assertIn(
            f"roslaunch '{PROFILE_DIR}/launch/lidar/px4ctrl.launch'",
            output,
        )
        self.assertNotIn("vglrun", output)
        self.assertNotIn("pc_sim.launch", output)

        roslaunch_commands = [
            line for line in output.splitlines()
            if "roslaunch " in line
        ]
        self.assertTrue(roslaunch_commands)
        self.assertTrue(
            all(":=" not in command for command in roslaunch_commands),
            roslaunch_commands,
        )

    def test_pc_sim_lidar_launch_defines_bridge_topics_and_period(self):
        root = ET.parse(
            PROFILE_DIR / "launch/lidar_bridge.launch"
        ).getroot()
        include = root.find("include")
        self.assertIsNotNone(include)
        arguments = {
            element.attrib["name"]: element.attrib["value"]
            for element in include.findall("arg")
        }
        self.assertEqual(
            arguments,
            {
                "input_topic": "/iris_0/velodyne_points",
                "output_topic": "/iris_0/velodyne_points_fastlivo",
                "scan_period": "0.1",
            },
        )

    def test_pc_sim_profile_does_not_define_ros_node_parameters(self):
        node_parameter_names = (
            "VINS_CONFIG",
            "REALSENSE_CONFIG",
            "PX4CTRL_CONFIG",
            "FASTLIVO2_CONFIG",
            "FASTLIVO2_CAMERA_CONFIG",
            "FASTLIVO2_LIDAR_INPUT_TOPIC",
            "FASTLIVO2_LIDAR_OUTPUT_TOPIC",
            "FASTLIVO2_LIDAR_SCAN_PERIOD",
            "IMU_TOPIC",
        )
        shell_names = " ".join(node_parameter_names)
        result = subprocess.run(
            [
                "bash",
                "-c",
                f"WORKSPACE={WORKSPACE!s}; source {PROFILE_DIR}/profile.env; "
                f"for name in {shell_names}; do "
                "if [[ -v $name ]]; then printf '%s\\n' \"$name\"; fi; "
                "done",
            ],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.stdout, "")

    def test_pc_sim_profile_exports_nvidia_vsync_setting(self):
        result = subprocess.run(
            [
                "bash",
                "-c",
                f"WORKSPACE={WORKSPACE!s}; source {PROFILE_DIR}/profile.env; "
                "bash -c 'printf %s \"$__GL_SYNC_TO_VBLANK\"'",
            ],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertEqual(result.stdout, "0")

    def test_fastlivo_uses_indoor1_topics_and_enables_image_fusion(self):
        config = yaml.safe_load(
            (
                PROFILE_DIR / "config/fastlivo2/gpu_lidar_sim.yaml"
            ).read_text()
        )
        common = config["common"]
        self.assertEqual(
            common["img_topic"],
            "/iris_0/stereo_camera/left/image_raw",
        )
        self.assertEqual(
            common["lid_topic"],
            "/iris_0/velodyne_points_fastlivo",
        )
        self.assertEqual(common["imu_topic"], "/iris_0/imu_gazebo")
        self.assertIs(common["img_en"], True)
        self.assertIs(common["lidar_en"], True)
        self.assertEqual(config["preprocess"]["lidar_type"], 2)
        self.assertEqual(config["preprocess"]["scan_line"], 32)
        self.assertIs(config["uav"]["imu_rate_odom"], True)

    def test_calibration_matches_iris_3d_gpu_lidar_sensor_poses(self):
        config = yaml.safe_load(
            (
                PROFILE_DIR / "config/fastlivo2/gpu_lidar_sim.yaml"
            ).read_text()
        )
        self.assertEqual(
            config["extrin_calib"]["extrinsic_T"],
            [0.0, 0.0, -0.17],
        )
        self.assertEqual(
            config["extrin_calib"]["Pcl"],
            [0.06, -0.08, -0.10],
        )

        camera = yaml.safe_load(
            (
                PROFILE_DIR / "config/fastlivo2/camera_stereo_sim.yaml"
            ).read_text()
        )
        self.assertEqual(camera["cam_width"], 640)
        self.assertEqual(camera["cam_height"], 480)
        self.assertEqual(camera["cam_fx"], 320.0)
        self.assertEqual(camera["cam_fy"], 320.0)
        self.assertEqual(camera["cam_cx"], 320.0)
        self.assertEqual(camera["cam_cy"], 240.0)
        self.assertEqual(camera["cam_d0"], -0.1)
        self.assertEqual(camera["cam_d1"], 0.01)
        self.assertEqual(camera["cam_d2"], 0.00005)
        self.assertEqual(camera["cam_d3"], -0.0001)


if __name__ == "__main__":
    unittest.main(verbosity=2)
