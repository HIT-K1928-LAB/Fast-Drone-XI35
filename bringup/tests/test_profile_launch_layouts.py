#!/usr/bin/env python3

import subprocess
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[2]
PROFILES_DIR = WORKSPACE / "bringup/profiles"
HARDWARE_PROFILES = ("pc", "xi35_10", "xi35_default")
ROSOUT_LOGGER_LAUNCH = (
    WORKSPACE
    / "src/common/utilities/uav_utils/launch/rosout_file_logger.launch"
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
            "pc_sim": ("lidar", "launch/lidar_bridge.launch"),
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
