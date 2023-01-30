# -*- coding: utf-8 -*-
import launch
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch_testing
import os
import sys
import unittest


def generate_test_description():

    abort_to_disarming = Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "abort_to_disarming",
            ]
        ),
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            abort_to_disarming,
            # TimerAction(period=2.0, actions=[basic_test]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "abort_to_disarming": abort_to_disarming,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, abort_to_disarming):
        self.proc_info.assertWaitForShutdown(abort_to_disarming, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes
    def test_gtest_pass(self, proc_info, abort_to_disarming):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=abort_to_disarming
        )