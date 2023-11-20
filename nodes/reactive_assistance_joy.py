#!/usr/bin/env python3
import numpy as np
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from luci_messages.msg import LuciJoystick, LuciZoneScaling, LuciDriveMode

from luci_third_party.react_utilities import LUCIScalingZoneConverter


class ReactiveAssistanceControllerJoystick(Node):
    def __init__(self):
        super().__init__('reactive_assistance_controller_joystick')

        self.__initialize_variables()
        self.__initialize_params()
        self.__initialize_publishers()
        self.__initialize_subscribers()
        self.__initialize_timers()

        drive_mode_ = LuciDriveMode()
        drive_mode_.mode = 3
        self.drive_mode_pub.publish(drive_mode_)
        self.get_logger().info('Reactive Assistance Controller Joystick node initialized')

    def __initialize_variables(self):
        self.user_cmd_joy = LuciJoystick()
        self.user_zone = 'origin'
        self.user_zone_idx = None
        self.reactive_command_joy = LuciJoystick()
        self.scaling_flag = False
        self.user_cmd_flag = False
        # this list hold the idx of all zones for which the reactive
        self.computed_zone_list = []
        #   command computation has taken place, to avoid double computation

    def __initialize_params(self):
        self.LUCI_scaling_handler = LUCIScalingZoneConverter(
            self.get_logger(), zone_threshold=7.0, version='lucijoystick')

        # TODO: get these from yaml file
        # TODO: dynamic thresholding depending on environment
        # threshold for maximum allowable downscaling of user command
        self.downscale_threshold = 0.2
        # threshold for maximum allowable difference between limit cmd magnitudes neighboring zones
        self.neighbor_diff_threshold = 0.1

    def __initialize_publishers(self):
        self.remote_joystick_pub = self.create_publisher(
            LuciJoystick, '/luci/remote_joystick', 10)
        self.drive_mode_pub = self.create_publisher(
            LuciDriveMode, '/luci/drive_mode', 10)
        drive_mode_ = LuciDriveMode()
        drive_mode_.mode = 3
        self.drive_mode_pub.publish(drive_mode_)
        self.get_logger().info('Switched drive mode to AUTO')

        # for debug
        # self.RA_joystick_pub = self.create_publisher(LuciJoystick, '/reactive_joystick', 10)

    def __initialize_subscribers(self):
        sub_cb_grp = MutuallyExclusiveCallbackGroup()

        self.zone_scaling_sub = self.create_subscription(
            LuciZoneScaling, '/luci/scaling', self.luci_zone_scaling_callback, 1, callback_group=sub_cb_grp)
        self.user_joystick_sub = self.create_subscription(
            LuciJoystick, '/luci/joystick_position', self.user_joy_callback, 1, callback_group=sub_cb_grp)

    def __initialize_timers(self):
        timer_cb_grp = MutuallyExclusiveCallbackGroup()
        self.reactive_command_timer = self.create_timer(
            0.05, self.reactive_command_callback, callback_group=timer_cb_grp)

    def luci_zone_scaling_callback(self, zone_vals):
        if not self.scaling_flag:
            self.scaling_flag = True
        self.LUCI_scaling_handler.update_scaling_values(zone_vals)

    def user_joy_callback(self, cmd):
        if not self.user_cmd_flag:
            self.user_cmd_flag = True
        self.user_cmd_joy = cmd

    def reactive_command_callback(self):
        # print("flags: scaling: {}, user_cmd: {}".format(self.scaling_flag, self.user_cmd_flag))
        if self.scaling_flag and self.user_cmd_flag:
            self.reactive_command_joy = self.compute_reactive_autonomy_command()
            _remote_command_zone = self.LUCI_scaling_handler.scalingzone_from_cmd(
                self.reactive_command_joy)
            print(
                f'reactive cmd: fb: {self.reactive_command_joy.forward_back}, lr: {self.reactive_command_joy.left_right}, zone: {_remote_command_zone}')
            # self.RA_joystick_pub.publish(self.reactive_command_joy)
            self.remote_joystick_pub.publish(self.reactive_command_joy)

    def compute_reactive_autonomy_command(self):
        '''    
        Compute the reactive autonomy command based on the current user command and scaling parameters
        TODO: catch cases where the all 4 neighboring zones are suppressed.

        Returns: the reactive autonomy command (unscaled) as a LuciJoystick
        '''
        # get the current zone from the user command
        self.user_zone, self.user_zone_idx = self.LUCI_scaling_handler.scalingzone_from_cmd(
            self.user_cmd_joy)
        self.get_logger().info('\nuser zone: {}, {}'.format(
            self.user_zone, self.user_zone_idx))

        if self.user_zone_idx is None:
            # if the user command is in the origin zone, then the reactive autonomy command is zero
            return LuciJoystick()
        else:
            reactive_cmd = LuciJoystick()

            mag, _, mag_preserved = self.LUCI_scaling_handler.compute_scaled_command_change(
                zone_idx=self.user_zone_idx, cmd=self.user_cmd_joy)

            if mag_preserved > self.downscale_threshold:    # LUCI safety has not downscaled too much
                # passthrough command with no reactive autonomy
                reactive_cmd = self.user_cmd_joy
            else:
                # LUCI safety has downscaled too much, so we need to compute an alternative command
                left_zone = int((self.user_zone_idx + 1) % 8)
                right_zone = int((self.user_zone_idx - 1) % 8)

                # first set of zones to check
                neighbor_zones = [left_zone, right_zone]

                closer_side = self.LUCI_scaling_handler.get_closer_zone(
                    cmd=self.user_cmd_joy, neighbor_zone_idxs=neighbor_zones)

                left_limit_cmd_info = self.LUCI_scaling_handler.compute_neighbor_limit_command_info(
                    mag, neighbor_zones[0], 'left')
                right_limit_cmd_info = self.LUCI_scaling_handler.compute_neighbor_limit_command_info(
                    mag, neighbor_zones[1], 'right')

                if left_limit_cmd_info[1] > self.downscale_threshold or right_limit_cmd_info[1] > self.downscale_threshold:
                    reactive_cmd = self.__pick_neighbor_command(closer_side, left_limit_cmd_info[0], right_limit_cmd_info[0],
                                                                left_limit_cmd_info[1], right_limit_cmd_info[1])

                else:
                    # neither preserve enough magnitude, so compute next neighbor zone for both
                    left_left_zone = int((left_zone + 1) % 8)
                    right_right_zone = int((right_zone - 1) % 8)

                    next_neighbor_zones = [left_left_zone, right_right_zone]

                    next_left_limit_cmd_info = self.LUCI_scaling_handler.compute_neighbor_limit_command_info(
                        mag, next_neighbor_zones[0], 'left')
                    next_right_limit_cmd_info = self.LUCI_scaling_handler.compute_neighbor_limit_command_info(
                        mag, next_neighbor_zones[1], 'right')

                    if next_left_limit_cmd_info[1] > self.downscale_threshold or next_right_limit_cmd_info[1] > self.downscale_threshold:
                        # compute cumulative magnitude preserved for each side
                        left_mag_preserved = (
                            left_limit_cmd_info[1] + next_left_limit_cmd_info[1])
                        right_mag_preserved = (
                            right_limit_cmd_info[1] + next_right_limit_cmd_info[1])

                        reactive_cmd = self.__pick_neighbor_command(closer_side, next_left_limit_cmd_info[0], next_right_limit_cmd_info[0],
                                                                    left_mag_preserved, right_mag_preserved)
                    else:
                        reactive_cmd = LuciJoystick()

            return reactive_cmd

    def __pick_neighbor_command(self, closer_side,
                                left_limit_cmd, right_limit_cmd,
                                left_mag_preserved, right_mag_preserved):
        '''
        Pick the neighbor side based on the following priority:
            1. The one that preserves more magnitude, by a certain threshold
                check for cumulative command preseved across neighbors (handled in main loop)
            2. if the magnitude preserved is similar, pick the one that is closer to the direction of the user command

        Args:
        + closer_side (str): the side that is closer to the user command
        + left_limit_cmd (LuciJoystick): the command that is the limit of the left neighbor zone
        + right_limit_cmd (LuciJoystick): the command that is the limit of the right neighbor zone
        + left_mag_preserved (float): the magnitude preserved by the left neighbor zone
        + right_mag_preserved (float): the magnitude preserved by the right neighbor zone

        Returns: 
        + the neighbor command that is picked as a LuciJoystick
        '''
        if abs(left_mag_preserved - right_mag_preserved) < self.neighbor_diff_threshold:
            print(
                "difference between neighbors is too little, picking based on DIRECTION")
            # tiebreak using the direction the command is in
            if 'left' in closer_side.lower():
                return left_limit_cmd
            else:
                return right_limit_cmd
        else:
            print("difference between neighbors is enough, picking based on MAGNITUDE")
            # pick the one with more magnitude preserved
            if left_mag_preserved > right_mag_preserved:
                print(
                    f"left side cumulatively preserves more magnitude: left {left_limit_cmd} {left_mag_preserved} vs right {right_limit_cmd} {right_mag_preserved}")
                return left_limit_cmd
            else:
                print(
                    f"right side preserves more magnitude: left {left_limit_cmd} {left_mag_preserved} vs right {right_limit_cmd} {right_mag_preserved}")
                return right_limit_cmd

    def shutdown_hook(self):
        self.reactive_command_joy = LuciJoystick()
        self.remote_joystick_pub.publish(self.reactive_command_joy)

        drive_mode_ = LuciDriveMode()
        drive_mode_.mode = 1
        self.drive_mode_pub.publish(drive_mode_)

        self.get_logger().info('Switched drive mode to USER')
        self.get_logger().info('Published ZERO cmd and switched back to native joystick')


def signal_handler(node):
    # Publish the shutdown message
    node.shutdown_hook()

    # Shutdown rclpy
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=4)
    controller = ReactiveAssistanceControllerJoystick()
    executor.add_node(controller)

    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(controller))

    try:
        controller.get_logger().info(
            'Reactive Assistance Controller (Joystick) is running, shutdown with CTRL-C')
        executor.spin()
    except (ExternalShutdownException):
        controller.shutdown_hook()
        controller.get_logger().info(
            'Keyboard interrupt, shutting down Reactive Assistance Controller (Joystick)\n')

    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
