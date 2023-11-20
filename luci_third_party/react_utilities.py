import math
from collections import OrderedDict
import numpy as np

# ros imports
from geometry_msgs.msg import Twist
from luci_messages.msg import LuciZoneScaling, LuciJoystick
try:
    from awl_core_msgs.msg import LowLevelCommand
except:
    print('awl_core_msgs not found, only able to run lucijoystick version')


class LUCIScalingZoneConverter():
    def __init__(self, node_logger, cmd_deadzone=0.05, zone_threshold=1.0, version='lucijoystick'):
        ''' Initialize the scaling zone converter

        Args:
        + node_logger - logger object from the node that is using this class
        + cmd_deadzone - float, deadzone for joystick commands
        + zone_threshold - float, buffer for each scaling zone (in degrees), accounting for numerical 
                            error in the binning process
        '''
        self.logger = node_logger
        self.version = version
        self.__initialize_scaling_zone_params(cmd_deadzone, zone_threshold)

    def __initialize_scaling_zone_params(self, cmd_deadzone, zone_threshold):
        alpha1 = math.radians(13.7)
        alpha2 = math.radians(78.45)
        alpha3 = math.radians(101.55)
        alpha4 = math.radians(166.3)

        bin_edges = [math.radians(-180)-0.001, -alpha4, -alpha3, -alpha2, -
                     alpha1, alpha1, alpha2, alpha3, alpha4, math.radians(180)+0.001]
        #   0.001 on extreme bins to account for slight numerical error?

        self.get_zone_bin = self.__initialize_bins(bin_edges)

        self.__compute_zone_angle_limits(bin_edges, zone_threshold)

        self.bintozone_map = {1: {'zone_str': 'back',           'index': 0},
                              2: {'zone_str': 'back_left',      'index': 1},
                              3: {'zone_str': 'right',          'index': 2},
                              4: {'zone_str': 'front_right',    'index': 3},
                              5: {'zone_str': 'front',          'index': 4},
                              6: {'zone_str': 'front_left',     'index': 5},
                              7: {'zone_str': 'left',           'index': 6},
                              8: {'zone_str': 'back_right',     'index': 7},
                              9: {'zone_str': 'back',           'index': 0}}

        self.update_scaling_values(LuciZoneScaling())

        # deadzone for joystick, might be handled by luci already
        self.joy_deadzone = cmd_deadzone
        # self.zone_deadzone = math.radians(zone_threshold)     # deadzone for each scaling zone (in degrees)

    def __initialize_bins(self, edges):
        return lambda x: np.digitize(x, edges)

    def __compute_zone_angle_limits(self, zone_edges, _zone_threshold):
        zone_threshold = math.radians(_zone_threshold)
        self.zone_angle_limits = OrderedDict({'back':         {'left': zone_edges[1]-zone_threshold, 'right': zone_edges[8]+zone_threshold},
                                              'back_left':    {'left': zone_edges[2]-zone_threshold, 'right': zone_edges[1]+zone_threshold},
                                              'right':        {'left': zone_edges[3]-zone_threshold, 'right': zone_edges[2]+zone_threshold},
                                              'front_right':  {'left': zone_edges[4]-zone_threshold, 'right': zone_edges[3]+zone_threshold},
                                              'front':        {'left': zone_edges[5]-zone_threshold, 'right': zone_edges[4]+zone_threshold},
                                              'front_left':   {'left': zone_edges[6]-zone_threshold, 'right': zone_edges[5]+zone_threshold},
                                              'left':         {'left': zone_edges[7]-zone_threshold, 'right': zone_edges[6]+zone_threshold},
                                              'back_right':   {'left': zone_edges[8]-zone_threshold, 'right': zone_edges[7]+zone_threshold}})

        self.zone_angle_limits_list = list(self.zone_angle_limits.items())

    def update_scaling_values(self, msg):
        '''
        Function to update values stored in this object 
        Converts LuciZoneScaling message type to dict

        NOTE: zone scaling values are saved in an dict counterclockwise from 'back' with increasing keys
                values can be accessed by 
                    items = list(self.scaling_values.items())
                    items[0][1]['linear'] returns the back_fb value                     

        Args: msg - LuciZoneScaling (https://lucimobility.github.io/luci-ros2-sdk-docs/Packages/Messages/msgs_package)
        '''
        self.scaling_values = OrderedDict({'back':         {'linear': msg.back_fb,         'angular': msg.back_rl},
                                           'back_left':    {'linear': msg.back_left_fb,    'angular': msg.back_left_rl},
                                           'right':        {'linear': msg.right_fb,        'angular': msg.right_rl},
                                           'front_right':  {'linear': msg.front_right_fb,  'angular': msg.front_right_rl},
                                           'front':        {'linear': msg.front_fb,        'angular': msg.front_rl},
                                           'front_left':   {'linear': msg.front_left_fb,   'angular': msg.front_left_rl},
                                           'left':         {'linear': msg.left_fb,         'angular': msg.left_rl},
                                           'back_right':   {'linear': msg.back_right_fb,   'angular': msg.back_right_rl}})

        self.scaling_values_list = list(self.scaling_values.items())

    def scalingzone_from_cmd(self, cmd):
        '''
        NOTE: CHECKED this function is correct (outputs the right zone as we expect from joystick values)
        Args: 
        + cmd - Twist if version is twist, LuciJoystick if version is lucijoystick

        Returns: 
        + zone bin string i.e. 'front', 'front_right' etc (keys of self.scaling_values)
        '''
        if isinstance(cmd, Twist):
            x_ = cmd.linear.x
            z_ = cmd.angular.z          # cw -, ccw +
        elif isinstance(cmd, LuciJoystick):
            x_ = cmd.forward_back       # forward +, backward -
            z_ = -cmd.left_right        # cw right +, ccw left -
        else:
            raise ValueError(
                f'Wrong type input {type(cmd)} to scalingzone_from_cmd -- must be Twist or LuciJoystick')

        if np.linalg.norm([x_, z_]) < self.joy_deadzone:
            return 'origin', None
        else:
            zone_bin = int(self.get_zone_bin(math.atan2(z_, x_)))

            return self.bintozone_map[zone_bin]['zone_str'], self.bintozone_map[zone_bin]['index']

    def compute_scaled_command_change(self, zone_idx, cmd=None):
        '''
        Args: 
        + cmd - Twist if version is twist, LuciJoystick if version is lucijoystick

        Returns: 
        + mag_scaled - float, magnitude of scaled command
        + percent_change - float, percent change in magnitude of command
        '''
        if isinstance(cmd, Twist):
            x_ = cmd.linear.x
            z_ = cmd.angular.z          # cw -, ccw -
        elif isinstance(cmd, LuciJoystick):
            x_ = cmd.forward_back       # forward +, backward -
            z_ = -cmd.left_right        # cw right +, ccw left -
        else:
            raise ValueError(
                f'Wrong type input {type(cmd)} to compute_scaled_command_change -- must be Twist or LuciJoystick')

        mag_ = np.linalg.norm([x_, z_])   # magnitude of command before scaling
        x_scaling_value_ = self.scaling_values_list[zone_idx][1]['linear']
        z_scaling_value_ = self.scaling_values_list[zone_idx][1]['angular']
        x_scaled_ = x_ * x_scaling_value_    # scaling value is in percentage
        z_scaled_ = z_ * z_scaling_value_     # scaling value is in percentage
        # magnitude of command after scaling
        mag_scaled = np.linalg.norm([x_scaled_, z_scaled_])
        # percent change in magnitude of command
        return mag_, mag_scaled, mag_scaled/mag_

    def compute_neighbor_limit_command_info(self, magnitude, neighbor_zone_idx, side_of_neighbor_zone):
        '''
        Args: 
        + magnitude - float, magnitude of original command (either a twist or lucijoystick magnitude)
        + neighbor_zone_idx - int, index of neighbor zone in self.scaling_values_list
        + side_of_neighbor_zone - string, 'left' or 'right' of neighbor zone

        Returns: 
        + limit_cmd - Twist if version is twist, LuciJoystick if version is lucijoystick
        + percent_change - float, percent change in magnitude of command
        '''
        # self.logger.info('neighbor_zone_idx: {}'.format(neighbor_zone_idx))

        # get the scaling values for the neighbor zone of interest
        linear_scale = self.scaling_values_list[neighbor_zone_idx][1]['linear']
        angular_scale = self.scaling_values_list[neighbor_zone_idx][1]['angular']

        # if this is neighbor zone is on the left, then the limit to use is the right limit and vv
        if 'left' in side_of_neighbor_zone.lower():
            limit_side = 'right'
        elif 'right' in side_of_neighbor_zone.lower():
            limit_side = 'left'
        else:
            self.logger.error(
                'side_of_neighbor_zone must be either \"left\" or \"right\"')

        # get the angle of the command limit for the neighbor zone
        cmd_angle = self.zone_angle_limits_list[neighbor_zone_idx][1][limit_side]
        print(f"cmd angle: {cmd_angle}")
        # compute the unscaled command in the neighbor zone
        limit_x_ = magnitude * math.cos(cmd_angle)
        limit_z_ = magnitude * math.sin(cmd_angle)

        if self.version == 'lucijoystick':
            limit_cmd = LuciJoystick()
            limit_cmd.forward_back = int(limit_x_)
            limit_cmd.left_right = -int(limit_z_)

        elif self.version == 'twist':
            limit_cmd = Twist()
            limit_cmd.linear.x = limit_x_
            limit_cmd.angular.z = limit_z_

        else:
            raise ValueError(
                'No such version of compute_neighbor_limit_command_info implemented')

        # compute the scaled command in the neighbor zone
        #      for checking if enough magnitude is preserved
        limit_x_scaled_ = limit_x_ * linear_scale
        limit_z_scaled_ = limit_z_ * angular_scale
        limit_mag_scaled = np.linalg.norm([limit_x_scaled_, limit_z_scaled_])

        if isinstance(limit_cmd, Twist):
            print(f'neighbor zone: {neighbor_zone_idx}, limit_cmd: {limit_cmd.linear.x}, {limit_cmd.angular.z}, limit mag scaled {limit_mag_scaled}, orimag {magnitude}, scale percentage {limit_mag_scaled/magnitude}')
        elif isinstance(limit_cmd, LuciJoystick):
            print(f'neighbor zone: {neighbor_zone_idx}, limit_cmd: {limit_cmd.forward_back}, {limit_cmd.left_right}, limit mag scaled {limit_mag_scaled}, orimag {magnitude}, scale percentage {limit_mag_scaled/magnitude}')

        return (limit_cmd, limit_mag_scaled/magnitude)

    def get_closer_zone(self, cmd, neighbor_zone_idxs):
        '''
        Args: 
        + cmd - Twist if version is twist, LuciJoystick if version is lucijoystick
        + neighbor_zone_idxs - list of ints, the left and right scaling zone idxs

        Returns: 
        + string - "left" or "right", whichever zone is closer to the command
        '''
        if isinstance(cmd, Twist):
            x_ = cmd.linear.x
            z_ = cmd.angular.z          # cw -, ccw -
        elif isinstance(cmd, LuciJoystick):
            x_ = cmd.forward_back       # forward +, backward -
            z_ = -cmd.left_right        # cw right +, ccw left -
        else:
            raise ValueError(
                f'Wrong type input {type(cmd)} to compute_scaled_command_change -- must be Twist or LuciJoystick')

        cmd_angle = math.atan2(z_, x_)
        left_limit_angle = self.zone_angle_limits_list[neighbor_zone_idxs[0]][1]['right']
        right_limit_angle = self.zone_angle_limits_list[neighbor_zone_idxs[1]][1]['left']

        if abs(cmd_angle - left_limit_angle) < abs(cmd_angle - right_limit_angle):
            return 'left'
        else:
            return 'right'
