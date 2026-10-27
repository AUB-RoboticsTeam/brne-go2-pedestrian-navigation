#!/usr/bin/env python3

import json
import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from unitree_api.msg import Request
from unitree_go.msg import SportModeState


MOVE_API_ID = 1008
STOP_MOVE_API_ID = 1003


class UnitreeCmdVelBridge(Node):
    """Convert Twist commands to the Unitree Go2 Sport request API."""

    def __init__(self):
        super().__init__('unitree_cmd_vel_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('request_topic', '/api/sport/request')
        self.declare_parameter('sport_state_topic', '/lf/sportmodestate')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('enabled', False)
        self.declare_parameter('auto_arm_on_goal', False)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('command_timeout', 0.30)
        self.declare_parameter('max_linear_x', 0.20)
        self.declare_parameter('max_linear_y', 0.0)
        self.declare_parameter('max_angular_z', 0.40)
        self.declare_parameter('max_tilt', 0.35)
        self.declare_parameter('zero_epsilon', 0.001)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        request_topic = self.get_parameter('request_topic').value
        sport_state_topic = self.get_parameter('sport_state_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.enabled = bool(self.get_parameter('enabled').value)
        self.last_command = None
        self.last_command_time = None
        self.stop_sent = False
        self.sport_state_received = False
        self.tilt_safe = True
        self.auto_arm_pending = False

        self.request_publisher = self.create_publisher(
            Request,
            request_topic,
            10,
        )
        self.command_subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.command_callback,
            10,
        )
        self.sport_state_subscription = self.create_subscription(
            SportModeState,
            sport_state_topic,
            self.sport_state_callback,
            qos_profile_sensor_data,
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10,
        )
        self.armed_publisher = self.create_publisher(
            Bool,
            '~/armed',
            1,
        )
        self.enable_service = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback,
        )
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.timer_callback,
        )

        state = 'ARMED' if self.enabled else 'DISARMED'
        self.get_logger().info(
            f'Go2 cmd_vel bridge {state}: {cmd_vel_topic} -> '
            f'{request_topic}'
        )
        self.publish_stop()
        self.publish_armed_state()

    @staticmethod
    def clamp(value, limit):
        limit = abs(float(limit))
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def scale_planar_command(x, yaw, max_x, max_yaw):
        """Limit forward and yaw speeds while retaining path curvature."""
        x = float(x)
        yaw = float(yaw)
        max_x = abs(float(max_x))
        max_yaw = abs(float(max_yaw))
        scale = 1.0

        if max_x == 0.0:
            x = 0.0
        elif abs(x) > max_x:
            scale = min(scale, max_x / abs(x))

        if max_yaw == 0.0:
            yaw = 0.0
        elif abs(yaw) > max_yaw:
            scale = min(scale, max_yaw / abs(yaw))

        return x * scale, yaw * scale

    def command_callback(self, message):
        values = (
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejected non-finite cmd_vel')
            self.last_command = None
            self.last_command_time = None
            self.publish_stop()
            return

        x, yaw = self.scale_planar_command(
            message.linear.x,
            message.angular.z,
            self.get_parameter('max_linear_x').value,
            self.get_parameter('max_angular_z').value,
        )
        self.last_command = (
            x,
            self.clamp(
                message.linear.y,
                self.get_parameter('max_linear_y').value,
            ),
            yaw,
        )
        self.last_command_time = self.get_clock().now()
        if self.enabled:
            self.stop_sent = False

    def enable_callback(self, request, response):
        requested_state = bool(request.data)
        if not requested_state:
            self.auto_arm_pending = False
        success, message = self.set_enabled(requested_state)
        response.success = success
        response.message = message
        return response

    def set_enabled(self, requested_state):
        if requested_state and not self.sport_state_received:
            return False, 'not armed: no Go2 SportModeState received'
        if requested_state and not self.tilt_safe:
            return False, 'not armed: robot roll/pitch exceeds limit'

        self.enabled = requested_state
        self.last_command = None
        self.last_command_time = None
        self.publish_stop()
        self.publish_armed_state()
        message = 'armed' if self.enabled else 'disarmed and stopped'
        self.get_logger().warn(
            'Go2 motion commands ARMED'
            if self.enabled
            else 'Go2 motion commands DISARMED'
        )
        return True, message

    def goal_callback(self, _message):
        if not bool(self.get_parameter('auto_arm_on_goal').value):
            return

        self.auto_arm_pending = True
        success, message = self.set_enabled(True)
        if success:
            self.auto_arm_pending = False
            self.get_logger().warn('Go2 bridge auto-armed by new goal')
        else:
            self.get_logger().warn(
                f'Goal received; automatic arm is pending: {message}'
            )

    def sport_state_callback(self, message):
        self.sport_state_received = True
        max_tilt = abs(float(self.get_parameter('max_tilt').value))
        roll = float(message.imu_state.rpy[0])
        pitch = float(message.imu_state.rpy[1])
        self.tilt_safe = max(abs(roll), abs(pitch)) <= max_tilt

        if self.auto_arm_pending and self.tilt_safe:
            success, _message = self.set_enabled(True)
            if success:
                self.auto_arm_pending = False
                self.get_logger().warn(
                    'Go2 bridge auto-armed after SportModeState became ready'
                )

        if self.enabled and not self.tilt_safe:
            self.enabled = False
            self.auto_arm_pending = False
            self.last_command = None
            self.last_command_time = None
            self.publish_stop()
            self.publish_armed_state()
            self.get_logger().error(
                'Go2 bridge auto-disarmed: excessive tilt '
                f'(roll={roll:.3f}, pitch={pitch:.3f} rad)'
            )

    def publish_armed_state(self):
        message = Bool()
        message.data = self.enabled
        self.armed_publisher.publish(message)

    def timer_callback(self):
        if not self.enabled or self.last_command_time is None:
            self.publish_stop()
            return

        timeout = float(self.get_parameter('command_timeout').value)
        command_age = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9
        if command_age > timeout:
            self.publish_stop()
            return

        x, y, yaw = self.last_command
        epsilon = float(self.get_parameter('zero_epsilon').value)
        if max(abs(x), abs(y), abs(yaw)) <= epsilon:
            self.publish_stop()
            return

        request = Request()
        request.header.identity.api_id = MOVE_API_ID
        request.parameter = json.dumps({'x': x, 'y': y, 'z': yaw})
        self.request_publisher.publish(request)
        self.stop_sent = False

    def publish_stop(self):
        if self.stop_sent:
            return
        request = Request()
        request.header.identity.api_id = STOP_MOVE_API_ID
        request.parameter = '{}'
        self.request_publisher.publish(request)
        self.stop_sent = True


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    bridge = UnitreeCmdVelBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            bridge.publish_stop()
            rclpy.spin_once(bridge, timeout_sec=0.05)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
