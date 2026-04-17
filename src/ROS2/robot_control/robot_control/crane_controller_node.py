#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import CraneMovement
from robot_interfaces.msg import CraneState
from std_msgs.msg import Float32, Float32MultiArray, UInt8, UInt8MultiArray


class CraneControllerNode(Node):
    AXIS_TRACK = 1
    AXIS_HOIST = 2

    def __init__(self):
        super().__init__('crane_controller_node')
        self.declare_parameter('crane_id', 'crane_left')
        self.declare_parameter('track_tolerance', 5.0)
        self.declare_parameter('action_timeout_sec', 30.0)
        self.declare_parameter('hoist_motor_addr', 2)
        self.declare_parameter('hoist_units_per_tick', 1.0)
        self.declare_parameter('hoist_vel', 500)
        self.declare_parameter('hoist_positive_direction', 1)

        self.crane_id = self.get_parameter('crane_id').value
        self.track_tolerance = float(self.get_parameter('track_tolerance').value)
        self.action_timeout_sec = float(self.get_parameter('action_timeout_sec').value)
        self.hoist_motor_addr = int(self.get_parameter('hoist_motor_addr').value)
        self.hoist_units_per_tick = float(self.get_parameter('hoist_units_per_tick').value)
        self.hoist_vel = int(self.get_parameter('hoist_vel').value)
        self.hoist_positive_direction = int(self.get_parameter('hoist_positive_direction').value)

        self._lock = threading.Lock()
        self._state = CraneState()
        self._last_motor_done = None
        self._last_error = None

        prefix = f'/{self.crane_id}'
        self.track_goal_pub = self.create_publisher(Float32, f'{prefix}/track_goal', 10)
        self.step_pub = self.create_publisher(Float32MultiArray, f'{prefix}/debug_step', 10)
        self.create_subscription(CraneState, f'{prefix}/state', self.state_callback, 20)
        self.create_subscription(UInt8, f'{prefix}/motor_done', self.done_callback, 20)
        self.create_subscription(UInt8MultiArray, f'{prefix}/command_error', self.error_callback, 20)

        cb_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            CraneMovement,
            f'{prefix}/move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )
        self.get_logger().info(f'Crane controller is ready for {self.crane_id}.')

    def goal_callback(self, goal_request):
        if goal_request.axis_id not in (self.AXIS_TRACK, self.AXIS_HOIST):
            self.get_logger().warning(f'reject invalid axis_id={goal_request.axis_id}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def state_callback(self, msg):
        with self._lock:
            self._state = msg

    def done_callback(self, msg):
        with self._lock:
            self._last_motor_done = int(msg.data)

    def error_callback(self, msg):
        with self._lock:
            if len(msg.data) >= 2:
                self._last_error = (int(msg.data[0]), int(msg.data[1]))

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        with self._lock:
            self._last_motor_done = None
            self._last_error = None
            current_state = self._state

        if goal.axis_id == self.AXIS_TRACK:
            target = float(goal.target_position)
            msg = Float32()
            msg.data = target
            self.track_goal_pub.publish(msg)
            success, error_code = self._wait_track(goal_handle, target)
        else:
            current = float(current_state.hoist_depth)
            delta = float(goal.target_position) - current
            ticks = int(round(abs(delta) / max(self.hoist_units_per_tick, 1e-6)))
            if ticks == 0:
                success, error_code = True, 0
            else:
                direction = self.hoist_positive_direction if delta >= 0.0 else 1 - self.hoist_positive_direction
                step_msg = Float32MultiArray()
                step_msg.data = [
                    float(self.hoist_motor_addr),
                    float(direction),
                    float(self.hoist_vel),
                    float(ticks),
                ]
                self.step_pub.publish(step_msg)
                success, error_code = self._wait_hoist(goal_handle, float(goal.target_position))

        result = CraneMovement.Result()
        result.success = success
        result.error_code = error_code
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _wait_track(self, goal_handle, target):
        deadline = time.monotonic() + self.action_timeout_sec
        feedback = CraneMovement.Feedback()
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 0
            with self._lock:
                state = self._state
                error = self._last_error
            feedback.current_position = float(state.track_pos)
            goal_handle.publish_feedback(feedback)
            if error is not None:
                return False, error[1]
            if abs(state.track_pos - target) <= self.track_tolerance:
                return True, 0
            time.sleep(0.05)
        return False, 1

    def _wait_hoist(self, goal_handle, target):
        deadline = time.monotonic() + self.action_timeout_sec
        feedback = CraneMovement.Feedback()
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 0
            with self._lock:
                state = self._state
                motor_done = self._last_motor_done
                error = self._last_error
            feedback.current_position = float(state.hoist_depth)
            goal_handle.publish_feedback(feedback)
            if error is not None:
                return False, error[1]
            if motor_done == self.hoist_motor_addr:
                return True, 0
            if abs(state.hoist_depth - target) <= self.hoist_units_per_tick:
                return True, 0
            time.sleep(0.05)
        return False, 1


def main(args=None):
    rclpy.init(args=args)
    node = CraneControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
