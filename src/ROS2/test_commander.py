#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import CraneMovement
from robot_interfaces.msg import CraneState
from robot_interfaces.srv import CraneTrigger
from std_msgs.msg import Bool


class SystemTestCommander(Node):
    CRANES = ('crane_left', 'crane_middle', 'crane_right')
    AXIS_TRACK = 1
    AXIS_HOIST = 2

    def __init__(self):
        super().__init__('system_test_commander')
        self.current_crane = 'crane_left'
        self.latest_states = {}
        self.action_clients = {}
        self.trigger_clients = {}

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.beep_pub = self.create_publisher(Bool, '/chassis/cmd_beep', 10)

        for crane_id in self.CRANES:
            self.create_subscription(
                CraneState,
                f'/{crane_id}/state',
                lambda msg, cid=crane_id: self._state_callback(cid, msg),
                20,
            )

        self.get_logger().info('System test commander is ready.')

    def _state_callback(self, crane_id, msg):
        self.latest_states[crane_id] = msg

    def _get_action_client(self, crane_id):
        client = self.action_clients.get(crane_id)
        if client is None:
            client = ActionClient(self, CraneMovement, f'/{crane_id}/move')
            self.action_clients[crane_id] = client
        return client

    def _get_trigger_client(self, crane_id):
        client = self.trigger_clients.get(crane_id)
        if client is None:
            client = self.create_client(CraneTrigger, f'/{crane_id}/trigger')
            self.trigger_clients[crane_id] = client
        return client

    def select_crane(self, crane_id):
        if crane_id not in self.CRANES:
            raise ValueError(f'unsupported crane id: {crane_id}')
        self.current_crane = crane_id

    def print_crane_state(self):
        state = self.latest_states.get(self.current_crane)
        if state is None:
            print(f'[{self.current_crane}] no state received yet')
            return
        print(f'[{self.current_crane}] track_pos={state.track_pos:.2f}')
        print(f'[{self.current_crane}] hoist_depth={state.hoist_depth:.2f}')
        print(f'[{self.current_crane}] yaw={state.carriage_yaw:.2f}')
        print(f'[{self.current_crane}] roll={state.carriage_roll:.2f}')
        print(f'[{self.current_crane}] pitch={state.carriage_pitch:.2f}')
        print(f'[{self.current_crane}] vacuum_power={state.vacuum_power}')
        print(f'[{self.current_crane}] outlet_open={state.is_outlet_open}')

    def move_crane_axis(self, axis_id, target_position):
        client = self._get_action_client(self.current_crane)
        if not client.wait_for_server(timeout_sec=5.0):
            print(f'[{self.current_crane}] action server not ready')
            return

        goal = CraneMovement.Goal()
        goal.axis_id = int(axis_id)
        goal.target_position = float(target_position)

        print(f'[{self.current_crane}] send action axis={goal.axis_id}, target={goal.target_position}')
        future = client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f'[{self.current_crane}] goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            print(f'[{self.current_crane}] no action result')
            return

        print(
            f'[{self.current_crane}] action finished: '
            f'success={result.result.success}, error_code={result.result.error_code}'
        )

    def _feedback_callback(self, feedback_msg):
        print(f'[{self.current_crane}] progress={feedback_msg.feedback.current_position:.2f}')

    def trigger_crane(self, trigger_type, target_value):
        client = self._get_trigger_client(self.current_crane)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'[{self.current_crane}] trigger service not ready')
            return

        request = CraneTrigger.Request()
        request.trigger_type = int(trigger_type)
        request.target_value = int(target_value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            print(f'[{self.current_crane}] no service response')
            return
        print(f'[{self.current_crane}] trigger result: success={response.success}, message={response.message}')

    def move_chassis(self, vx, vth):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(vth)
        self.cmd_vel_pub.publish(msg)
        print(f'[chassis] cmd_vel sent: vx={vx}, vth={vth}')

    def beep_chassis(self):
        msg = Bool()
        msg.data = True
        self.beep_pub.publish(msg)
        print('[chassis] beep sent')


def print_menu(current_crane):
    print('\n================ System Test Commander ================')
    print(f'current crane: {current_crane}')
    print('1. switch crane')
    print('2. crane track move')
    print('3. crane hoist move')
    print('4. vacuum on')
    print('5. vacuum off')
    print('6. outlet open')
    print('7. outlet close')
    print('8. show current crane state')
    print('9. chassis forward')
    print('a. chassis rotate')
    print('s. chassis stop')
    print('b. chassis beep')
    print('q. quit')
    print('======================================================')


def choose_crane(commander):
    print('available cranes:')
    for index, crane_id in enumerate(commander.CRANES, start=1):
        print(f'{index}. {crane_id}')
    choice = input('choose crane index: ').strip()
    mapping = {'1': 'crane_left', '2': 'crane_middle', '3': 'crane_right'}
    crane_id = mapping.get(choice)
    if crane_id is None:
        print('invalid crane selection')
        return
    commander.select_crane(crane_id)
    print(f'switched to {crane_id}')


def main(args=None):
    rclpy.init(args=args)
    commander = SystemTestCommander()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(commander)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            print_menu(commander.current_crane)
            choice = input('input command: ').strip().lower()
            if choice == '1':
                choose_crane(commander)
            elif choice == '2':
                target = float(input('input track target position: ').strip())
                commander.move_crane_axis(SystemTestCommander.AXIS_TRACK, target)
            elif choice == '3':
                target = float(input('input hoist target position: ').strip())
                commander.move_crane_axis(SystemTestCommander.AXIS_HOIST, target)
            elif choice == '4':
                power = int(input('input vacuum duty (0-100, default 100): ').strip() or '100')
                commander.trigger_crane(CraneTrigger.Request.TRIGGER_VACUUM, power)
            elif choice == '5':
                commander.trigger_crane(CraneTrigger.Request.TRIGGER_VACUUM, 0)
            elif choice == '6':
                angle = int(input('input outlet open angle (default 100): ').strip() or '100')
                commander.trigger_crane(CraneTrigger.Request.TRIGGER_OUTLET, angle)
            elif choice == '7':
                commander.trigger_crane(CraneTrigger.Request.TRIGGER_OUTLET, 0)
            elif choice == '8':
                commander.print_crane_state()
            elif choice == '9':
                commander.move_chassis(0.2, 0.0)
            elif choice == 'a':
                commander.move_chassis(0.0, 0.5)
            elif choice == 's':
                commander.move_chassis(0.0, 0.0)
            elif choice == 'b':
                commander.beep_chassis()
            elif choice == 'q':
                break
            else:
                print('invalid command')
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
