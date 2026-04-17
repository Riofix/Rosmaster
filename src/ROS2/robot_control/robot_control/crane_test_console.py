#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import CraneMovement
from robot_interfaces.msg import CraneState
from robot_interfaces.srv import CraneTrigger


class CraneTestConsole(Node):
    AXIS_TRACK = 1
    AXIS_HOIST = 2

    CRANES = ('crane_left', 'crane_middle', 'crane_right')

    def __init__(self):
        super().__init__('crane_test_console')
        self.current_crane = 'crane_left'
        self.latest_states = {}
        self._state_subscriptions = {}
        self._action_clients = {}
        self._trigger_clients = {}

        for crane_id in self.CRANES:
            self._state_subscriptions[crane_id] = self.create_subscription(
                CraneState,
                f'/{crane_id}/state',
                lambda msg, cid=crane_id: self._state_callback(cid, msg),
                20,
            )

        self.get_logger().info('Crane test console is ready.')

    def _state_callback(self, crane_id, msg):
        self.latest_states[crane_id] = msg

    def _get_action_client(self, crane_id):
        client = self._action_clients.get(crane_id)
        if client is None:
            client = ActionClient(self, CraneMovement, f'/{crane_id}/move')
            self._action_clients[crane_id] = client
        return client

    def _get_trigger_client(self, crane_id):
        client = self._trigger_clients.get(crane_id)
        if client is None:
            client = self.create_client(CraneTrigger, f'/{crane_id}/trigger')
            self._trigger_clients[crane_id] = client
        return client

    def select_crane(self, crane_id):
        if crane_id not in self.CRANES:
            raise ValueError(f'unsupported crane_id: {crane_id}')
        self.current_crane = crane_id

    def print_state(self):
        state = self.latest_states.get(self.current_crane)
        if state is None:
            print(f'[{self.current_crane}] 暂无状态回传')
            return
        print(f'[{self.current_crane}] track_pos={state.track_pos:.2f}')
        print(f'[{self.current_crane}] hoist_depth={state.hoist_depth:.2f}')
        print(f'[{self.current_crane}] yaw={state.carriage_yaw:.2f}')
        print(f'[{self.current_crane}] roll={state.carriage_roll:.2f}')
        print(f'[{self.current_crane}] pitch={state.carriage_pitch:.2f}')
        print(f'[{self.current_crane}] vacuum_power={state.vacuum_power}')
        print(f'[{self.current_crane}] outlet_open={state.is_outlet_open}')

    def move_axis(self, axis_id, target_position):
        client = self._get_action_client(self.current_crane)
        if not client.wait_for_server(timeout_sec=5.0):
            print(f'[{self.current_crane}] 动作服务器未就绪')
            return

        goal = CraneMovement.Goal()
        goal.axis_id = axis_id
        goal.target_position = float(target_position)

        print(f'[{self.current_crane}] 正在发送动作: axis={axis_id}, target={target_position}')
        send_future = client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f'[{self.current_crane}] 动作请求被拒绝')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            print(f'[{self.current_crane}] 未收到动作结果')
            return

        print(
            f'[{self.current_crane}] 动作结束: success={result.result.success}, '
            f'error_code={result.result.error_code}'
        )

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f'[{self.current_crane}] 当前进度位置: {feedback.current_position:.2f}')

    def trigger(self, trigger_type, target_value):
        client = self._get_trigger_client(self.current_crane)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'[{self.current_crane}] Trigger 服务未就绪')
            return

        request = CraneTrigger.Request()
        request.trigger_type = int(trigger_type)
        request.target_value = int(target_value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            print(f'[{self.current_crane}] 未收到服务响应')
            return
        print(f'[{self.current_crane}] trigger 返回: success={response.success}, message={response.message}')


def print_menu(current_crane):
    print('\n================ Crane Test Console ================')
    print(f'当前吊具: {current_crane}')
    print('1. 切换吊具')
    print('2. 轨道移动')
    print('3. 升降移动')
    print('4. 启动吸盘')
    print('5. 关闭吸盘')
    print('6. 打开出料口')
    print('7. 关闭出料口')
    print('8. 查看当前状态')
    print('q. 退出')
    print('====================================================')


def choose_crane(console):
    print('可选吊具:')
    for index, crane_id in enumerate(console.CRANES, start=1):
        print(f'{index}. {crane_id}')
    choice = input('选择吊具编号: ').strip()
    mapping = {'1': 'crane_left', '2': 'crane_middle', '3': 'crane_right'}
    crane_id = mapping.get(choice)
    if crane_id is None:
        print('无效选择')
        return
    console.select_crane(crane_id)
    print(f'已切换到 {crane_id}')


def main(args=None):
    rclpy.init(args=args)
    console = CraneTestConsole()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(console)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            print_menu(console.current_crane)
            choice = input('请输入操作编号: ').strip().lower()
            if choice == '1':
                choose_crane(console)
            elif choice == '2':
                target = float(input('输入轨道目标位置: ').strip())
                console.move_axis(CraneTestConsole.AXIS_TRACK, target)
            elif choice == '3':
                target = float(input('输入升降目标位置: ').strip())
                console.move_axis(CraneTestConsole.AXIS_HOIST, target)
            elif choice == '4':
                duty = int(input('输入吸盘功率(0-100，建议100): ').strip() or '100')
                console.trigger(CraneTrigger.Request.TRIGGER_VACUUM, duty)
            elif choice == '5':
                console.trigger(CraneTrigger.Request.TRIGGER_VACUUM, 0)
            elif choice == '6':
                angle = int(input('输入出料口打开角度(建议100): ').strip() or '100')
                console.trigger(CraneTrigger.Request.TRIGGER_OUTLET, angle)
            elif choice == '7':
                console.trigger(CraneTrigger.Request.TRIGGER_OUTLET, 0)
            elif choice == '8':
                console.print_state()
            elif choice == 'q':
                break
            else:
                print('无效输入，请重新选择')
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        console.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
