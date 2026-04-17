# ROS2 Architecture

## Package Layers

### `robot_link`
- Responsibility: physical link layer only
- Inputs/Outputs: raw serial bytes and raw TCP bytes
- Notes: does not parse protocol fields and does not contain business logic

### `robot_protocol`
- Responsibility: protocol encode/decode layer
- Chassis side: handles Rosmaster V3.5.1 packet format
- Crane side: handles STM32F103 packet format
- Notes: publishes parsed state and accepts protocol-facing command topics

### `robot_control`
- Responsibility: control layer
- Inputs: decision-layer actions, services, and control topics
- Outputs: protocol-layer command topics such as `/<crane_id>/track_goal` and `/<crane_id>/debug_step`
- Notes: does not touch serial/TCP directly and does not implement packet framing

### `robot_interfaces`
- Responsibility: shared ROS interfaces
- Contents: `CraneMovement.action`, `CraneTrigger.srv`, `CraneState.msg`, `DeviceState.msg`

### `robot_bringup`
- Responsibility: launch composition
- Notes: assembles link, protocol, and control nodes into one runtime graph

## Command Flow

### Chassis
`decision -> /cmd_vel -> robot_protocol/chassis_packer -> robot_link/serial_node -> Rosmaster_V3.5.1`

### Crane track axis
`decision -> robot_control CraneMovement(Action) -> /<crane_id>/track_goal -> robot_protocol/crane_packer -> robot_link/tcp_server_node -> STM32F103`

### Crane hoist axis
`decision -> robot_control CraneMovement(Action) -> /<crane_id>/debug_step -> robot_protocol/crane_packer -> robot_link/tcp_server_node -> STM32F103`

## Feedback Flow

### Chassis
`Rosmaster_V3.5.1 -> robot_link/serial_node -> /serial/rx_raw -> robot_protocol/chassis_parser -> /odom, /chassis/twist`

### Crane
`STM32F103 -> robot_link/tcp_server_node -> /<crane_id>/tcp_rx_raw -> robot_protocol/crane_parser -> /<crane_id>/state, /<crane_id>/motor_done, /<crane_id>/command_error`

## Boundary Rules

- `robot_link` must not know command semantics.
- `robot_protocol` must not expose decision logic.
- `robot_control` must not construct raw packets or manage sockets/serial ports.
- Rosmaster V3.5.1 and STM32F103 must remain in separate codec/parser paths because their packet formats differ.
