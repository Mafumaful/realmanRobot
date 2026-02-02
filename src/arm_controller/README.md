# Arm Controller

ROS2 Humble 机械臂控制功能包，支持正逆运动学和三次样条轨迹规划。

## 功能

- 正逆运动学求解 (基于KDL库)
- 三次样条(Cubic Spline)轨迹插值
- 支持三个机械臂: arm_A, arm_B, arm_S

## 依赖

```bash
sudo apt install ros-humble-kdl-parser ros-humble-orocos-kdl ros-humble-tf2-kdl
```

## 编译

```bash
cd /path/to/realmanRobot
colcon build --packages-select arm_controller
source install/setup.bash
```

## 运行

```bash
ros2 launch arm_controller arm_controller.launch.py \
  urdf_path:=/path/to/realmanRobot/urdf/robot_V13.urdf
```

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/robot/joint_command` | sensor_msgs/JointState | 发布 | 关节指令 |
| `/robot/joint_states` | sensor_msgs/JointState | 订阅 | 关节状态 |
| `~/target_pose` | geometry_msgs/PoseStamped | 订阅 | 目标位姿 |

## 使用示例

发送目标点 (通过frame_id指定机械臂):

```bash
# 控制 arm_A
ros2 topic pub /arm_controller_node/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'arm_A'}, pose: {position: {x: 0.3, y: 0.1, z: -0.3}, orientation: {w: 1.0}}}"

# 控制 arm_B
ros2 topic pub /arm_controller_node/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'arm_B'}, pose: {position: {x: -0.3, y: 0.1, z: -0.3}, orientation: {w: 1.0}}}"

# 控制 arm_S
ros2 topic pub /arm_controller_node/target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'arm_S'}, pose: {position: {x: 0.0, y: 0.0, z: -0.5}, orientation: {w: 1.0}}}"
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `urdf_path` | "" | URDF文件路径 |
| `control_rate` | 100.0 | 控制频率 (Hz) |
| `trajectory_duration` | 3.0 | 轨迹执行时间 (秒) |
