# 机器人关节结构文档索引

本文件夹包含三臂机器人系统的完整关节结构分析文档。

## 文档列表

### 1. [robot_joint_structure.md](robot_joint_structure.md)
**完整的关节结构文档**
- 机器人概述（19自由度系统）
- 底座和平台系统详解
- 三个机械臂的详细关节信息
- 每个关节的参数、运动范围、位置和姿态
- 设计用途分析

### 2. [robot_joint_tree_diagram.md](robot_joint_tree_diagram.md)
**可视化树状结构图**
- ASCII艺术风格的完整关节树
- 机械臂A的详细结构图
- 清晰展示父子关系和连接方式

### 3. [robot_joint_comparison.md](robot_joint_comparison.md)
**三臂对比分析**
- 关节运动范围对比表
- 旋转轴方向对比
- 空间布局和安装位置
- 特殊限制说明（机械臂S的单向关节）
- 工作空间分析

## 快速参考

### 机器人总览
- **总自由度**: 19 DOF
  - 1 DOF: 底座平台旋转
  - 6 DOF: 机械臂A
  - 6 DOF: 机械臂B
  - 6 DOF: 机械臂S

### 三个机械臂特点

| 机械臂 | 位置 | 特点 |
|--------|------|------|
| **A** | 右前方 [0.156, 0.142, -0.227] | 标准6-DOF，全范围运动 |
| **B** | 左前方 [-0.142, 0.156, -0.227] | 标准6-DOF，全范围运动 |
| **S** | 中央下方 [0.0004, 0.0004, -0.400] | 特殊6-DOF，有单向和受限关节 |

### 关键发现

⚠️ **机械臂S的特殊限制**:
- `joint_platform_S2`: 单向运动 (0~189°)
- `joint_platform_S3`: 单向运动 (0~180°)
- `joint_platform_S4`: 受限范围 (±88.8°)
- `joint_platform_S5`: 受限范围 (±88.8°)

## 源文件参考

- URDF: `/root/code/realmanRobot/urdf/robot_V13.urdf`
- 配置: `/root/code/realmanRobot/config/joint_names_robot_V13.yaml`
- 启动: `/root/code/realmanRobot/launch/display.launch`

---

*文档生成日期: 2026-02-02*
