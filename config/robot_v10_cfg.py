# Copyright (c) 2024, Your Name
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Robot V10 parallel manipulator."""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

# 获取当前文件的目录，用于构建USD文件的相对路径
import os
ROBOT_V10_USD_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "urdf", "robot_V10", "robot_V10.usd"
)

##
# Configuration
##

ROBOT_V10_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ROBOT_V10_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),  # wxyz格式
        joint_pos={
            # 平台主关节
            "joint_platform_D1": 0.0,
            # 机械臂A关节
            "joint_platform_A1": 0.0,
            "joint_platform_A2": 0.0,
            "joint_platform_A3": 0.0,
            "joint_platform_A4": 0.0,
            "joint_platform_A5": 0.0,
            "joint_platform_A6": 0.0,
            # 机械臂B关节
            "joint_platform_B1": 0.0,
            "joint_platform_B2": 0.0,
            "joint_platform_B3": 0.0,
            "joint_platform_B4": 0.0,
            "joint_platform_B5": 0.0,
            "joint_platform_B6": 0.0,
            # 机械臂S关节
            "joint_platform_S1": 0.0,
            "joint_platform_S2": 0.0,
            "joint_platform_S3": 0.0,
            "joint_platform_S4": 0.0,
            "joint_platform_S5": 0.0,
            "joint_platform_S6": 0.0,
        },
    ),
    actuators={
        # 平台主关节执行器
        "platform": ImplicitActuatorCfg(
            joint_names_expr=["joint_platform_D1"],
            effort_limit=100.0,
            velocity_limit=50.0,
            stiffness=1000.0,
            damping=100.0,
        ),
        # 机械臂A执行器
        "arm_a": ImplicitActuatorCfg(
            joint_names_expr=["joint_platform_A.*"],
            effort_limit=100.0,
            velocity_limit=50.0,
            stiffness=800.0,
            damping=80.0,
        ),
        # 机械臂B执行器
        "arm_b": ImplicitActuatorCfg(
            joint_names_expr=["joint_platform_B.*"],
            effort_limit=100.0,
            velocity_limit=50.0,
            stiffness=800.0,
            damping=80.0,
        ),
        # 机械臂S执行器
        "arm_s": ImplicitActuatorCfg(
            joint_names_expr=["joint_platform_S.*"],
            effort_limit=100.0,
            velocity_limit=50.0,
            stiffness=800.0,
            damping=80.0,
        ),
    },
)
