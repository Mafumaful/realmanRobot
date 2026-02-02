#!/usr/bin/env python3
# Copyright (c) 2024, Your Name
# SPDX-License-Identifier: BSD-3-Clause

"""
Robot V10 加载脚本

使用方法:
    ./isaaclab.sh -p scripts/load_robot.py
"""

import argparse
from omni.isaac.lab.app import AppLauncher

# 添加命令行参数
parser = argparse.ArgumentParser(description="Load Robot V10 in Isaac Lab")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动Isaac Sim
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 在AppLauncher之后导入其他模块
import torch
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext

# 导入机器人配置
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.robot_v10_cfg import ROBOT_V10_CFG


def design_scene() -> dict:
    """设计仿真场景，添加地面和机器人"""

    # 添加地面
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # 添加光源
    cfg_light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    cfg_light.func("/World/Light", cfg_light)

    # 创建机器人配置（修改prim路径）
    robot_cfg = ROBOT_V10_CFG.copy()
    robot_cfg.prim_path = "/World/Robot_V10"

    # 创建机器人实例
    robot = Articulation(cfg=robot_cfg)

    return {"robot": robot}


def run_simulator(sim: SimulationContext, entities: dict):
    """运行仿真循环"""

    robot: Articulation = entities["robot"]

    # 定义仿真步进
    sim_dt = sim.get_physics_dt()
    count = 0

    # 仿真循环
    while simulation_app.is_running():
        # 重置
        if count % 500 == 0:
            count = 0
            # 重置机器人状态
            root_state = robot.data.default_root_state.clone()
            robot.write_root_state_to_sim(root_state)
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            print("[INFO]: 重置机器人状态...")

        # 应用随机关节位置目标（示例）
        if count % 100 == 0:
            # 生成随机目标位置
            joint_pos_target = robot.data.default_joint_pos + \
                torch.randn_like(robot.data.joint_pos) * 0.1
            robot.set_joint_position_target(joint_pos_target)

        # 写入数据到仿真
        robot.write_data_to_sim()
        # 步进仿真
        sim.step()
        # 更新机器人状态
        robot.update(sim_dt)
        count += 1


def main():
    """主函数"""

    # 初始化仿真上下文
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cuda:0")
    sim = SimulationContext(sim_cfg)

    # 设置相机视角
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # 设计场景
    scene_entities = design_scene()

    # 播放仿真
    sim.reset()
    print("[INFO]: 仿真设置完成!")

    # 打印机器人信息
    robot = scene_entities["robot"]
    print(f"[INFO]: 机器人关节数量: {robot.num_joints}")
    print(f"[INFO]: 关节名称: {robot.joint_names}")

    # 运行仿真
    run_simulator(sim, scene_entities)


if __name__ == "__main__":
    main()
    simulation_app.close()
