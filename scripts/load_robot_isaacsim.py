#!/usr/bin/env python3
"""
Robot V10 Isaac Sim 加载脚本

使用方法:
    ./isaac-sim.sh --exec /Users/miakho/Code/robot/robot_V10/scripts/load_robot_isaacsim.py

或者在Isaac Sim的Script Editor中运行
"""

from isaacsim import SimulationApp

# 启动仿真应用
simulation_app = SimulationApp({"headless": False})

# 导入必要模块（必须在SimulationApp之后）
import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
import numpy as np
import os

# USD文件路径
USD_PATH = "/Users/miakho/Code/robot/robot_V10/urdf/robot_V10/robot_V10.usd"


def main():
    # 创建世界
    world = World(stage_units_in_meters=1.0)

    # 添加地面
    world.scene.add_default_ground_plane()

    # 加载机器人USD
    add_reference_to_stage(usd_path=USD_PATH, prim_path="/World/Robot_V10")

    # 创建Articulation对象
    robot = world.scene.add(
        Articulation(
            prim_path="/World/Robot_V10",
            name="robot_v10",
        )
    )

    # 重置世界
    world.reset()

    # 打印机器人信息
    print("=" * 50)
    print("Robot V10 加载成功!")
    print(f"关节数量: {robot.num_dof}")
    print(f"关节名称: {robot.dof_names}")
    print("=" * 50)

    # 仿真循环
    while simulation_app.is_running():
        world.step(render=True)

        if world.current_time_step_index % 100 == 0:
            # 获取当前关节位置
            joint_positions = robot.get_joint_positions()
            print(f"当前关节位置: {joint_positions}")


if __name__ == "__main__":
    main()
    simulation_app.close()
