#!/usr/bin/env python3
"""
URDF 转 USD 转换脚本

功能：将 robot_V10.urdf 转换为 Isaac Sim 可用的 USD 格式

使用方法：
    ./isaac-sim.sh --exec /root/code/realmanRobot/scripts/convert_urdf_to_usd.py
"""

from isaacsim import SimulationApp

# 启动仿真应用（无头模式，转换完成后自动关闭）
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands
from omni.isaac.core.utils.extensions import enable_extension
import carb
import os

# 启用 URDF 导入扩展
enable_extension("omni.importer.urdf")

# 文件路径配置
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
URDF_PATH = os.path.join(PROJECT_ROOT, "urdf/robot_V10.urdf")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "urdf/robot_V10")
USD_OUTPUT = os.path.join(OUTPUT_DIR, "robot_V10.usd")


def convert_urdf_to_usd():
    """转换 URDF 到 USD"""

    print("=" * 60)
    print("开始转换 URDF 到 USD")
    print("=" * 60)
    print(f"URDF 文件: {URDF_PATH}")
    print(f"输出目录: {OUTPUT_DIR}")
    print()

    # 检查 URDF 文件是否存在
    if not os.path.exists(URDF_PATH):
        print(f"错误: URDF 文件不存在: {URDF_PATH}")
        return False

    # 确保输出目录存在
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    try:
        # 导入 URDF
        print("正在导入 URDF...")

        # 使用 Isaac Sim 的 URDF 导入命令
        success, result = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=URDF_PATH,
            import_config=omni.isaac.urdf.ImportConfig(
                set_instanceable=False,
                import_inertia_tensor=True,
                fix_base=False,  # 不固定基座，允许机器人移动
                merge_fixed_joints=False,
                self_collision=False,  # 可根据需要启用
                default_drive_type=omni.isaac.urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION,
                default_drive_strength=1e7,
                default_position_drive_damping=1e5,
                distance_scale=1.0,
                density=0.0,  # 使用 URDF 中的质量信息
            ),
        )

        if not success:
            print("错误: URDF 导入失败")
            return False

        print("✓ URDF 导入成功")

        # 保存为 USD 文件
        print(f"正在保存 USD 文件到: {USD_OUTPUT}")
        omni.usd.get_context().save_as_stage(USD_OUTPUT)
        print("✓ USD 文件保存成功")

        print()
        print("=" * 60)
        print("转换完成!")
        print("=" * 60)
        print(f"输出文件: {USD_OUTPUT}")
        print()
        print("下一步:")
        print("  1. 使用 Isaac Sim 打开 USD 文件检查")
        print("  2. 运行 load_robot_isaacsim.py 加载机器人")
        print("=" * 60)

        return True

    except Exception as e:
        print(f"错误: 转换过程中出现异常: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = convert_urdf_to_usd()
    simulation_app.close()

    # 返回退出码
    exit(0 if success else 1)
