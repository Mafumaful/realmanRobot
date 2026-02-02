"""
在 Isaac Sim GUI 中运行的 URDF 转换脚本

使用方法：
1. 启动 Isaac Sim GUI: /home/wang/isaac-sim/isaac-sim.sh
2. 打开 Script Editor: Window -> Script Editor
3. 复制此脚本内容到 Script Editor
4. 点击 Run 按钮执行

或者直接在 Python Console 中运行
"""

import omni.kit.commands
from pxr import Usd, UsdGeom
import omni.usd
import os

# 配置路径
PROJECT_ROOT = "/root/code/realmanRobot"
URDF_PATH = os.path.join(PROJECT_ROOT, "urdf/robot_V10.urdf")
OUTPUT_USD = os.path.join(PROJECT_ROOT, "urdf/robot_V10/robot_V10.usd")

print("=" * 60)
print("开始转换 URDF 到 USD")
print("=" * 60)
print(f"URDF 文件: {URDF_PATH}")
print(f"输出文件: {OUTPUT_USD}")
print()

# 检查文件是否存在
if not os.path.exists(URDF_PATH):
    print(f"错误: URDF 文件不存在: {URDF_PATH}")
else:
    try:
        # 导入 URDF
        print("正在导入 URDF...")

        omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=URDF_PATH,
            import_config=omni.isaac.urdf.ImportConfig(
                set_instanceable=False,
                import_inertia_tensor=True,
                fix_base=False,
                merge_fixed_joints=False,
                self_collision=False,
                default_drive_type=omni.isaac.urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION,
                default_drive_strength=1e7,
                default_position_drive_damping=1e5,
                distance_scale=1.0,
                density=0.0,
            ),
        )

        print("✓ URDF 导入成功")

        # 保存 USD
        print(f"正在保存到: {OUTPUT_USD}")
        omni.usd.get_context().save_as_stage(OUTPUT_USD)
        print("✓ USD 文件保存成功")

        print()
        print("=" * 60)
        print("转换完成!")
        print("=" * 60)

    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
