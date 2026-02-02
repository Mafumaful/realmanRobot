#!/bin/bash
# URDF 转 USD 转换脚本包装器
# 设置必要的环境变量并运行 Isaac Sim

ISAAC_SIM_PATH="/home/wang/isaac-sim"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/convert_urdf_to_usd.py"

echo "=========================================="
echo "URDF 转 USD 转换工具"
echo "=========================================="
echo "Isaac Sim 路径: $ISAAC_SIM_PATH"
echo "转换脚本: $PYTHON_SCRIPT"
echo ""

# 检查 Isaac Sim 是否存在
if [ ! -f "$ISAAC_SIM_PATH/isaac-sim.sh" ]; then
    echo "错误: 找不到 Isaac Sim"
    echo "请检查路径: $ISAAC_SIM_PATH"
    exit 1
fi

# 运行转换
echo "开始转换..."
cd "$ISAAC_SIM_PATH"
./python.sh "$PYTHON_SCRIPT"

echo ""
echo "转换完成！"
