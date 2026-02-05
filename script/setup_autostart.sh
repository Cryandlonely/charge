#!/bin/bash

# ROS2 charge_control 开机自启配置脚本
# 用法: sudo ./setup_autostart.sh

set -e

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请使用 sudo 运行此脚本"
    echo "用法: sudo $0"
    exit 1
fi

echo "=========================================="
echo "   ROS2 charge_control 开机自启配置脚本"
echo "=========================================="

# 获取实际用户名（非 root）
REAL_USER=${SUDO_USER:-sunrise}
WORKSPACE="/home/${REAL_USER}/charge/charge_ws"

echo "[1/3] 用户: $REAL_USER"
echo "      工作空间: $WORKSPACE"

# 创建 systemd 服务文件
echo "[2/3] 创建 systemd 服务文件..."
cat > /etc/systemd/system/charge_control.service << EOF
[Unit]
Description=Charge Control ROS2 Node
After=network.target

[Service]
Type=simple
User=${REAL_USER}
WorkingDirectory=${WORKSPACE}
ExecStartPre=/bin/sleep 5
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && ros2 run charge_control charge'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
echo "      已创建 /etc/systemd/system/charge_control.service"

# 启用服务
echo "[3/3] 启用开机自启..."
systemctl daemon-reload
systemctl enable charge_control.service
echo "      服务已启用"

echo ""
echo "=========================================="
echo "              配置完成!"
echo "=========================================="
echo ""
echo "常用命令:"
echo "  启动服务:   sudo systemctl start charge_control"
echo "  停止服务:   sudo systemctl stop charge_control"
echo "  重启服务:   sudo systemctl restart charge_control"
echo "  查看状态:   sudo systemctl status charge_control"
echo "  查看日志:   journalctl -u charge_control -f"
echo "  禁用自启:   sudo systemctl disable charge_control"
echo ""
