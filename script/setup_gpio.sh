#!/bin/bash

# GPIO 永久配置脚本
# 用法: sudo ./setup_gpio.sh

set -e

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请使用 sudo 运行此脚本"
    echo "用法: sudo $0"
    exit 1
fi

echo "=========================================="
echo "       GPIO 永久权限配置脚本"
echo "=========================================="

# 获取实际用户名（非 root）
REAL_USER=${SUDO_USER:-$USER}
echo "[1/5] 当前用户: $REAL_USER"

# 1. 创建 gpio 用户组并将用户添加到该组
echo "[2/5] 创建 gpio 用户组并添加用户..."
groupadd -f gpio
usermod -aG gpio "$REAL_USER"
echo "      用户 $REAL_USER 已添加到 gpio 组"

# 2. 创建 udev 规则文件
echo "[3/5] 创建 udev 规则文件..."
cat > /etc/udev/rules.d/99-gpio.rules << 'EOF'
# Allow gpio group to access GPIO pins
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
SUBSYSTEM=="gpio", KERNEL=="gpio*", MODE="0660", GROUP="gpio"

# Allow gpio group to export/unexport GPIOs
SUBSYSTEM=="gpio", ACTION=="add", RUN+="/bin/chmod 0666 /sys/class/gpio/export"
SUBSYSTEM=="gpio", ACTION=="add", RUN+="/bin/chmod 0666 /sys/class/gpio/unexport"

# When a GPIO is exported, allow all users to access it
SUBSYSTEM=="gpio", ACTION=="add", KERNEL=="gpio[0-9]*", RUN+="/bin/sh -c 'chmod 0666 /sys/class/gpio/gpio%n/direction; chmod 0666 /sys/class/gpio/gpio%n/value; chmod 0666 /sys/class/gpio/gpio%n/edge 2>/dev/null || true'"
EOF
echo "      已创建 /etc/udev/rules.d/99-gpio.rules"

# 3. 重新加载 udev 规则
echo "[4/5] 重新加载 udev 规则..."
udevadm control --reload-rules
udevadm trigger
echo "      udev 规则已重新加载"

# 4. 立即修改当前 GPIO 文件权限
echo "[5/5] 修改当前 GPIO 文件权限..."
chmod 666 /sys/class/gpio/export /sys/class/gpio/unexport 2>/dev/null || true
for gpio in /sys/class/gpio/gpio*/; do
    if [ -d "$gpio" ]; then
        chmod 666 "${gpio}direction" 2>/dev/null || true
        chmod 666 "${gpio}value" 2>/dev/null || true
        chmod 666 "${gpio}edge" 2>/dev/null || true
    fi
done
echo "      GPIO 文件权限已修改"

echo ""
echo "=========================================="
echo "              配置完成!"
echo "=========================================="
echo ""
echo "注意: 如果仍有权限问题，请重新登录或重启系统"
echo "      使用户组变更生效"
echo ""
