#!/bin/bash
set -e

echo "=== Настройка Pilotix Rover (Ubuntu 24.04 Server + ROS 2 Jazzy + Systemd) ==="

# 1. Системные обновления
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales git curl wget python3-pip python3-colcon-common-extensions python3-rosdep2

# 2. Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Репозиторий ROS 2 Jazzy (Noble Numbat)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Установка компонентов ROS Jazzy
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-asio-cmake-module \
  ros-jazzy-io-context \
  ros-jazzy-teleop-twist-keyboard

# 5. Подготовка Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
[ ! -d "pilotix_rover" ] && git clone https://github.com/eugenediatlov/pilotix.production.guarder.soft.main pilotix_rover
[ ! -d "vesc" ] && git clone -b ros2 https://github.com/f1tenth/vesc.git

# 6. Зависимости через rosdep
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# 7. Python зависимости (Hardware)
pip install smbus2 VL53L1X pyvesc --break-system-packages

# 8. Права доступа (USB и I2C)
sudo usermod -a -G dialout,i2c $USER
echo 'KERNEL=="ttyACM[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-vesc.rules
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-vesc.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 9. Сборка (Оптимизировано для Raspberry Pi 5)
colcon build --symlink-install --parallel-workers 4

# 10. Настройка .bashrc
grep -q "ROS_DOMAIN_ID=0" ~/.bashrc || echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 11. Создание Systemd сервиса для автостарта
sudo tee /etc/systemd/system/rover.service > /dev/null <<EOF
[Unit]
Description=Pilotix Rover ROS2 Launch Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$HOME/ros2_ws
Environment="PYTHONUNBUFFERED=1"
Environment="ROS_DOMAIN_ID=0"
Environment="HOME=$HOME"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source $HOME/ros2_ws/install/setup.bash && ros2 launch my_rover everything.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Активация сервиса
sudo systemctl daemon-reload
sudo systemctl enable rover.service

echo "=== Настройка завершена. Перезагрузагрузка через 5 секунд. ==="
sleep 5
sudo reboot
