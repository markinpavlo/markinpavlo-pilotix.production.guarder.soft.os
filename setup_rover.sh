#!/bin/bash

# Цвета для лога
GREEN='\033[0;32m'
NC='\033[0m'

# Функция для отрисовки прогресса внизу экрана
draw_progress() {
    local progress=$1
    local task=$2
    local width=40
    local filled=$(( progress * width / 100 ))
    tput sc
    tput cup $(tput lines) 0
    printf "\e[44m\e[97m [%-${width}s] %d%% | %s \e[0m" "$(printf "%${filled}s" | tr ' ' '#')" "$progress" "$task"
    tput rc
}

set -e

# --- ПАРАМЕТРЫ ---
INTERFACE="eth0"         
FIXED_IP="1.2.3.4/24"
HOSTNAME="pilotix"
STAGES=13
CURRENT_STAGE=0

step() {
    CURRENT_STAGE=$((CURRENT_STAGE + 1))
    local percent=$(( CURRENT_STAGE * 100 / STAGES ))
    echo -e "${GREEN}>>> [${CURRENT_STAGE}/${STAGES}] $1${NC}"
    draw_progress "$percent" "$1"
}

echo "=== Запуск установки Pilotix Rover (Ubuntu 24.04 Server) ==="

# 0. Имя хоста и DNS
step "Настройка сетевого имени $HOSTNAME..."
sudo hostnamectl set-hostname $HOSTNAME
sudo sed -i "s/127.0.1.1.*/127.0.1.1 $HOSTNAME/" /etc/hosts
sudo sed -i 's/#DNS=/DNS=8.8.8.8 1.1.1.1/' /etc/systemd/resolved.conf || true
sudo systemctl restart systemd-resolved || true

# 1. Сеть (DHCP + Аварийный статический IP)
step "Конфигурация Netplan..."
sudo tee /etc/netplan/99-hybrid-network.yaml > /dev/null <<EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    $INTERFACE:
      dhcp4: true
      addresses:
        - $FIXED_IP
EOF
sudo chmod 600 /etc/netplan/99-hybrid-network.yaml
sudo netplan apply || true

# 2. Локаль
step "Настройка локали..."
sudo apt update
sudo apt install -y locales curl gnupg avahi-daemon
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Репозиторий ROS 2 Jazzy
step "Подключение репозитория ROS 2..."
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Инструменты сборки
step "Установка инструментов разработки..."
sudo apt update
sudo apt install -y ros-dev-tools python3-colcon-common-extensions

# 5. Компоненты ROS
step "Установка пакетов ROS Jazzy..."
sudo apt install -y ros-jazzy-ros-base \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-asio-cmake-module \
  ros-jazzy-io-context \
  ros-jazzy-teleop-twist-keyboard

# 6. Workspace
step "Клонирование репозиториев..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
[ ! -d "pilotix_rover" ] && git clone https://github.com/eugenediatlov/pilotix.production.guarder.soft.main pilotix_rover
[ ! -d "vesc" ] && git clone -b ros2 https://github.com/f1tenth/vesc.git
cd ~/ros2_ws

# 7. Rosdep
step "Обновление зависимостей через rosdep..."
source /opt/ros/jazzy/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# 8. Python библиотеки
step "Установка Python пакетов (Hardware)..."
pip install smbus2 VL53L1X pyvesc --break-system-packages

# 9. Права и Udev
step "Настройка прав доступа USB/I2C..."
sudo usermod -a -G dialout,i2c $USER
echo 'KERNEL=="ttyACM[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-vesc.rules
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-vesc.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 10. Сборка
step "Сборка проекта (colcon build)..."
colcon build --symlink-install --parallel-workers 4

# 11. Настройка .bashrc и вывод IP
step "Настройка окружения .bashrc..."
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
# Добавляем вывод IP при логине
grep -q "hostname -I" ~/.bashrc || echo 'echo -e "\n${GREEN}Pilotix Rover IP: $(hostname -I)${NC}\n"' >> ~/.bashrc

# 12. Сервис автостарта
step "Активация rover.service..."
sudo tee /etc/systemd/system/rover.service > /dev/null <<EOF
[Unit]
Description=Pilotix Rover ROS2 Launch
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

sudo systemctl daemon-reload
sudo systemctl enable rover.service

# Очистка строки прогресса
tput cup $(tput lines) 0
printf "\033[K"

echo -e "\n${GREEN}=== Установка завершена успешно! ===${NC}"
echo "Для подключения с любого ПК в сети используйте:"
echo "ssh $USER@$HOSTNAME.local"
echo "Перезагрузите систему для активации всех прав: sudo reboot"
