#!/bin/bash
set -e

echo "=== Полная настройка Pilotix Rover (ROS 2 Jazzy + XFCE Autostart) ==="

# 1. Системные обновления
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales git curl wget python3-pip python3-colcon-common-extensions xterm

# 2. Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2.5 Настройка X11, LightDM и XFCE
export DEBIAN_FRONTEND=noninteractive
echo "lightdm shared/default-x-display-manager select lightdm" | sudo debconf-set-selections
sudo apt install -y xfce4 xfce4-goodies lightdm
sudo systemctl disable gdm3 || true
sudo systemctl enable lightdm
sudo update-alternatives --set x-session-manager /usr/bin/startxfce4

# 3. Репозиторий ROS 2 Jazzy
sudo install -m 0755 -d /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Установка ROS Jazzy и ВСЕХ необходимых компонентов
sudo apt update
sudo apt install -y ros-jazzy-ros-core ros-dev-tools \
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
[ ! -f /etc/apt/sources.list.d/20-default.list ] && sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# 7. Python зависимости (Hardware)
pip install smbus2 VL53L1X pyvesc --break-system-packages

# 8. Права доступа (USB и I2C)
sudo usermod -a -G dialout,i2c $USER
echo 'KERNEL=="ttyACM[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-vesc.rules
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-vesc.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 9. Сборка
colcon build --symlink-install

# 10. Настройка .bashrc
grep -q "ROS_DOMAIN_ID=0" ~/.bashrc || echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc || echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 11. Настройка автостарта в XFCE
mkdir -p ~/.config/autostart
cat <<EOF > ~/.config/autostart/rover_app.desktop
[Desktop Entry]
Type=Application
Name=Rover Autostart
Exec=xterm -hold -e "bash -c 'source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=0; ros2 launch my_rover everything.launch.py'"
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
chmod +x ~/.config/autostart/rover_app.desktop

echo "=== Настройка завершена успешно. Перезагрузите систему. ==="
