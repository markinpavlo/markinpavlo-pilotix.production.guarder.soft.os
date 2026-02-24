#!/bin/bash
set -e

echo "=== Настройка удаленной станции управления Pilotix (ROS 2 Jazzy) ==="

# 1. Обновление и базовые утилиты
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales git curl wget python3-pip python3-colcon-common-extensions xterm

# 2. Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Репозиторий ROS 2 Jazzy
sudo install -m 0755 -d /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Установка ROS Jazzy и пакетов визуализации/управления
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher

# 5. Подготовка рабочего пространства (Workspace)
mkdir -p ~/rover_remote_ws/src
cd ~/rover_remote_ws/src

# Клонируем ваш проект и сообщения vesc для корректного отображения в RViz
[ ! -d "pilotix_rover" ] && git clone https://github.com/eugenediatlov/pilotix.production.guarder.soft.main pilotix_rover
[ ! -d "vesc" ] && git clone -b ros2 https://github.com/f1tenth/vesc.git

# 6. Установка зависимостей через rosdep
cd ~/rover_remote_ws
source /opt/ros/jazzy/setup.bash
[ ! -f /etc/apt/sources.list.d/20-default.list ] && sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# 7. Сборка пакетов (на удаленном ПК собираем всё для запуска launch-файлов)
colcon build --symlink-install

# 8. Настройка .bashrc
grep -q "ROS_DOMAIN_ID=0" ~/.bashrc || echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "source ~/rover_remote_ws/install/setup.bash" ~/.bashrc || echo "source ~/rover_remote_ws/install/setup.bash" >> ~/.bashrc

# 9. Создание алиаса для быстрого запуска управления
if ! grep -q "alias rover_control" ~/.bashrc; then
    echo "alias rover_control='ros2 launch my_rover everything.launch.py'" >> ~/.bashrc
fi

echo "=== Настройка удаленной станции завершена! ==="
echo "Перезапустите терминал и используйте команду: rover_control"
