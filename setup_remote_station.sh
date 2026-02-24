#!/bin/bash
set -e

echo "=== Настройка удаленной станции управления (Ubuntu 24.04) ==="

# 1. Обновление и установка необходимых утилит
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales git curl wget python3-pip xterm wmctrl

# 2. Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Репозиторий ROS 2 Jazzy
sudo install -m 0755 -d /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Установка пакетов ROS 2 и зависимостей
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher

# 5. Скачивание конфигурационного файла RViz
# Используем прямую ссылку на raw-файл для скачивания через wget
RVIZ_CONFIG_URL="https://raw.githubusercontent.com/eugenediatlov/pilotix.production.guarder.soft.os/main/partizan_rover.rviz"
wget -O ~/partizan_rover.rviz $RVIZ_CONFIG_URL
echo "Файл конфигурации RViz скачан в домашнюю директорию."

# 6. Очистка .bashrc от старых путей (Humble/Turtlebot)
sed -i '/humble/d' ~/.bashrc
sed -i '/turtlebot/d' ~/.bashrc

# 7. Настройка окружения в .bashrc
grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "ROS_DOMAIN_ID=0" ~/.bashrc || echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# 8. Создание исправленного алиаса rover_control
# Используется логика: запуск RViz в фоне, пауза 4с, затем xterm на передний план
# 8. Создание исправленного алиаса rover_control
sed -i '/alias rover_control/d' ~/.bashrc

# ВАЖНО: Команда после -e идет БЕЗ внутренних кавычек
echo "alias rover_control='rviz2 -d ~/partizan_rover.rviz > /dev/null 2>&1 & (sleep 4 && xterm -geometry 90x25+0+0 -T \"ROVER_CONTROL\" -e ros2 run teleop_twist_keyboard teleop_twist_keyboard & sleep 2 && wmctrl -a \"ROVER_CONTROL\" -b add,above) &'" >> ~/.bashrc

echo "=== Настройка завершена! ==="
echo "Для запуска управления введите: rover_control"
