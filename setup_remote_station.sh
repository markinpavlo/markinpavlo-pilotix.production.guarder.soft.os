#!/bin/bash
set -e

echo "=== Настройка станции управления Pilotix (ROS 2 Jazzy) ==="

# 1. Обновление и базовые утилиты
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales git curl wget python3-pip

# 2. Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Репозиторий ROS 2 Jazzy
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Установка ROS 2 и инструментов
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard

# 5. Подготовка рабочего пространства
mkdir -p ~/rover_remote_ws/src
cd ~/rover_remote_ws/src
if [ ! -d "vesc" ]; then
    git clone -b ros2 https://github.com/f1tenth/vesc.git
    find vesc -mindepth 1 -maxdepth 1 ! -name 'vesc_msgs' -exec rm -rf {} +
fi

# 6. Сборка
cd ~/rover_remote_ws
source /opt/ros/jazzy/setup.bash
[ ! -f /etc/apt/sources.list.d/20-default.list ] && sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-select vesc_msgs

# 7. СОЗДАНИЕ LAUNCH-ФАЙЛА (Запуск всего одной командой)
mkdir -p ~/rover_remote_ws/launch
cat <<EOF > ~/rover_remote_ws/launch/control_station.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Запуск RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        # Запуск Телеуправления (в новом окне терминала, так как нужен ввод с клавиатуры)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e', # Требуется установка xterm для выноса клавиатуры в отдельное окно
            output='screen'
        )
    ])
EOF

# 8. Установка xterm (нужен для выноса телеопа в отдельное окно)
sudo apt install -y xterm

# 9. Настройка .bashrc и создание алиаса 'rover_control'
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ~/rover_remote_ws/install/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    # Создаем короткую команду для запуска
    echo "alias rover_control='ros2 launch ~/rover_remote_ws/launch/control_station.launch.py'" >> ~/.bashrc
fi

echo "=== Настройка завершена! ==="
echo "Для запуска управления просто введите команду: rover_control"
