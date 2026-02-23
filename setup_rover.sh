#!/bin/bash
set -e

echo "=== Начинаю полную настройку системы Pilotix Rover (ROS 2 Jazzy) ==="

# 1. Обновление системы и установка базовых утилит
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales git curl wget python3-pip

# 2. Настройка локали
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Добавление репозитория ROS 2 Jazzy
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Установка ROS 2 Jazzy Core и инструментов сборки
sudo apt update
sudo apt install -y ros-jazzy-ros-core ros-dev-tools python3-colcon-common-extensions

# 5. Инициализация rosdep
if [ ! -f /etc/apt/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 6. Подготовка рабочего пространства
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Клонирование основного репозитория
if [ ! -d "pilotix_rover" ]; then
    git clone https://github.com/eugenediatlov/pilotix.production.guarder.soft.main pilotix_rover
fi

# Клонирование ветки ROS 2 для VESC
if [ ! -d "vesc" ]; then
    git clone -b ros2 https://github.com/f1tenth/vesc.git
fi

# 7. Установка зависимостей через rosdep
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# 8. Установка специфичных зависимостей для сборки vesc_driver
sudo apt install -y ros-jazzy-asio-cmake-module ros-jazzy-io-context

# 9. Установка Python-библиотек для железа
pip install pyvesc VL53L1X smbus2 --break-system-packages --user

# 10. Настройка прав доступа (udev rules) для VESC и последовательных портов
sudo usermod -a -G dialout $USER
# Создание правила для VESC (обычно отображается как /dev/ttyACM0 или /dev/ttyUSB0)
echo 'KERNEL=="ttyACM[0-9]*", MODE="0666"' | sudo tee /etc/apt/../udev/rules.d/99-vesc.rules
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee -a /etc/apt/../udev/rules.d/99-vesc.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 11. Сборка проекта
colcon build --symlink-install

# 12. Автоматизация окружения (добавление в .bashrc)
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# Добавление ROS_DOMAIN_ID
if ! grep -q "export ROS_DOMAIN_ID=0" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
fi

echo "=== Настройка завершена успешно! ==="
echo "ВАЖНО: Выйдите из системы и войдите снова (или перезагрузитесь), чтобы права групп вступили в силу."
echo "Затем выполните: source ~/.bashrc"
