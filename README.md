# Узлы ROS2 для управления БПЛА

## Установка необходимого ПО
    
### 1) Установка ROS2

	sudo apt update && sudo apt install locales
	sudo locale-gen en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8
	
	sudo apt install -y software-properties-common
	sudo add-apt-repository universe
	
	
	sudo apt update && sudo apt install curl -y
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	
	sudo apt update
	sudo apt upgrade
	
	sudo apt install -y ros-humble-desktop

### 2) Установка сборщика проектов colcon:

	sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update
	sudo apt install -y python3-colcon-common-extensions

### 3) Установка rknn-toolkit-lite2

	sudo apt-get update
	sudo apt-get install -y python3 python3-dev python3-pip gcc
	sudo apt-get install -y python3-opencv
	sudo apt-get install -y python3-numpy
	sudo apt-get install git
	sudo apt-get install wget
	sudo apt-get install python3-setuptools
	wget https://github.com/rockchip-linux/rknpu2/raw/master/runtime/RK356X/Linux/librknn_api/aarch64/librknnrt.so
	sudo mv librknnrt.so /usr/lib/librknnrt.so
	cd ~ && git clone https://github.com/rockchip-linux/rknn-toolkit2.git
	cd rknn-toolkit2/rknn_toolkit_lite2/packages/ && pip3 install rknn_toolkit_lite2-1.5.2-cp310-cp310-linux_aarch64.whl


## Установка узлов

### 1) Создаём рабочее пространство ROS и клонируем узлы в него

	cd ~ && mkdir ros2_ws 
	git clone ${GIT_REPO} 

### 2) Устанавливаем узел с камерой из источников 

Открыть терминал и ввести:

	sudo apt-get install -y ros-${ROS_DISTRO}-v4l2-camera

## Настройка рабочего пространства

### 1) Настройка файла .bashrc

Настройка работы с ROS2 в терминале:

	echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc

Настройка работы с colcon с помощью сборщика в bashrc:

	echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
 
Настрока работы с рабочим пространством

	echo "source /home/$USER/ros2_ws/install/setup.bash" >> ~/.bashrc

### 2) Устранение неполадок

Для определения всех зависимостей в рабочем пространстве
	
	sudo apt install python3-rosdep2
	rosdep update
	rosdep install --from-paths src --ignore-src -r -y

*Исправление ошибки версий setuptools, заменив 59.6.0 на 58.2.0:*

	pip3 install setuptools==58.2.0

*Исправление ошибки не найденных зависимостей, при сборке*

	export AMENT_PREFIX_PATH=''

## Собираем узлы и запускаем их

	source ~/.bashrc
	cd ~/ros2_ws && colcon build
	ros2 launch detector detection_simple.launch.py

