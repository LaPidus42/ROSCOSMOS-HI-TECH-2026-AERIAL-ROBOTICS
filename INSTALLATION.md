# Установка и запуск комплекса обнаружения пожаров

## 1. Подключение оборудования

1. При отключённом питании подключите к Orange Pi тепловизор и курсовую камеру по USB. Используйте те же USB-разъёмы при каждом запуске.
2. Подключите Orange Pi к бортовой сети Ethernet. При необходимости подключите автопилот к UART-разъёму MAVLink.
3. Подключите штатный блок питания Orange Pi и включите питание. Дождитесь загрузки системы.

## 2. Установка ПО

На Orange Pi с Ubuntu 20.04 и ROS Noetic выполните:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LaPidus42/ROSCOSMOS-HI-TECH-2026-AERIAL-ROBOTICS.git
ln -s ROSCOSMOS-HI-TECH-2026-AERIAL-ROBOTICS/fire_detection_ros_rknn fire_detection_ros_rknn_slop2
pip3 install -r fire_detection_ros_rknn_slop2/requirements.txt
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

Модель детекции уже находится в каталоге `fire_detection_ros_rknn_slop2/models/`.

## 3. Запуск и проверка

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch fire_detection_ros_rknn_slop2 fire_system2.launch vehicle:=vtol
```

На компьютере оператора откройте в браузере `http://<IP-адрес Orange Pi>:8080`. Установка выполнена правильно, если в интерфейсе отображаются оба видеопотока, а обнаруженные объекты выделяются рамками и появляются на карте и в журнале.

Для остановки нажмите `Ctrl+C`, выполните `sudo poweroff`, дождитесь выключения Orange Pi и только затем отключите питание.
