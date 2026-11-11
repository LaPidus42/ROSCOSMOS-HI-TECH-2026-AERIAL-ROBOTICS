# VTOL с системой обнаружения пожаров “Вега”
# Ссылка на паспорт проекта - https://docs.google.com/document/d/1ulLIB_YMaG4zhtzfm-pvwOxbdowuhzrmpYwXogpDT6c/edit?usp=sharing


**Назначение:** Мониторинг местности на предмет пожаров, детекция и передача координат на НСУ

VTOL-квадроплан, оснащенный вычислительным модулем на базе микрокомпьютера Orange Pi 5 для задач машинного зрения, а также бортовым вычислителем «Меркурий» с интегрированным радиомодемом, который обеспечивает передачу видеопотока с тепловизора и камеры на наземную станцию.

[Инструкция по установке и эксплуатации](INSTALLATION.md) — подключение оборудования, установка ПО, запуск детекции и веб-интерфейса, завершение работы.

## Структура

| Каталог | Содержимое |
|---|---|
| [`fire_detection_ros_rknn/`](fire_detection_ros_rknn/) | ROS-пакет: детекция огня на NPU, геопривязка, журнал, веб-интерфейс НСУ |
| [`radio/`](radio/) | сервисы радиомодема: трансляция видео с тепловизора и камеры |

## Зависимости

- Orange Pi 5 (RK3588), Ubuntu 20.04, ROS Noetic
- RKNN Toolkit Lite 2 (`rknn-lite`), драйвер NPU ≥ 0.9
- Python 3.8+: `pymavlink`, `pyserial`, `numpy`, `opencv-python`
- ROS: `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`

## Установка

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/LaPidus42/ROSCOSMOS-HI-TECH-2026-AERIAL-ROBOTICS.git
ln -s ROSCOSMOS-HI-TECH-2026-AERIAL-ROBOTICS/fire_detection_ros_rknn .

pip3 install -r fire_detection_ros_rknn/requirements.txt
cp <модель>.rknn fire_detection_ros_rknn/models/

cd ~/catkin_ws && rosdep install --from-paths src -y --ignore-src
catkin_make && source devel/setup.bash
```

## Запуск

```bash
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=vtol
```

Для Техника 6s:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=test
```

Интерфейс НСУ: `http://<IP Orange Pi>:8080`. Проверка без дрона: `python3 fire_detection_ros_rknn/tools/simulate.py`.

Подробнее: [дашборд](fire_detection_ros_rknn/README_DASHBOARD.md), [радиомодемы](radio/README.md).
