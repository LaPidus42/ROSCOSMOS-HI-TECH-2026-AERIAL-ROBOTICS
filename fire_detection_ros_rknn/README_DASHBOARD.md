# Координаты объектов и журнал обнаружений

## Два режима

| | ВТОЛ (`vehicle:=vtol`) | Тестовый дрон (`vehicle:=test`) |
|---|---|---|
| Навигация | GPS | ArUco-маркеры, без GPS |
| Координаты объектов | широта/долгота | X/Y в метрах на плане площадки |
| Конфиг | `config/dashboard_vtol.yaml` | `config/dashboard_test.yaml` |

```bash
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=vtol
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=test
```

## Что добавлено

```
scripts/geo_dashboard_node.py   ROS-нода: топики -> координаты -> журнал -> API
scripts/geo_core.py             геометрия, трекер, SQLite, план площадки, HTTP-сервер (без ROS)
scripts/img_convert.py          sensor_msgs/Image -> JPEG для курсовой камеры (без cv_bridge)
scripts/mavlink_telemetry_node.py  + /mavlink/rel_alt, /mavlink/gps_info, /mavlink/local_position
config/dashboard_vtol.yaml, config/dashboard_test.yaml
tools/simulate.py               демонстрация обоих режимов без ROS и дрона
```

## Тестовый дрон (образ Technic / Clover)

На дроне уже работают MAVROS, `aruco_detect`, `aruco_map`, `vpe_publisher`, `web_video_server`.
Поэтому при `vehicle:=test`:

- `mavlink_telemetry_node` **не запускается**: последовательный порт автопилота держит MAVROS;
- позиция берётся из TF `aruco_map` → `base_link` (система карты маркеров = система плана),
  через фрейм `map` автопилота, так что она есть и когда маркеры ненадолго пропадают;
- «сколько секунд без маркеров» — по топику `/aruco_map/pose`;
- скорость — `/mavros/local_position/velocity_local`;
- HTTP-сервер на порту **8088** (8080 занят `web_video_server`);
- детектору огня отдаётся одно ядро NPU (остальные занимает `rknn_face`), Foxglove не запускается.

Установка на дрон:

```bash
cp -r fire_detection_ros_rknn ~/technic_ws/src/
cp <модель>.rknn ~/technic_ws/src/fire_detection_ros_rknn/models/
cd ~/technic_ws && catkin_make && source devel/setup.bash   # или catkin build, как принято в образе
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=test
```

Проверка, что позиция доступна (дрон на площадке, маркеры в кадре):

```bash
rosrun tf tf_echo aruco_map base_link
```

И что тепловизор найден: `v4l2-ctl --list-devices` — параметр `device_search_id` в launch
задаёт USB-порт камеры, на Orange Pi 5 Pro он может отличаться от ВТОЛ.

Другие источники позиции (`local/source`): `pose` — любой топик позы,
`mavlink` — /mavlink/local_position от нашей mavlink-ноды на аппаратах без MAVROS.

## Обязательно настроить

- Углы обзора тепловизора `camera/hfov_deg`, `camera/vfov_deg` и крепление камеры —
  отдельно для каждого аппарата.
- На тестовом дроне `tracker/assoc_radius_m` должен быть меньше расстояния между очагами.
- Проверка точности на тестовой площадке: положите источник тепла в точку с известными
  координатами и сравните с координатами объекта в журнале.

## Журнал

SQLite `~/fire_logs/detections.db`, общий для обоих аппаратов: у каждой записи есть
режим (`gps`/`local`) и номер полёта. CSV (`/api/export.csv`) содержит и lat/lon, и X/Y.

## Адреса API

| Адрес | Что отдаёт |
|---|---|
| `/api/stream` | события `telemetry`, `detection`, `object`, `localmap` (SSE) |
| `/api/objects`, `/api/events`, `/api/export.csv`, `/api/sessions` | объекты и журнал |
| `/api/localmap` (GET), `/api/localmap/{image,config,markers,fit,from_ros}` (POST) | план площадки |
| `/video/thermal.mjpg`, `/video/course.mjpg`, `/api/video_status` | видео |

Сервер без авторизации: рассчитан на закрытую сеть борта и наземной станции.

## Проверка без дрона

```bash
python3 tools/simulate.py                  # ВТОЛ
python3 tools/simulate.py --mode local     # тестовый дрон над полем ArUco
```
В локальном режиме файлы поля маркеров кладутся в `tools/sim_aruco/`.
