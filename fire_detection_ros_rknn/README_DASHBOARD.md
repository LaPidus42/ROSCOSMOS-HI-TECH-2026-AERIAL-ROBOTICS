# Веб-интерфейс: координаты объектов, карта или план, журнал, видео

## Два режима

| | ВТОЛ (`vehicle:=vtol`) | Тестовый дрон (`vehicle:=test`) |
|---|---|---|
| Навигация | GPS | ArUco-маркеры, без GPS |
| Координаты объектов | широта/долгота | X/Y в метрах на плане площадки |
| Подложка | карта местности (схема/спутник/офлайн-тайлы) | загруженное изображение площадки + карта маркеров + сетка |
| Конфиг | `config/dashboard_vtol.yaml` | `config/dashboard_test.yaml` |

```bash
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=vtol
roslaunch fire_detection_ros_rknn fire_system.launch vehicle:=test
```

Интерфейс: ВТОЛ — `http://<IP Orange Pi>:8080`, тестовый дрон — `http://10.42.0.1:8088`.

## Что добавлено

```
scripts/geo_dashboard_node.py   ROS-нода: топики -> координаты -> журнал -> веб
scripts/geo_core.py             геометрия, трекер, SQLite, план площадки, HTTP-сервер (без ROS)
scripts/img_convert.py          sensor_msgs/Image -> JPEG для курсовой камеры (без cv_bridge)
scripts/mavlink_telemetry_node.py  + /mavlink/rel_alt, /mavlink/gps_info, /mavlink/local_position
web/                            интерфейс (Leaflet лежит локально)
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
- веб-интерфейс на порту **8088** (8080 занят `web_video_server`):
  `http://10.42.0.1:8088` при подключении к Wi-Fi дрона;
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

Карта маркеров — `map33.txt` из пакета aruco_pose; во вкладке «План» её вместе с
изображением забирает кнопка «Взять карту с дрона».

Другие источники позиции (`local/source`): `pose` — любой топик позы,
`mavlink` — /mavlink/local_position от нашей mavlink-ноды на аппаратах без MAVROS.

## План площадки (вкладка «План», только тестовый режим)

- **«Взять карту с дрона»** — одна кнопка: изображение из `/aruco_map/image` и файл карты
  из параметра `/aruco_map/map` (у вас `map33.txt`). Изображение сразу выставляется по
  маркерам: узел aruco_map рисует карту 2000×2000 пикс. с полями 200 пикс., вписывая
  габарит маркеров. Проверка: нарисованные поверх маркеры должны лечь на маркеры
  изображения. Если не легли, поменяйте «поля изображения» и нажмите «Подогнать по маркерам».
- **Вручную**: любое изображение (схема зала, фото сверху) — задаются ширина в метрах и
  левый нижний угол, изменения видны сразу и сохраняются на Orange Pi; карта маркеров —
  текстовый файл формата aruco_pose (`id размер x y z rot_z rot_y rot_x`).

## Видео

Раскрывающиеся панели справа: тепловизор с рамками (`/fire/annotated`) и камеры из списка
`cameras` в конфиге аппарата. На тестовом дроне это основная камера (вниз, по ней работают
ArUco и оптический поток) и вторая камера. Поддерживаются `sensor_msgs/Image`
(bgr8, rgb8, mono8/16, yuv422, bayer) и `CompressedImage`. Нода подписывается на топик,
только пока панель открыта; кнопка «Крупно» разворачивает видео поверх карты.

## Обязательно настроить

- Углы обзора тепловизора `camera/hfov_deg`, `camera/vfov_deg` и крепление камеры —
  отдельно для каждого аппарата.
- На тестовом дроне `tracker/assoc_radius_m` должен быть меньше расстояния между очагами.
- Проверка точности на тестовой площадке: положите источник тепла в точку с известными
  координатами и сравните с координатами объекта в интерфейсе.

## Журнал

SQLite `~/fire_logs/detections.db`, общий для обоих аппаратов: у каждой записи есть
режим (`gps`/`local`) и номер полёта. CSV из вкладки «Журнал» содержит и lat/lon, и X/Y.

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
В локальном режиме файлы поля маркеров кладутся в `tools/sim_aruco/` — их можно
загрузить во вкладке «План» вручную.
