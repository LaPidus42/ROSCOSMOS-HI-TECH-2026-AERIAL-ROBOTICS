# Модемы

| | IP | SSH |
|---|---|---|
| Борт | 192.168.1.15 | `ssh root@192.168.1.15` (только пароль) |
| Наземка | 192.168.1.14 | `ssh root@192.168.1.14` (только пароль) |

## Видео с борта

Борт шлёт H.264 RTP/UDP на `192.168.1.100:5600`, у компьютера должен быть этот IP.

### Настройка качества

```bash
ssh root@192.168.1.15 usbcam-preset stable   # max | stable | min
```

| Пресет | Разрешение | Битрейт | Когда |
|---|---|---|---|
| `max` | 1600x1200@30 | 5 Мбит/с | хорошая связь |
| `stable` | 1280x720@30 | 2.5 Мбит/с | по умолчанию |
| `min` | 640x480@30 | 0.8 Мбит/с | слабая связь, дальность |

`usbcam-preset` без аргумента показывает текущие параметры. Ручная настройка: `/etc/default/usbcam-stream`, затем `systemctl restart usbcam-stream`.

### Просмотр

Из любой директории (Linux и macOS; на маке сначала `brew install ffmpeg`):
```bash
printf 'v=0\no=- 0 0 IN IP4 0.0.0.0\ns=cam\nc=IN IP4 0.0.0.0\nt=0 0\nm=video 5600 RTP/AVP 96\na=rtpmap:96 H264/90000\n' | ffplay -fflags nobuffer -flags low_delay -framedrop -f sdp -protocol_whitelist fd,pipe,udp,rtp -i -
```

Или через gstreamer (на маке `brew install gstreamer`):
```bash
gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" ! rtpjitterbuffer latency=50 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

С файлом: `ffplay -fflags nobuffer -flags low_delay -framedrop -protocol_whitelist file,udp,rtp -i board-camera.sdp` или открыть `board-camera.sdp` в VLC.

На macOS при первом запуске разрешить входящие подключения в файрволе.

Если видео встало (камера иногда замолкает без отключения USB): `ssh root@192.168.1.15 systemctl restart usbcam-stream`.

## Видео с тепловизора (детекция)

Борт шлёт H.264 RTP/UDP на `192.168.1.100:5601`: 640x480, 10 к/с, 600 кбит/с. Источник: ROS-топик `/fire/annotated` (тепловизор с рамками детекции) на плате детекции Orange Pi `192.168.1.20`. Сервис `thermal-stream` на модеме забирает его через rosbridge (`ws://192.168.1.20:9090`) и кодирует в H.264. Сам детектор выдаёт ~2 к/с, поэтому между новыми кадрами повторяется последний.

Просмотр:
```bash
printf 'v=0\no=- 0 0 IN IP4 0.0.0.0\ns=thermal\nc=IN IP4 0.0.0.0\nt=0 0\nm=video 5601 RTP/AVP 96\na=rtpmap:96 H264/90000\n' | ffplay -fflags nobuffer -flags low_delay -framedrop -f sdp -protocol_whitelist fd,pipe,udp,rtp -i -
```
Или `ffplay -fflags nobuffer -flags low_delay -framedrop -protocol_whitelist file,udp,rtp -i board-thermal.sdp`, или gstreamer-командой выше с `port=5601`. Первую секунду ffplay пишет `non-existing PPS 0 referenced`: он ждёт ключевой кадр, это нормально.

Настройки в `/etc/default/thermal-stream` (топик, адрес rosbridge, порт ПК, FPS, битрейт), затем `systemctl restart thermal-stream`. Лог: `journalctl -u thermal-stream -f`, раз в минуту пишет фактическую частоту топика.

Если картинка застыла: детекция на Orange Pi не запущена или топик не публикуется (в логе `topic rate 0.0 Hz`). Если пропала связь с rosbridge, скрипт сам переподключается.

