#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Симулятор для проверки веб-интерфейса без ROS, дрона и камер.

  python3 tools/simulate.py                 # ВТОЛ: полёт по GPS, карта местности
  python3 tools/simulate.py --mode local    # тестовый дрон над полем ArUco, план в метрах
  открыть http://localhost:8080

В локальном режиме симулятор создаёт поле маркеров (изображение + map.txt
в формате Clover) в tools/sim_aruco/ и сразу подставляет его как план.
Эти же файлы можно загрузить вручную во вкладке «План», чтобы попробовать.

Для видео нужен OpenCV (pip install opencv-python), без него всё остальное работает.
"""
import argparse
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

from geo_core import (DashboardCore, DashboardServer, EventStore, Geolocator,  # noqa: E402
                      LocalMapStore, ObjectTracker, _mv, _rot_zyx, offset_latlon)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

W, H = 640, 480


def _t(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def project(geo, tel, north, east):
    """Смещение точки от дрона (м, NED) -> пиксель или None, если вне кадра."""
    v = (north, east, tel['agl'])
    v = _mv(_t(_rot_zyx(math.radians(tel['roll']), math.radians(tel['pitch']), math.radians(tel['yaw']))), v)
    v = _mv(_t(geo.r_mount), v)
    v = _mv(_t(geo._OPT_TO_BODY), v)
    if v[2] <= 0:
        return None
    u = W / 2 + v[0] / v[2] / geo.tan_h * (W / 2) * geo.sx
    w = H / 2 + v[1] / v[2] / geo.tan_v * (H / 2) * geo.sy
    return (u, w) if 0 <= u < W and 0 <= w < H else None


# ---------------------------------------------------------------------------
# Поле ArUco (локальный режим)
# ---------------------------------------------------------------------------
def make_aruco_field(out_dir, nx=6, ny=5, size=0.33, dist=1.0, img=2000, margin=200):
    """
    Сетка маркеров DICT_4X4_250 с центрами в (i*dist, j*dist) и изображение карты,
    нарисованное как у узла aruco_map (drawPlanarBoard: габарит маркеров вписан
    в img×img за вычетом полей margin, ось Y вверх).
    -> (map_path, png_path, (x0, y0, width_m) — истинное положение изображения)
    """
    os.makedirs(out_dir, exist_ok=True)
    lines = ['# id length x y z rot_z rot_y rot_x']
    markers = []
    for j in range(ny):
        for i in range(nx):
            mid = j * nx + i
            markers.append((mid, i * dist, j * dist))
            lines.append('%d %.3f %.3f %.3f 0 0 0 0' % (mid, size, i * dist, j * dist))
    map_path = os.path.join(out_dir, 'map33.txt')
    with open(map_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    min_x, min_y = -size / 2, -size / 2
    bw, bh = (nx - 1) * dist + size, (ny - 1) * dist + size
    inner = img - 2 * margin
    scale = min(inner / bw, inner / bh)
    pad_x = margin + (inner - bw * scale) / 2
    pad_y = margin + (inner - bh * scale) / 2
    truth = (min_x - pad_x / scale, min_y - pad_y / scale, img / scale)

    png_path = os.path.join(out_dir, 'aruco_map_image.png')
    if cv2 is not None:
        canvas = np.full((img, img, 3), 255, np.uint8)
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        side = int(round(size * scale))
        for mid, mx, my in markers:
            m = cv2.aruco.generateImageMarker(dic, mid, side)
            u = int(round(pad_x + (mx - size / 2 - min_x) * scale))
            v = int(round(img - pad_y - (my + size / 2 - min_y) * scale))   # ось Y изображения вниз
            canvas[v:v + side, u:u + side] = m[..., None]
        # оси, как с image_axis=true
        ox, oy = int(round(pad_x - min_x * scale)), int(round(img - pad_y + min_y * scale))
        cv2.arrowedLine(canvas, (ox, oy), (ox + int(scale), oy), (0, 0, 255), 6, tipLength=0.15)
        cv2.arrowedLine(canvas, (ox, oy), (ox, oy - int(scale)), (0, 255, 0), 6, tipLength=0.15)
        cv2.imwrite(png_path, canvas)
    return map_path, png_path, truth


# ---------------------------------------------------------------------------
# Синтетическое видео
# ---------------------------------------------------------------------------
class FakeVideo:
    def __init__(self, field_png=None, field_meta=None):
        self.field = cv2.imread(field_png) if (cv2 is not None and field_png and os.path.exists(field_png)) else None
        self.meta = field_meta     # (x0, y0, width_m)

    def thermal(self, dets, colors):
        if cv2 is None:
            return None
        g = np.random.default_rng().normal(60, 6, (H // 4, W // 4)).astype(np.float32)
        g = cv2.resize(g, (W, H))
        for d in dets:
            cv2.circle(g, (int(d['cx']), int(d['cy'])), int(d['w'] / 3), 240 if d['class_name'] == 'fire' else 150, -1)
        img = cv2.applyColorMap(cv2.GaussianBlur(np.clip(g, 0, 255).astype(np.uint8), (0, 0), 6), cv2.COLORMAP_INFERNO)
        for d in dets:
            c = colors.get(d['class_name'], (255, 255, 255))
            x1, y1 = int(d['cx'] - d['w'] / 2), int(d['cy'] - d['h'] / 2)
            cv2.rectangle(img, (x1, y1), (x1 + int(d['w']), y1 + int(d['h'])), c, 2)
            cv2.putText(img, '%s %.2f' % (d['class_name'], d['conf']), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()

    def course(self, tel):
        """Вид вниз из «курсовой камеры» над изображением поля маркеров."""
        if cv2 is None:
            return None
        if self.field is None or self.meta is None or tel.get('x') is None:
            img = np.full((360, 640, 3), 90, np.uint8)
            cv2.putText(img, 'course camera (sim)', (170, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (220, 220, 220), 2)
            return cv2.imencode('.jpg', img)[1].tobytes()
        x0, y0, wm = self.meta
        hpx, wpx = self.field.shape[:2]
        ppm = wpx / wm
        cx = (tel['x'] - x0) * ppm
        cy = hpx - (tel['y'] - y0) * ppm
        half_w = tel['agl'] * math.tan(math.radians(35)) * ppm
        # поворот так, чтобы нос дрона смотрел вверх кадра
        M = cv2.getRotationMatrix2D((cx, cy), -tel['yaw'], 640 / (2 * half_w))
        M[0, 2] += 320 - cx
        M[1, 2] += 180 - cy
        view = cv2.warpAffine(self.field, M, (640, 360), borderValue=(60, 60, 60))
        return cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('gps', 'local'), default='gps')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--lat', type=float, default=55.7512)
    ap.add_argument('--lon', type=float, default=37.6184)
    ap.add_argument('--agl', type=float, default=None, help='высота, м (по умолчанию 80 для gps, 2 для local)')
    ap.add_argument('--speed', type=float, default=None, help='м/с (по умолчанию 10 для gps, 0.6 для local)')
    ap.add_argument('--db', default=os.path.join(HERE, 'sim_detections.db'))
    args = ap.parse_args()
    local = args.mode == 'local'
    agl = args.agl or (2.0 if local else 80.0)
    speed = args.speed or (0.6 if local else 10.0)

    geo = Geolocator(mode=args.mode, hfov_deg=56, vfov_deg=42, mount_pitch_deg=-90,
                     min_agl_m=0.3 if local else 3.0, max_range_m=30 if local else 1500)
    store = EventStore(args.db, time.strftime('%Y%m%d_%H%M%S'), mode=args.mode)
    tracker = ObjectTracker(store, mode=args.mode, assoc_radius_m=0.4 if local else 20.0)
    core = DashboardCore(geo, store, tracker, video_streams=('thermal', 'course'))
    ui = {
        'vehicle': 'Тестовый дрон (симуляция)' if local else 'ВТОЛ (симуляция)',
        'tile_url': 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
        'tile_attribution': '© OpenStreetMap contributors', 'tile_max_zoom': 19,
        'satellite_url': 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        'center': [args.lat, args.lon], 'zoom': 17,
        'class_labels': {'fire': 'Очаг возгорания', 'car': 'Автомобиль'},
        'class_colors': {'fire': '#ff5a36', 'car': '#f2c230'},
        'stale_after_s': 60,
        'video_labels': {'thermal': 'Тепловизор с детекциями', 'course': 'Курсовая камера'},
    }

    localmap, field_meta, png = None, None, None
    rnd = random.Random(7)
    if local:
        localmap = LocalMapStore(os.path.join(HERE, 'sim_localmap'))
        map_path, png, field_meta = make_aruco_field(os.path.join(HERE, 'sim_aruco'))

        def ros_loader():            # то же, что делает нода: /aruco_map/image + файл карты
            data = open(png, 'rb').read()
            from geo_core import image_size
            w, h = image_size(data)
            return data, w, h, 'image/png', open(map_path).read(), os.path.basename(map_path)
        if os.path.exists(png):
            localmap.ros_loader = ros_loader
        # цели в координатах плана (x, y)
        truth = [('fire', 1.3, 2.6), ('fire', 3.7, 0.8), ('fire', 4.4, 3.4), ('car', 2.2, 1.4)]
        # «змейка» над полем 0..5 x 0..4 м
        wps = []
        for k, x in enumerate([0.0, 1.25, 2.5, 3.75, 5.0]):
            wps += [(x, 0.0), (x, 4.0)] if k % 2 == 0 else [(x, 4.0), (x, 0.0)]
    else:
        truth = [('fire' if i < 4 else 'car',) + offset_latlon(args.lat, args.lon, rnd.uniform(-250, 250),
                                                                rnd.uniform(-250, 250)) for i in range(9)]
        wps = []
        for k, e in enumerate(range(-300, 301, 60)):
            n0, n1 = (-300, 300) if k % 2 == 0 else (300, -300)
            wps += [(e, n0), (e, n1)]             # (east, north) — как (x, y)

    srv = DashboardServer(core, '0.0.0.0', args.port, os.path.join(HERE, '..', 'web'), ui, localmap=localmap)
    srv.start()
    print('Режим %s. Откройте http://localhost:%d  (Ctrl+C — выход)' % (args.mode, args.port))
    if local:
        print('Файлы поля маркеров для ручной загрузки: %s' % os.path.join(HERE, 'sim_aruco'))

    video = FakeVideo(png, field_meta)
    bgr = {'fire': (54, 90, 255), 'car': (48, 194, 242)}
    seg, pos = 0, list(wps[0])
    blind_since = time.time()     # у верхнего края поля маркеры «не видны» — проверка индикатора
    t_prev = time.time()
    try:
        while True:
            now = time.time()
            dt, t_prev = now - t_prev, now
            tx, ty = wps[(seg + 1) % len(wps)]
            dx, dy = tx - pos[0], ty - pos[1]
            dist = math.hypot(dx, dy)
            if dist <= speed * dt:
                seg = (seg + 1) % len(wps)
                pos = [tx, ty]
            else:
                pos[0] += dx / dist * speed * dt
                pos[1] += dy / dist * speed * dt
            yaw = math.degrees(math.atan2(dx, dy)) % 360          # по часовой от +Y (север)
            tel = dict(agl=agl + rnd.gauss(0, 0.02 if local else 0.3), roll=rnd.gauss(0, 2),
                       pitch=rnd.gauss(-3, 1), yaw=yaw,
                       vn=dy / max(dist, 1e-6) * speed, ve=dx / max(dist, 1e-6) * speed, vd=0.0)
            if local:
                tel.update(x=pos[0], y=pos[1], pos_t=now, ref_t=now if pos[1] < 3.5 else blind_since)
            else:
                lat, lon = offset_latlon(args.lat, args.lon, pos[1], pos[0])
                tel.update(lat=lat, lon=lon, amsl=150 + agl, fix=3, sats=14)
            if local and pos[1] < 3.5:
                blind_since = now
            core.on_telemetry(now, **tel)
            ftel = dict(tel, t=now)

            dets = []
            for cls, a, b in truth:
                if local:
                    north, east = b - pos[1], a - pos[0]
                else:
                    north = math.radians(a - lat) * 6371000.0
                    east = math.radians(b - lon) * 6371000.0 * math.cos(math.radians(lat))
                px = project(geo, ftel, north, east)
                if px and rnd.random() < 0.8:
                    size = 60 if cls == 'fire' else 45
                    dets.append({'cx': px[0] + rnd.gauss(0, 4), 'cy': px[1] + rnd.gauss(0, 4),
                                 'w': size, 'h': size * 0.8, 'class_id': 0 if cls == 'fire' else 1,
                                 'class_name': cls, 'conf': min(0.99, rnd.uniform(0.62, 0.97))})
            if rnd.random() < 0.01:   # редкая ложная детекция
                dets.append({'cx': rnd.uniform(0, W), 'cy': rnd.uniform(0, H), 'w': 40, 'h': 30,
                             'class_id': 0, 'class_name': 'fire', 'conf': 0.66})
            core.on_detections({'timestamp': now, 'frame_width': W, 'frame_height': H, 'detections': dets})

            if core.videos['thermal'].clients:
                core.on_video_frame('thermal', video.thermal(dets, bgr))
            if core.videos['course'].clients:
                core.on_video_frame('course', video.course(ftel))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()
        store.close()


if __name__ == '__main__':
    main()
