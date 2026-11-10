# -*- coding: utf-8 -*-
"""
Ядро веб-интерфейса обнаружения (без зависимостей от ROS).

Два режима навигации:
  gps   — координаты объектов в широте/долготе (ВТОЛ, полёт по GPS);
  local — координаты в метрах на плане площадки (тестовый дрон по ArUco-маркерам).
          Система плана: X вправо, Y вверх (как в карте маркеров ArUco),
          курс БПЛА считается по часовой стрелке от оси +Y.

Компоненты:
  Geolocator      — пиксель рамки -> точка на земле (ориентация + установка камеры + высота)
  TelemetryBuffer — история телеметрии, выбор отсчёта по времени кадра
  ObjectTracker   — объединение повторных детекций одного объекта
  EventStore      — журнал в SQLite
  LocalMapStore   — подложка плана площадки (изображение + карта маркеров)
  DashboardServer — HTTP: статика, REST, Server-Sent Events, MJPEG
"""

import csv
import io
import json
import math
import mimetypes
import os
import queue
import sqlite3
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

MODES = ('gps', 'local')

# ----------------------------------------------------------------------------
# Геометрия
# ----------------------------------------------------------------------------
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _rot_zyx(roll, pitch, yaw):
    """Матрица поворота body->NED (ZYX, углы в радианах, соглашение MAVLink)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def _mv(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def offset_latlon(lat, lon, north_m, east_m):
    """Сдвиг точки на north/east метров (локальные радиусы WGS-84)."""
    phi = math.radians(lat)
    s = math.sin(phi)
    k = 1.0 - WGS84_E2 * s * s
    r_n = WGS84_A * (1.0 - WGS84_E2) / (k ** 1.5)
    r_e = WGS84_A / math.sqrt(k)
    dlat = math.degrees(north_m / r_n)
    dlon = math.degrees(east_m / (r_e * max(math.cos(phi), 1e-9)))
    return lat + dlat, lon + dlon


def distance_m(lat1, lon1, lat2, lon2):
    """Расстояние (равнопромежуточная аппроксимация, точна на сотнях метров)."""
    phi = math.radians((lat1 + lat2) / 2.0)
    dn = math.radians(lat2 - lat1) * 6371000.0
    de = math.radians(lon2 - lon1) * 6371000.0 * math.cos(phi)
    return math.hypot(dn, de)


class Geolocator:
    """
    Пересчёт пикселя (u, v) в точку на земле.

    Оптическая система камеры: x вправо, y вниз, z вдоль оси объектива.
    Установка камеры относительно корпуса (FRD):
      mount_pitch_deg = 0   — вперёд по курсу,
      mount_pitch_deg = -90 — строго вниз (надир), верх кадра = нос БПЛА.
    stabilized = True — камера на подвесе, крен/тангаж БПЛА не учитываются.
    """

    _OPT_TO_BODY = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))

    def __init__(self, mode='gps', hfov_deg=56.0, vfov_deg=42.0, mount_pitch_deg=-90.0,
                 mount_roll_deg=0.0, mount_yaw_deg=0.0, flip_x=False, flip_y=False,
                 stabilized=False, min_agl_m=3.0, max_range_m=1500.0, pos_timeout=1.0):
        if mode not in MODES:
            raise ValueError('mode должен быть gps или local')
        self.mode = mode
        self.tan_h = math.tan(math.radians(hfov_deg) / 2.0)
        self.tan_v = math.tan(math.radians(vfov_deg) / 2.0)
        self.r_mount = _rot_zyx(math.radians(mount_roll_deg),
                                math.radians(mount_pitch_deg),
                                math.radians(mount_yaw_deg))
        self.sx = -1.0 if flip_x else 1.0
        self.sy = -1.0 if flip_y else 1.0
        self.stabilized = stabilized
        self.min_agl = min_agl_m
        self.max_range = max_range_m
        self.pos_timeout = pos_timeout

    def _ray_ned(self, tel, u, v, w, h):
        xn = (u - w / 2.0) / (w / 2.0) * self.tan_h * self.sx
        yn = (v - h / 2.0) / (h / 2.0) * self.tan_v * self.sy
        ray = _mv(self._OPT_TO_BODY, (xn, yn, 1.0))
        ray = _mv(self.r_mount, ray)
        roll = 0.0 if self.stabilized else math.radians(tel['roll'] or 0.0)
        pitch = 0.0 if self.stabilized else math.radians(tel['pitch'] or 0.0)
        ray = _mv(_rot_zyx(roll, pitch, math.radians(tel['yaw'] or 0.0)), ray)
        n = math.sqrt(ray[0] ** 2 + ray[1] ** 2 + ray[2] ** 2)
        return ray[0] / n, ray[1] / n, ray[2] / n

    def check_telemetry(self, tel):
        if tel is None:
            return 'no_telemetry'
        if self.mode == 'gps':
            if (tel.get('fix') or 0) < 3 or tel.get('lat') is None or \
                    (tel['lat'] == 0.0 and tel['lon'] == 0.0):
                return 'no_fix'
        else:
            if tel.get('x') is None or tel.get('pos_t') is None:
                return 'no_position'
            if tel['t'] - tel['pos_t'] > self.pos_timeout:
                return 'stale_position'
        if tel.get('agl') is None:
            return 'no_alt'
        if tel['agl'] < self.min_agl:
            return 'low_alt'
        return 'ok'

    def _shift(self, tel, north, east):
        """Точка дрона + смещение -> координаты режима (lat, lon) или (x, y)."""
        if self.mode == 'gps':
            return offset_latlon(tel['lat'], tel['lon'], north, east)
        return tel['x'] + east, tel['y'] + north

    def locate(self, tel, u, v, w, h):
        """-> (pos, ground_distance_m, status); pos = {'lat','lon'} или {'x','y'}"""
        status = self.check_telemetry(tel)
        if status != 'ok':
            return None, None, status
        dn, de, dd = self._ray_ned(tel, u, v, w, h)
        if dd < 1e-3:
            return None, None, 'above_horizon'
        t = tel['agl'] / dd
        north, east = t * dn, t * de
        dist = math.hypot(north, east)
        if dist > self.max_range:
            return None, dist, 'too_far'
        a, b = self._shift(tel, north, east)
        pos = {'lat': a, 'lon': b} if self.mode == 'gps' else {'x': a, 'y': b}
        return pos, dist, 'ok'

    def footprint(self, tel, w, h):
        """Углы поля зрения камеры на земле: [[lat,lon]...] или [[x,y]...]."""
        if self.check_telemetry(tel) != 'ok':
            return None
        pts = []
        for u, v in ((0, 0), (w, 0), (w, h), (0, h)):
            dn, de, dd = self._ray_ned(tel, u, v, w, h)
            horiz = math.hypot(dn, de)
            if dd > 1e-3 and tel['agl'] / dd * horiz <= self.max_range:
                t = tel['agl'] / dd
                north, east = t * dn, t * de
            else:   # луч выше горизонта или слишком далеко — обрезаем дальностью
                k = self.max_range / max(horiz, 1e-9)
                north, east = dn * k, de * k
            pts.append(list(self._shift(tel, north, east)))
        return pts


# ----------------------------------------------------------------------------
# Телеметрия
# ----------------------------------------------------------------------------
TEL_FIELDS = ('lat', 'lon', 'amsl', 'agl', 'roll', 'pitch', 'yaw',
              'fix', 'sats', 'vn', 've', 'vd', 'x', 'y', 'pos_t', 'ref_t')


class TelemetryBuffer:
    """Хранит последние ~20 с телеметрии. Поля, пришедшие разными топиками
    с одинаковой меткой времени, сливаются в один отсчёт."""

    def __init__(self, maxlen=1500):
        self._buf = deque(maxlen=maxlen)
        self._cur = {k: None for k in TEL_FIELDS}
        self._cur.update(roll=0.0, pitch=0.0, yaw=0.0, fix=0, sats=0)
        self._lock = threading.Lock()

    def update(self, stamp, **fields):
        with self._lock:
            self._cur.update({k: v for k, v in fields.items() if k in TEL_FIELDS})
            sample = dict(self._cur, t=stamp)
            if self._buf and abs(self._buf[-1]['t'] - stamp) < 1e-4:
                self._buf[-1] = sample
            else:
                self._buf.append(sample)

    def latest(self):
        with self._lock:
            return dict(self._buf[-1]) if self._buf else None

    def at(self, stamp):
        """Ближайший по времени отсчёт и разница во времени (с)."""
        with self._lock:
            if not self._buf:
                return None, None
            best = min(self._buf, key=lambda s: abs(s['t'] - stamp))
            return dict(best), abs(best['t'] - stamp)


# ----------------------------------------------------------------------------
# Трекер объектов
# ----------------------------------------------------------------------------
class ObjectTracker:
    """Детекции одного класса ближе assoc_radius (+ доля от дальности) к уже
    известному объекту считаются тем же объектом. Координаты объекта —
    среднее, взвешенное по уверенности."""

    def __init__(self, store, mode='gps', assoc_radius_m=20.0, range_factor=0.05, min_hits=2):
        self.store = store
        self.mode = mode
        self.keys = ('lat', 'lon') if mode == 'gps' else ('x', 'y')
        self.radius = assoc_radius_m
        self.range_factor = range_factor
        self.min_hits = min_hits
        self.objects = {}
        self._lock = threading.Lock()

    def _dist(self, p, o):
        if self.mode == 'gps':
            return distance_m(p['lat'], p['lon'], o['lat'], o['lon'])
        return math.hypot(p['x'] - o['x'], p['y'] - o['y'])

    def update(self, cls, pos, conf, ts, dist):
        """-> (object_dict, event); event: 'new' | 'update' | None (ещё не подтверждён)"""
        gate = self.radius + self.range_factor * (dist or 0.0)
        k1, k2 = self.keys
        with self._lock:
            best, best_d = None, gate
            for o in self.objects.values():
                if o['cls'] != cls:
                    continue
                d = self._dist(pos, o)
                if d <= best_d:
                    best, best_d = o, d
            if best is None:
                best = {'id': None, 'cls': cls, 'lat': None, 'lon': None, 'x': None, 'y': None,
                        'w': conf, 'first_ts': ts, 'last_ts': ts, 'hits': 1,
                        'max_conf': conf, 'last_conf': conf}
                best[k1], best[k2] = pos[k1], pos[k2]
                best['id'] = self.store.insert_object(best)
                self.objects[best['id']] = best
            else:
                wsum = best['w'] + conf
                best[k1] = (best[k1] * best['w'] + pos[k1] * conf) / wsum
                best[k2] = (best[k2] * best['w'] + pos[k2] * conf) / wsum
                best['w'] = min(wsum, 50.0)        # старые наблюдения не «замораживают» точку
                best['hits'] += 1
                best['last_ts'] = max(best['last_ts'], ts)
                best['first_ts'] = min(best['first_ts'], ts)
                best['max_conf'] = max(best['max_conf'], conf)
                best['last_conf'] = conf
                self.store.update_object(best)
            if best['hits'] < self.min_hits:
                event = None
            elif best['hits'] == self.min_hits:
                event = 'new'
            else:
                event = 'update'
            return dict(best), event

    def confirmed(self):
        with self._lock:
            return [{k: v for k, v in o.items() if k != 'w'}
                    for o in self.objects.values() if o['hits'] >= self.min_hits]


# ----------------------------------------------------------------------------
# Журнал (SQLite)
# ----------------------------------------------------------------------------
class EventStore:
    DET_COLUMNS = (
        ('session', 'TEXT'), ('mode', 'TEXT'), ('ts', 'REAL'), ('cls', 'TEXT'),
        ('cls_id', 'INTEGER'), ('conf', 'REAL'),
        ('lat', 'REAL'), ('lon', 'REAL'), ('x', 'REAL'), ('y', 'REAL'),
        ('ground_dist', 'REAL'), ('geo_status', 'TEXT'), ('obj_id', 'INTEGER'),
        ('px', 'REAL'), ('py', 'REAL'), ('pw', 'REAL'), ('ph', 'REAL'),
        ('drone_lat', 'REAL'), ('drone_lon', 'REAL'), ('drone_x', 'REAL'), ('drone_y', 'REAL'),
        ('drone_agl', 'REAL'), ('drone_yaw', 'REAL'), ('tel_age', 'REAL'),
    )
    OBJ_COLUMNS = (
        ('session', 'TEXT'), ('mode', 'TEXT'), ('cls', 'TEXT'),
        ('lat', 'REAL'), ('lon', 'REAL'), ('x', 'REAL'), ('y', 'REAL'),
        ('first_ts', 'REAL'), ('last_ts', 'REAL'), ('hits', 'INTEGER'), ('max_conf', 'REAL'),
    )
    DET_COLS = ('id',) + tuple(c for c, _ in DET_COLUMNS)
    OBJ_COLS = ('id',) + tuple(c for c, _ in OBJ_COLUMNS)

    def __init__(self, path, session, mode='gps', commit_interval=1.0):
        path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self.path = path
        self.session = session
        self.mode = mode
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=NORMAL')
        self._ensure_table('detections', self.DET_COLUMNS)
        self._ensure_table('objects', self.OBJ_COLUMNS)
        self.db.execute('CREATE INDEX IF NOT EXISTS idx_det_ts ON detections(ts)')
        self.db.execute('CREATE INDEX IF NOT EXISTS idx_det_session ON detections(session)')
        self.db.execute('CREATE INDEX IF NOT EXISTS idx_obj_session ON objects(session)')
        self.db.commit()
        self._lock = threading.Lock()
        self._commit_interval = commit_interval
        self._last_commit = time.monotonic()

    def _ensure_table(self, name, columns):
        """Создаёт таблицу; в журнал старой версии добавляет недостающие столбцы."""
        cols = ', '.join('%s %s' % c for c in columns)
        self.db.execute('CREATE TABLE IF NOT EXISTS %s (id INTEGER PRIMARY KEY AUTOINCREMENT, %s)'
                        % (name, cols))
        have = {r[1] for r in self.db.execute('PRAGMA table_info(%s)' % name)}
        for c, t in columns:
            if c not in have:
                self.db.execute('ALTER TABLE %s ADD COLUMN %s %s' % (name, c, t))

    def _maybe_commit(self):
        if time.monotonic() - self._last_commit >= self._commit_interval:
            self.db.commit()
            self._last_commit = time.monotonic()

    def _fill(self, cols, d):
        fixed = {'session': self.session, 'mode': self.mode}
        return [fixed[c] if c in fixed else d.get(c) for c in cols]

    def insert_detection(self, d):
        cols = self.DET_COLS[1:]
        with self._lock:
            cur = self.db.execute('INSERT INTO detections (%s) VALUES (%s)'
                                  % (','.join(cols), ','.join('?' * len(cols))), self._fill(cols, d))
            self._maybe_commit()
            return cur.lastrowid

    def insert_object(self, o):
        cols = self.OBJ_COLS[1:]
        with self._lock:
            cur = self.db.execute('INSERT INTO objects (%s) VALUES (%s)'
                                  % (','.join(cols), ','.join('?' * len(cols))), self._fill(cols, o))
            self._maybe_commit()
            return cur.lastrowid

    def update_object(self, o):
        with self._lock:
            self.db.execute(
                'UPDATE objects SET lat=?, lon=?, x=?, y=?, first_ts=?, last_ts=?, hits=?, max_conf=? '
                'WHERE id=?', (o['lat'], o['lon'], o['x'], o['y'], o['first_ts'], o['last_ts'],
                               o['hits'], o['max_conf'], o['id']))
            self._maybe_commit()

    def flush(self):
        with self._lock:
            self.db.commit()
            self._last_commit = time.monotonic()

    def query_detections(self, session=None, cls=None, since=None, limit=1000, located_only=False):
        sql, args = 'SELECT %s FROM detections WHERE 1=1' % ','.join(self.DET_COLS), []
        if session:
            sql += ' AND session=?'
            args.append(session)
        if cls:
            sql += ' AND cls=?'
            args.append(cls)
        if since is not None:
            sql += ' AND ts>=?'
            args.append(since)
        if located_only:
            sql += " AND geo_status='ok'"
        sql += ' ORDER BY id DESC'
        if limit:
            sql += ' LIMIT ?'
            args.append(int(limit))
        with self._lock:
            rows = self.db.execute(sql, args).fetchall()
        return [dict(zip(self.DET_COLS, r)) for r in rows]

    def query_objects(self, session=None, min_hits=1):
        sql, args = 'SELECT %s FROM objects WHERE hits>=?' % ','.join(self.OBJ_COLS), [min_hits]
        if session:
            sql += ' AND session=?'
            args.append(session)
        with self._lock:
            rows = self.db.execute(sql, args).fetchall()
        return [dict(zip(self.OBJ_COLS, r)) for r in rows]

    def sessions(self):
        with self._lock:
            rows = self.db.execute(
                'SELECT session, MAX(mode), MIN(ts), MAX(ts), COUNT(*) FROM detections '
                'GROUP BY session ORDER BY MIN(ts) DESC').fetchall()
        return [{'session': r[0], 'mode': r[1] or 'gps', 'from': r[2], 'to': r[3], 'count': r[4]}
                for r in rows]

    def close(self):
        with self._lock:
            self.db.commit()
            self.db.close()


# ----------------------------------------------------------------------------
# План площадки для локального режима
# ----------------------------------------------------------------------------
def parse_aruco_map(text):
    """
    Карта маркеров в формате Clover/aruco_pose:
        # id  length  x  y  z  rot_z  rot_y  rot_x
        0     0.33    0  0  0  0      0      0
    x, y — центр маркера (м), rot_z — поворот (рад). Нужны минимум id, length, x, y.
    """
    markers, errors = [], []
    for n, line in enumerate(text.splitlines(), 1):
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.replace(',', ' ').replace(';', ' ').split()
        try:
            if len(parts) < 4:
                raise ValueError('нужно минимум 4 числа')
            m = {'id': int(float(parts[0])), 'length': float(parts[1]),
                 'x': float(parts[2]), 'y': float(parts[3]),
                 'z': float(parts[4]) if len(parts) > 4 else 0.0,
                 'rot_z': float(parts[5]) if len(parts) > 5 else 0.0}
            markers.append(m)
        except ValueError as e:
            errors.append('строка %d: %s' % (n, e))
    return markers, errors


def marker_corners(m):
    h = m['length'] / 2.0
    c, sn = math.cos(m.get('rot_z') or 0.0), math.sin(m.get('rot_z') or 0.0)
    return [(m['x'] + c * dx - sn * dy, m['y'] + sn * dx + c * dy)
            for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))]


def fit_board_image(markers, img_w, img_h, margin_px):
    """
    Положение изображения карты, нарисованного как cv::aruco::drawPlanarBoard
    (так рисует /aruco_map/image): габарит углов маркеров вписан в изображение
    за вычетом полей margin_px с сохранением пропорций и по центру, ось Y вверх.
    -> {'x0', 'y0', 'width_m'} — левый нижний угол и ширина изображения в метрах.
    """
    pts = [p for m in markers for p in marker_corners(m)]
    min_x, max_x = min(p[0] for p in pts), max(p[0] for p in pts)
    min_y, max_y = min(p[1] for p in pts), max(p[1] for p in pts)
    bw, bh = max(max_x - min_x, 1e-6), max(max_y - min_y, 1e-6)
    inner_w, inner_h = img_w - 2 * margin_px, img_h - 2 * margin_px
    if inner_w <= 0 or inner_h <= 0:
        raise ValueError('поля больше изображения')
    scale = min(inner_w / bw, inner_h / bh)              # пикселей на метр
    pad_x = margin_px + (inner_w - bw * scale) / 2.0
    pad_y = margin_px + (inner_h - bh * scale) / 2.0
    return {'x0': min_x - pad_x / scale, 'y0': min_y - pad_y / scale, 'width_m': img_w / scale}


class LocalMapStore:
    IMAGE_TYPES = {'image/png': '.png', 'image/jpeg': '.jpg', 'image/webp': '.webp'}

    def __init__(self, data_dir):
        self.dir = os.path.expanduser(data_dir)
        os.makedirs(self.dir, exist_ok=True)
        self._state_path = os.path.join(self.dir, 'localmap.json')
        self._lock = threading.Lock()
        self.ros_loader = None      # функция, которая достаёт изображение и карту из ROS (ставит нода)
        self.ros_margin = 200
        self.state = {'image': None, 'image_version': 0, 'image_w': None, 'image_h': None,
                      'x0': 0.0, 'y0': 0.0, 'width_m': 10.0, 'opacity': 0.85,
                      'markers': [], 'markers_name': None}
        try:
            with open(self._state_path, encoding='utf-8') as f:
                self.state.update(json.load(f))
        except (OSError, ValueError):
            pass

    def _save(self):
        os.makedirs(self.dir, exist_ok=True)
        tmp = self._state_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(self.state, f, ensure_ascii=False)
        os.replace(tmp, self._state_path)

    def public(self):
        with self._lock:
            s = dict(self.state)
        s['image_url'] = ('localmap/image?v=%d' % s['image_version']) if s['image'] else None
        s['ros_available'] = self.ros_loader is not None
        s['ros_margin'] = self.ros_margin
        return s

    def set_image(self, data, ctype, img_w, img_h):
        ext = self.IMAGE_TYPES.get(ctype.split(';')[0].strip().lower())
        if not ext:
            raise ValueError('поддерживаются PNG, JPEG, WebP')
        with self._lock:
            self._drop_image_file()
            os.makedirs(self.dir, exist_ok=True)
            name = 'localmap_image' + ext
            with open(os.path.join(self.dir, name), 'wb') as f:
                f.write(data)
            self.state.update(image=name, image_version=self.state['image_version'] + 1,
                              image_w=img_w, image_h=img_h)
            self._save()

    def _drop_image_file(self):
        if self.state['image']:
            try:
                os.remove(os.path.join(self.dir, self.state['image']))
            except OSError:
                pass

    def clear_image(self):
        with self._lock:
            self._drop_image_file()
            self.state.update(image=None, image_version=self.state['image_version'] + 1)
            self._save()

    def image_path(self):
        with self._lock:
            return os.path.join(self.dir, self.state['image']) if self.state['image'] else None

    def set_config(self, cfg):
        with self._lock:
            for k in ('x0', 'y0', 'width_m', 'opacity'):
                if k in cfg and cfg[k] is not None:
                    v = float(cfg[k])
                    if k == 'width_m' and v <= 0:
                        raise ValueError('ширина должна быть больше 0')
                    self.state[k] = v
            self._save()

    def set_markers(self, text, name=None):
        markers, errors = parse_aruco_map(text)
        if not markers:
            raise ValueError('в файле не найдено маркеров' + (': ' + errors[0] if errors else ''))
        with self._lock:
            self.state.update(markers=markers, markers_name=name)
            self._save()
        return len(markers), errors

    def fit_to_markers(self, margin_px):
        with self._lock:
            st = dict(self.state)
        if not st['image'] or not st['markers']:
            raise ValueError('нужны и изображение, и карта маркеров')
        self.set_config(fit_board_image(st['markers'], st['image_w'], st['image_h'], float(margin_px)))

    def clear_markers(self):
        with self._lock:
            self.state.update(markers=[], markers_name=None)
            self._save()


# ----------------------------------------------------------------------------
# Рассылка событий (SSE) и видео (MJPEG)
# ----------------------------------------------------------------------------
class Broker:
    def __init__(self):
        self._clients = set()
        self._lock = threading.Lock()

    def subscribe(self):
        q = queue.Queue(maxsize=1000)
        with self._lock:
            self._clients.add(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            self._clients.discard(q)

    def publish(self, event, data):
        payload = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait((event, payload))
            except queue.Full:
                pass

    @property
    def count(self):
        with self._lock:
            return len(self._clients)


class VideoHub:
    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self._clients = 0
        self._cl_lock = threading.Lock()
        self.last_frame_time = None

    @property
    def clients(self):
        with self._cl_lock:
            return self._clients

    def add_client(self, delta):
        with self._cl_lock:
            self._clients += delta

    def push(self, jpeg_bytes):
        with self._cond:
            self._frame = jpeg_bytes
            self._seq += 1
            self.last_frame_time = time.time()
            self._cond.notify_all()

    def wait(self, last_seq, timeout=2.0):
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout)
            return self._seq, self._frame


# ----------------------------------------------------------------------------
# Сборка: вход данных -> геопривязка -> журнал -> рассылка
# ----------------------------------------------------------------------------
GEO_STATUS_TEXT = {
    'ok': 'координаты определены',
    'no_telemetry': 'нет телеметрии',
    'stale_telemetry': 'телеметрия устарела',
    'no_fix': 'нет 3D-фикса GPS',
    'no_position': 'нет позиции по маркерам',
    'stale_position': 'позиция устарела',
    'no_alt': 'нет высоты',
    'low_alt': 'БПЛА ниже минимальной высоты',
    'above_horizon': 'объект выше горизонта',
    'too_far': 'дальше допустимой дальности',
}


class DashboardCore:
    def __init__(self, geolocator, store, tracker, max_tel_age=0.5, telemetry_hz=5.0,
                 object_update_hz=2.0, default_frame=(640, 480), video_streams=('thermal',)):
        self.geo = geolocator
        self.mode = geolocator.mode
        self.store = store
        self.tracker = tracker
        self.tel = TelemetryBuffer()
        self.broker = Broker()
        self.videos = {name: VideoHub() for name in video_streams}
        self.max_tel_age = max_tel_age
        self._tel_period = 1.0 / telemetry_hz
        self._obj_period = 1.0 / object_update_hz
        self._last_tel_pub = 0.0
        self._last_obj_pub = {}
        self.frame_size = default_frame
        self.stats = {'frames': 0, 'detections': 0, 'located': 0, 'last_det_ts': None,
                      'started': time.time()}
        self._last_tel_rx = None

    # --- входы ---
    def on_telemetry(self, stamp, **fields):
        self.tel.update(stamp, **fields)
        self._last_tel_rx = time.time()
        now = time.monotonic()
        if now - self._last_tel_pub >= self._tel_period:
            self._last_tel_pub = now
            self.broker.publish('telemetry', self.telemetry_snapshot())

    def telemetry_snapshot(self):
        t = self.tel.latest()
        if t is None:
            return {'available': False}
        gs = None
        if t.get('vn') is not None and t.get('ve') is not None:
            gs = math.hypot(t['vn'], t['ve'])
        return {
            'available': True, 'ts': t['t'], 'lat': t['lat'], 'lon': t['lon'],
            'x': t['x'], 'y': t['y'],
            'pos_age': (t['t'] - t['pos_t']) if t.get('pos_t') is not None else None,
            # сколько секунд маркеры не видны (только для источника tf с фреймом одометрии)
            'ref_age': max(0.0, t['t'] - t['ref_t']) if t.get('ref_t') is not None else None,
            'agl': t['agl'], 'amsl': t['amsl'], 'yaw': t['yaw'], 'roll': t['roll'],
            'pitch': t['pitch'], 'fix': t['fix'], 'sats': t['sats'], 'gs': gs,
            'geo_status': self.geo.check_telemetry(t),
            'rx_age': (time.time() - self._last_tel_rx) if self._last_tel_rx else None,
            'footprint': self.geo.footprint(t, *self.frame_size),
        }

    def on_detections(self, payload):
        """payload — словарь из JSON топика /fire/detections."""
        ts = float(payload.get('timestamp') or time.time())
        w = int(payload.get('frame_width') or self.frame_size[0])
        h = int(payload.get('frame_height') or self.frame_size[1])
        self.frame_size = (w, h)
        self.stats['frames'] += 1
        dets = payload.get('detections') or []
        if not dets:
            return []
        tel, age = self.tel.at(ts)
        out = []
        for d in dets:
            pos, dist = None, None
            if tel is not None and age is not None and age > self.max_tel_age:
                status = 'stale_telemetry'
            else:
                pos, dist, status = self.geo.locate(tel, d['cx'], d['cy'], w, h)
            cls = d.get('class_name', 'class_%s' % d.get('class_id'))
            conf = float(d.get('conf', 0.0))
            obj, obj_event = None, None
            if status == 'ok':
                obj, obj_event = self.tracker.update(cls, pos, conf, ts, dist)
            rec = {
                'ts': ts, 'mode': self.mode, 'cls': cls, 'cls_id': d.get('class_id'), 'conf': conf,
                'lat': None, 'lon': None, 'x': None, 'y': None,
                'ground_dist': dist, 'geo_status': status, 'obj_id': obj['id'] if obj else None,
                'px': d['cx'], 'py': d['cy'], 'pw': d.get('w'), 'ph': d.get('h'),
                'drone_lat': tel.get('lat') if tel else None,
                'drone_lon': tel.get('lon') if tel else None,
                'drone_x': tel.get('x') if tel else None,
                'drone_y': tel.get('y') if tel else None,
                'drone_agl': tel.get('agl') if tel else None,
                'drone_yaw': tel.get('yaw') if tel else None,
                'tel_age': age,
            }
            if pos:
                rec.update(pos)
            rec['id'] = self.store.insert_detection(rec)
            self.stats['detections'] += 1
            self.stats['last_det_ts'] = ts
            if status == 'ok':
                self.stats['located'] += 1
            self.broker.publish('detection', rec)
            if obj_event:
                self._publish_object(obj, obj_event == 'new')
            out.append(rec)
        return out

    def _publish_object(self, obj, is_new):
        now = time.monotonic()
        if not is_new and now - self._last_obj_pub.get(obj['id'], 0.0) < self._obj_period:
            return
        self._last_obj_pub[obj['id']] = now
        data = {k: obj[k] for k in ('id', 'cls', 'lat', 'lon', 'x', 'y', 'first_ts', 'last_ts',
                                    'hits', 'max_conf', 'last_conf')}
        data['is_new'] = is_new
        self.broker.publish('object', data)

    def on_video_frame(self, stream, jpeg_bytes):
        hub = self.videos.get(stream)
        if hub is not None and jpeg_bytes:
            hub.push(jpeg_bytes)


# ----------------------------------------------------------------------------
# Размер изображения без сторонних библиотек (для пропорций подложки)
# ----------------------------------------------------------------------------
def image_size(data):
    try:
        if data[:8] == b'\x89PNG\r\n\x1a\n':
            return int.from_bytes(data[16:20], 'big'), int.from_bytes(data[20:24], 'big')
        if data[:2] == b'\xff\xd8':
            i = 2
            while i < len(data) - 9:
                if data[i] != 0xFF:
                    i += 1
                    continue
                marker = data[i + 1]
                if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
                    return int.from_bytes(data[i + 7:i + 9], 'big'), int.from_bytes(data[i + 5:i + 7], 'big')
                i += 2 + int.from_bytes(data[i + 2:i + 4], 'big')
        if data[:4] == b'RIFF' and data[8:12] == b'WEBP':
            chunk = data[12:16]
            if chunk == b'VP8X':
                return 1 + int.from_bytes(data[24:27], 'little'), 1 + int.from_bytes(data[27:30], 'little')
            if chunk == b'VP8 ':
                return (int.from_bytes(data[26:28], 'little') & 0x3FFF,
                        int.from_bytes(data[28:30], 'little') & 0x3FFF)
            if chunk == b'VP8L':
                b = int.from_bytes(data[21:25], 'little')
                return (b & 0x3FFF) + 1, ((b >> 14) & 0x3FFF) + 1
    except Exception:
        pass
    return None, None


# ----------------------------------------------------------------------------
# HTTP-сервер
# ----------------------------------------------------------------------------
mimetypes.add_type('application/javascript', '.js')
mimetypes.add_type('text/css', '.css')
mimetypes.add_type('image/webp', '.webp')

MAX_UPLOAD = 40 * 1024 * 1024


def _fmt(v, f):
    return '' if v is None else f % v


def make_handler(core, web_dir, ui_config, tiles_dir=None, localmap=None):
    web_dir = os.path.realpath(web_dir)
    tiles_dir = os.path.realpath(tiles_dir) if tiles_dir else None

    class Handler(BaseHTTPRequestHandler):
        server_version = 'FireDashboard/2.0'

        def log_message(self, fmt, *args):
            pass

        # --- утилиты ---
        def _send(self, code, body, ctype, extra=None):
            if isinstance(body, str):
                body = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, data, code=200):
            self._send(code, json.dumps(data, ensure_ascii=False), 'application/json; charset=utf-8')

        def _file(self, root, rel, cache='no-cache'):
            path = os.path.realpath(os.path.join(root, rel))
            if not path.startswith(root + os.sep) or not os.path.isfile(path):
                return self._send(404, 'not found', 'text/plain')
            ctype = mimetypes.guess_type(path)[0] or 'application/octet-stream'
            with open(path, 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', cache)
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            n = int(self.headers.get('Content-Length') or 0)
            if n > MAX_UPLOAD:
                raise ValueError('файл больше %d МБ' % (MAX_UPLOAD // 2 ** 20))
            return self.rfile.read(n) if n > 0 else b''

        # --- GET ---
        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            p = url.path
            try:
                if p in ('/', '/index.html'):
                    return self._file(web_dir, 'index.html')
                if p == '/api/config':
                    cfg = dict(ui_config)
                    cfg.update(session=core.store.session, mode=core.mode,
                               local_tiles=bool(tiles_dir), geo_status_text=GEO_STATUS_TEXT,
                               video_streams=[{'name': n, 'label': ui_config.get('video_labels', {}).get(n, n)}
                                              for n in core.videos])
                    return self._json(cfg)
                if p == '/api/state':
                    return self._json({'telemetry': core.telemetry_snapshot(),
                                       'stats': core.stats, 'clients': core.broker.count})
                if p == '/api/video_status':
                    now = time.time()
                    return self._json({n: {'age': (now - h.last_frame_time) if h.last_frame_time else None,
                                           'clients': h.clients} for n, h in core.videos.items()})
                if p == '/api/objects':
                    s = q.get('session', 'current')
                    if s == 'current':
                        return self._json(core.tracker.confirmed())
                    return self._json(core.store.query_objects(
                        None if s == 'all' else s, core.tracker.min_hits))
                if p == '/api/events':
                    return self._json(core.store.query_detections(
                        session=self._session(q), cls=q.get('cls') or None,
                        since=float(q['since']) if 'since' in q else None,
                        limit=min(int(q.get('limit', 500)), 100000),
                        located_only=q.get('located') == '1'))
                if p == '/api/sessions':
                    return self._json(core.store.sessions())
                if p == '/api/export.csv':
                    return self._csv(q)
                if p == '/api/stream':
                    return self._sse()
                if p == '/api/localmap':
                    return self._json(localmap.public() if localmap else None)
                if p == '/localmap/image' and localmap:
                    ip = localmap.image_path()
                    if not ip:
                        return self._send(404, 'no image', 'text/plain')
                    return self._file(os.path.dirname(os.path.realpath(ip)),
                                      os.path.basename(ip), cache='max-age=31536000')
                if p == '/video.mjpg':
                    return self._mjpeg('thermal')
                if p.startswith('/video/') and p.endswith('.mjpg'):
                    return self._mjpeg(p[len('/video/'):-len('.mjpg')])
                if p.startswith('/tiles/') and tiles_dir:
                    return self._file(tiles_dir, p[len('/tiles/'):], cache='max-age=86400')
                return self._file(web_dir, p.lstrip('/'))
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                try:
                    self._json({'error': str(e)}, 500)
                except Exception:
                    pass

        # --- POST: план площадки ---
        def do_POST(self):
            p = urlparse(self.path).path
            q = {k: v[0] for k, v in parse_qs(urlparse(self.path).query).items()}
            try:
                if not localmap:
                    return self._json({'error': 'план площадки доступен только в локальном режиме'}, 400)
                if p == '/api/localmap/image':
                    data = self._body()
                    w, h = image_size(data)
                    if not w:
                        raise ValueError('не удалось прочитать изображение')
                    localmap.set_image(data, self.headers.get('Content-Type', ''), w, h)
                elif p == '/api/localmap/image/delete':
                    localmap.clear_image()
                elif p == '/api/localmap/config':
                    localmap.set_config(json.loads(self._body() or b'{}'))
                elif p == '/api/localmap/markers':
                    n, errors = localmap.set_markers(self._body().decode('utf-8', 'replace'),
                                                     q.get('name'))
                    state = localmap.public()
                    core.broker.publish('localmap', state)
                    return self._json({'ok': True, 'count': n, 'warnings': errors[:10], 'state': state})
                elif p == '/api/localmap/fit':
                    localmap.fit_to_markers(float(q.get('margin', localmap.ros_margin)))
                elif p == '/api/localmap/from_ros':
                    if localmap.ros_loader is None:
                        raise ValueError('загрузка с дрона недоступна')
                    img, w, h, ctype, text, name = localmap.ros_loader()
                    localmap.set_image(img, ctype, w, h)
                    localmap.set_markers(text, name)
                    localmap.fit_to_markers(localmap.ros_margin)
                elif p == '/api/localmap/markers/delete':
                    localmap.clear_markers()
                else:
                    return self._json({'error': 'not found'}, 404)
                state = localmap.public()
                core.broker.publish('localmap', state)    # другие открытые вкладки обновятся
                return self._json({'ok': True, 'state': state})
            except ValueError as e:
                return self._json({'error': str(e)}, 400)
            except Exception as e:
                return self._json({'error': repr(e)}, 500)

        # --- журнал ---
        def _session(self, q):
            s = q.get('session', 'current')
            if s == 'all':
                return None
            return core.store.session if s == 'current' else s

        def _csv(self, q):
            core.store.flush()
            rows = core.store.query_detections(session=self._session(q),
                                               cls=q.get('cls') or None, limit=None)
            rows.reverse()
            sep = q.get('sep', ';')
            buf = io.StringIO()
            wr = csv.writer(buf, delimiter=sep, lineterminator='\r\n')
            wr.writerow(['id', 'time_local', 'unix_ts', 'mode', 'class', 'confidence',
                         'lat', 'lon', 'x_m', 'y_m', 'ground_dist_m', 'geo_status', 'object_id',
                         'drone_lat', 'drone_lon', 'drone_x_m', 'drone_y_m', 'drone_alt_m',
                         'drone_yaw_deg', 'bbox_cx', 'bbox_cy', 'bbox_w', 'bbox_h', 'session'])
            for r in rows:
                wr.writerow([
                    r['id'], time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(r['ts']))
                    + '.%03d' % int((r['ts'] % 1) * 1000), '%.3f' % r['ts'], r['mode'] or 'gps',
                    r['cls'], '%.3f' % r['conf'],
                    _fmt(r['lat'], '%.7f'), _fmt(r['lon'], '%.7f'),
                    _fmt(r['x'], '%.3f'), _fmt(r['y'], '%.3f'),
                    _fmt(r['ground_dist'], '%.2f'), r['geo_status'], r['obj_id'] or '',
                    _fmt(r['drone_lat'], '%.7f'), _fmt(r['drone_lon'], '%.7f'),
                    _fmt(r['drone_x'], '%.3f'), _fmt(r['drone_y'], '%.3f'),
                    _fmt(r['drone_agl'], '%.2f'), _fmt(r['drone_yaw'], '%.1f'),
                    _fmt(r['px'], '%.0f'), _fmt(r['py'], '%.0f'),
                    _fmt(r['pw'], '%.0f'), _fmt(r['ph'], '%.0f'), r['session']])
            name = 'detections_%s.csv' % time.strftime('%Y%m%d_%H%M%S')
            self._send(200, '\ufeff' + buf.getvalue(), 'text/csv; charset=utf-8',
                       {'Content-Disposition': 'attachment; filename="%s"' % name})

        # --- потоки ---
        def _sse(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            q = core.broker.subscribe()
            try:
                self.wfile.write(b'retry: 2000\n\n')
                self.wfile.flush()
                while not getattr(self.server, 'stopping', False):
                    try:
                        ev, data = q.get(timeout=10)
                    except queue.Empty:
                        self.wfile.write(b': ping\n\n')
                        self.wfile.flush()
                        continue
                    self.wfile.write(('event: %s\ndata: %s\n\n' % (ev, data)).encode('utf-8'))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                core.broker.unsubscribe(q)

        def _mjpeg(self, name):
            hub = core.videos.get(name)
            if hub is None:
                return self._send(404, 'no such video stream', 'text/plain')
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            hub.add_client(1)
            seq = -1
            try:
                while not getattr(self.server, 'stopping', False):
                    seq2, frame = hub.wait(seq)
                    if frame is None or seq2 == seq:
                        continue
                    seq = seq2
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                hub.add_client(-1)

    return Handler


class DashboardServer:
    def __init__(self, core, host, port, web_dir, ui_config, tiles_dir=None, localmap=None):
        self.httpd = ThreadingHTTPServer((host, port),
                                         make_handler(core, web_dir, ui_config, tiles_dir, localmap))
        self.httpd.daemon_threads = True
        self.httpd.stopping = False
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.httpd.stopping = True
        self.httpd.shutdown()
        self.httpd.server_close()
