#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Geo Dashboard Node — координаты обнаруженных объектов + веб-интерфейс.
Режимы (~mode):
gps   — ВТОЛ: координаты из GPS, объекты в широте/долготе на карте местности.
local — тестовый дрон по ArUco: позиция в метрах, объекты на плане площадки.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # geo_core2 рядом
import rospy
from geometry_msgs.msg import PointStamped, TwistStamped
from sensor_msgs.msg import CompressedImage, Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String, UInt16MultiArray

from geo_core2 import (DashboardCore, DashboardServer, EventStore, Geolocator,
                       LocalMapStore, ObjectTracker)


def quat_to_euler_deg(q):
    """q — объект с x, y, z, w или кортеж (x, y, z, w) -> roll, pitch, yaw (градусы)."""
    if isinstance(q, (tuple, list)):
        q = type('Q', (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny, cosy)) % 360.0
    return math.degrees(roll), math.degrees(pitch), yaw


def bgr_to_hex(c):
    return '#%02x%02x%02x' % (int(c[2]), int(c[1]), int(c[0]))


class VideoRelay:
    """Подписывается на топик, только пока в браузере открыт этот видеопоток."""
    def __init__(self, core, name, topic, max_fps=10.0, max_width=640, quality=75):
        self.core, self.name, self.topic = core, name, topic
        self.period = 1.0 / max_fps if max_fps > 0 else 0.0
        self.max_width, self.quality = max_width, quality
        self.sub = None
        self.idle_since = None
        self._last = 0.0
        self._warned = False

    def _detect_class(self):
        try:
            import rostopic
            ttype, _, _ = rostopic.get_topic_type(self.topic, blocking=False)
        except Exception:
            ttype = None
            
        if ttype == 'sensor_msgs/CompressedImage':
            return CompressedImage
        if ttype == 'sensor_msgs/Image':
            return Image
        if ttype is None:
            return None  # топик ещё не опубликован — попробуем на следующем такте
            
        if not self._warned:
            rospy.logwarn('Видео %s: тип %s не поддерживается', self.topic, ttype)
            self._warned = True
        return None

    def tick(self):
        hub = self.core.videos[self.name]
        # Подписываемся, если есть клиенты и подписки ещё нет
        if hub.clients > 0 and self.sub is None:
            cls = self._detect_class()
            if cls is not None:
                self.sub = rospy.Subscriber(self.topic, cls, self._on_msg, queue_size=1, buff_size=2 ** 24)
                rospy.loginfo('Видео «%s»: подписка на %s (%s)', self.name, self.topic, cls.__name__)
                self.idle_since = None
        # Отписываемся, если клиентов нет уже 10 секунд
        elif hub.clients == 0 and self.sub is not None:
            now = time.monotonic()
            self.idle_since = self.idle_since or now
            if now - self.idle_since > 10.0:
                self.sub.unregister()
                self.sub = None
                rospy.loginfo('Видео «%s»: отписка от %s (нет клиентов)', self.name, self.topic)

    def _on_msg(self, msg):
        now = time.monotonic()
        if now - self._last < self.period:
            return
        self._last = now
        try:
            import img_convert  # numpy/cv2 нужны только для видео
            if isinstance(msg, CompressedImage):
                jpeg = img_convert.compressed_to_jpeg(msg.data, self.max_width, self.quality)
            else:
                jpeg = img_convert.encode_jpeg(img_convert.image_msg_to_bgr(msg), self.max_width, self.quality)
            if jpeg:
                self.core.on_video_frame(self.name, jpeg)
        except Exception as e:
            rospy.logwarn_throttle(10, 'Видео %s: %s' % (self.topic, e))


class GeoDashboardNode:
    def __init__(self):
        rospy.init_node('geo_dashboard_node')
        P = lambda name, default: rospy.get_param('~' + name, default)
        
        self.mode = P('mode', 'gps')
        if self.mode not in ('gps', 'local'):
            rospy.logerr('~mode=%s: допустимо gps или local. Использую gps.', self.mode)
            self.mode = 'gps'

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if not os.path.isdir(os.path.join(pkg_dir, 'web')):
            import rospkg
            pkg_dir = rospkg.RosPack().get_path('fire_detection_ros_rknn_slop2')
            
        web_dir = P('web_dir', os.path.join(pkg_dir, 'web'))
        tiles_dir = P('tiles_dir', '') or None
        if tiles_dir:
            tiles_dir = os.path.expanduser(tiles_dir)
            if not os.path.isdir(tiles_dir):
                rospy.logwarn('tiles_dir %s не найден — локальные тайлы отключены', tiles_dir)
                tiles_dir = None

        geo = Geolocator(
            mode=self.mode,
            hfov_deg=P('camera/hfov_deg', 56.0), vfov_deg=P('camera/vfov_deg', 42.0),
            mount_pitch_deg=P('camera/mount_pitch_deg', -90.0),
            mount_roll_deg=P('camera/mount_roll_deg', 0.0),
            mount_yaw_deg=P('camera/mount_yaw_deg', 0.0),
            flip_x=P('camera/flip_x', False), flip_y=P('camera/flip_y', False),
            stabilized=P('camera/stabilized', False),
            min_agl_m=P('min_agl_m', 3.0), max_range_m=P('max_range_m', 1500.0),
            pos_timeout=P('local/position_timeout', 1.0))

        session = time.strftime('%Y%m%d_%H%M%S')
        db_path = P('db_path', '~/fire_logs/detections.db')
        self.store = EventStore(db_path, session, mode=self.mode)
        
        tracker = ObjectTracker(self.store, mode=self.mode,
                                assoc_radius_m=P('tracker/assoc_radius_m', 20.0),
                                range_factor=P('tracker/range_factor', 0.05),
                                min_hits=P('tracker/min_hits', 2))

        # =========================================================================
        # ВИДЕОПОТОКИ (ИСПРАВЛЕНО: self.core создается ДО создания VideoRelay)
        # =========================================================================
        streams, labels, self.relays = [], {}, []
        
        # 1. Тепловизор (основной поток)
        if P('video/enabled', True):
            streams.append('thermal')
            labels['thermal'] = P('video/label', 'Тепловизор с детекциями')
            
        # 2. Собираем конфигурации всех дополнительных камер (включая курсовую)
        cams = []
        if P('course_camera/enabled', True):
            cams.append({
                'name': 'course',
                'topic': P('course_camera/topic', '/course_camera/image_raw'),
                'label': P('course_camera/label', 'Курсовая камера (USB)'),
                'max_fps': float(P('course_camera/max_fps', 25.0)),
                'max_width': int(P('course_camera/max_width', 640)),
                'jpeg_quality': int(P('course_camera/jpeg_quality', 75))
            })
            
        # Добавляем любые другие камеры из списка ~cameras
        extra_cams = P('cameras', [])
        if isinstance(extra_cams, list):
            for c in extra_cams:
                if c.get('enabled', True) and c.get('topic') and c.get('name') != 'course':
                    cams.append(c)

        # Обрабатываем список камер, формируя streams и labels
        for i, c in enumerate(cams):
            name = ''.join(ch for ch in str(c.get('name', 'cam%d' % i)) if ch.isalnum() or ch in '_-')
            if name in streams:
                name += str(i)
            streams.append(name)
            labels[name] = c.get('label', name)
            c['name'] = name  # сохраняем уникальное имя обратно в конфиг

        # 3. СОЗДАЕМ DashboardCore (ТЕПЕРЬ self.core существует!)
        self.core = DashboardCore(geo, self.store, tracker,
                                  max_tel_age=P('max_telemetry_age', 0.5),
                                  telemetry_hz=P('telemetry_hz', 5.0),
                                  video_streams=streams)

        # 4. Создаем VideoRelay для всех собранных камер (теперь self.core доступен)
        if 'thermal' in streams:
            self.relays.append(VideoRelay(self.core, 'thermal', P('video/topic', '/fire/annotated'),
                                          max_fps=P('video/max_fps', 10.0), max_width=0))
                                          
        for c in cams:
            self.relays.append(VideoRelay(
                self.core, c['name'], c['topic'],
                max_fps=c['max_fps'],
                max_width=c['max_width'],
                quality=c['jpeg_quality']
            ))

        # план площадки (только локальный режим)
        localmap = None
        if self.mode == 'local':
            data_dir = P('local/map_dir', os.path.join(os.path.dirname(os.path.expanduser(db_path)), 'localmap'))
            localmap = LocalMapStore(data_dir)
            localmap.ros_margin = P('local/aruco_image_margin', 200)
            self._aruco_image_topic = P('local/aruco_image_topic', '/aruco_map/image')
            self._aruco_map_param = P('local/aruco_map_param', '/aruco_map/map')
            if self._aruco_image_topic:
                localmap.ros_loader = self._load_aruco_from_ros

        class_names = rospy.get_param('/fire_detector_rknn_slop2/class_names', 
                                      rospy.get_param('/fire_detector/class_names', ['fire', 'car']))
        class_colors = P('class_colors', {})
        if not class_colors:
            bgr = rospy.get_param('/fire_detector_rknn_slop2/class_colors', 
                                  rospy.get_param('/fire_detector/class_colors', [[0, 0, 255], [0, 255, 255]]))
            class_colors = {n: bgr_to_hex(bgr[i % len(bgr)]) for i, n in enumerate(class_names)}

        ui_config = {
            'vehicle': P('vehicle_name', 'ВТОЛ' if self.mode == 'gps' else 'Тестовый дрон'),
            'tile_url': P('map/tile_url', 'https://tile.openstreetmap.org/{z}/{x}/{y}.png'),
            'tile_attribution': P('map/tile_attribution', '© OpenStreetMap contributors'),
            'tile_max_zoom': P('map/max_zoom', 19),
            'satellite_url': P('map/satellite_url',
                               'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'),
            'center': P('map/center', [55.751, 37.618]),
            'zoom': P('map/zoom', 16),
            'class_labels': P('class_labels', {'fire': 'Очаг возгорания', 'car': 'Автомобиль'}),
            'class_colors': class_colors,
            'stale_after_s': P('stale_after_s', 60),
            'video_labels': labels,
        }

        host, port = P('host', '0.0.0.0'), int(P('port', 8080))
        self.server = DashboardServer(self.core, host, port, web_dir, ui_config, tiles_dir, localmap)
        self.server.start()

        # ---- телеметрия ----
        self.attitude_from_pose = False
        self._last_nav_stamp = None
        self._gps_info_seen = False
        self._have_rel_alt = False
        self.ground_alt_amsl = P('ground_alt_amsl', None)

        if self.mode == 'gps':
            rospy.Subscriber('/mavlink/attitude', Imu, self._on_att, queue_size=20)
            rospy.Subscriber('/mavlink/velocity', TwistStamped, self._on_vel, queue_size=20)
            rospy.Subscriber('/mavlink/navsat', NavSatFix, self._on_nav, queue_size=20)
            rospy.Subscriber('/mavlink/rel_alt', Float32, self._on_rel_alt, queue_size=20)
            rospy.Subscriber('/mavlink/gps_info', UInt16MultiArray, self._on_gps_info, queue_size=5)
            rospy.Timer(rospy.Duration(10.0), self._check_alt_source, oneshot=True)
        else:
            self._setup_local(P)

        rospy.Subscriber('/fire/detections', String, self._on_det, queue_size=50)

        if self.relays:
            rospy.Timer(rospy.Duration(1.0), lambda _: [r.tick() for r in self.relays])
            
        rospy.Timer(rospy.Duration(2.0), lambda _: self.store.flush())
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo('Веб-интерфейс: http://%s:%d  режим=%s  журнал=%s  сессия=%s',
                      host, port, self.mode, self.store.path, session)
        rospy.loginfo('Видеопотоки: %s', ', '.join(streams))

    # ================= локальный режим =================
    def _setup_local(self, P):
        self.local_source = P('local/source', 'tf')
        self.pose_frame = P('local/pose_frame', 'enu')
        rot = math.radians(P('local/frame_rotation_deg', 0.0))
        self._rot = (math.cos(rot), math.sin(rot), math.degrees(rot))
        off = P('local/frame_offset', [0.0, 0.0])
        self._off = (float(off[0]), float(off[1]))
        self._pos_seen = False
        self._start_t = rospy.Time.now().to_sec()
        self._markers_topic, self._markers_t = '', None

        if self.local_source == 'tf':
            self.attitude_from_pose = True
            import tf2_ros
            self._tf2 = tf2_ros
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.tf_map = P('local/tf/map_frame', 'aruco_map')
            self.tf_body = P('local/tf/body_frame', 'base_link')
            self.tf_odom = P('local/tf/odom_frame', 'map') or None
            self._tf_last = 0.0
            self._tf_err = None
            rospy.Timer(rospy.Duration(1.0 / float(P('local/tf/rate_hz', 20.0))), self._poll_tf)
        elif self.local_source == 'pose':
            self.attitude_from_pose = True
            self.pose_topic = P('local/pose_topic', '/aruco/pose')
            self._pose_sub = None
            rospy.Timer(rospy.Duration(1.0), self._try_subscribe_pose)
        else:
            rospy.Subscriber('/mavlink/attitude', Imu, self._on_att, queue_size=20)
            rospy.Subscriber('/mavlink/local_position', PointStamped, self._on_local_ned, queue_size=20)

        self._markers_topic = P('local/markers_topic', '')
        self._markers_t = None
        if self._markers_topic:
            rospy.Subscriber(self._markers_topic, rospy.AnyMsg, self._on_markers_seen, queue_size=5)

        vel_topic = P('local/velocity_topic', '/mavlink/velocity')
        self._vel_enu = P('local/velocity_frame', 'ned') == 'enu'
        if vel_topic:
            rospy.Subscriber(vel_topic, TwistStamped, self._on_vel, queue_size=20)
        rospy.Timer(rospy.Duration(10.0), self._check_local_source, oneshot=True)

    def _on_markers_seen(self, _msg):
        self._markers_t = rospy.Time.now().to_sec()

    def _to_plan(self, x, y, yaw_ned=None):
        c, s, deg = self._rot
        px = c * x - s * y + self._off[0]
        py = s * x + c * y + self._off[1]
        yaw = None if yaw_ned is None else (yaw_ned - deg) % 360.0
        return px, py, yaw

    def _feed_pose(self, stamp, p, q, frame, ref_t=None):
        roll, pitch, yaw = quat_to_euler_deg(q)
        if frame == 'ned':
            x, y, agl, roll_n, pitch_n, yaw_n = p[1], p[0], -p[2], roll, pitch, yaw
        else:
            x, y, agl = p[0], p[1], p[2]
            roll_n, pitch_n, yaw_n = roll, -pitch, (90.0 - yaw) % 360.0
        px, py, yaw_plan = self._to_plan(x, y, yaw_n)
        self._pos_seen = True
        fields = dict(x=px, y=py, agl=agl, pos_t=stamp, roll=roll_n, pitch=pitch_n, yaw=yaw_plan)
        if self._markers_topic:
            ref_t = self._markers_t if self._markers_t is not None else self._start_t
            if ref_t is not None:
                fields['ref_t'] = ref_t
        self.core.on_telemetry(stamp, **fields)

    def _load_aruco_from_ros(self):
        import cv2
        import img_convert
        try:
            msg = rospy.wait_for_message(self._aruco_image_topic, Image, timeout=5.0)
        except rospy.ROSException:
            raise ValueError('за 5 с не пришло изображение из %s' % self._aruco_image_topic)
        bgr = img_convert.image_msg_to_bgr(msg)
        ok, png = cv2.imencode('.png', bgr)
        if not ok:
            raise ValueError('не удалось закодировать изображение карты')
        path = rospy.get_param(self._aruco_map_param, None)
        if not path or not os.path.isfile(path):
            raise ValueError('файл карты маркеров не найден')
        with open(path, encoding='utf-8', errors='replace') as f:
            text = f.read()
        return png.tobytes(), bgr.shape[1], bgr.shape[0], 'image/png', text, os.path.basename(path)

    @staticmethod
    def _qmul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz)

    def _qrot(self, q, v):
        x, y, z, w = self._qmul(self._qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
        return x, y, z

    @staticmethod
    def _tf_parts(t):
        tr, r = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w)

    def _frame_exists(self, frame):
        try:
            return self.tf_buffer._frameExists(frame)
        except AttributeError:
            return ('%s:' % frame) in self.tf_buffer.all_frames_as_yaml()

    def _poll_tf(self, _):
        tf2 = self._tf2
        errors = (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException)
        try:
            if self.tf_odom:
                ref = self.tf_buffer.lookup_transform(self.tf_map, self.tf_odom, rospy.Time(0))
                body = self.tf_buffer.lookup_transform(self.tf_odom, self.tf_body, rospy.Time(0))
                (tr, rr), (tb, rb) = self._tf_parts(ref), self._tf_parts(body)
                rp = self._qrot(rr, tb)
                p = (tr[0] + rp[0], tr[1] + rp[1], tr[2] + rp[2])
                q = self._qmul(rr, rb)
                stamp = body.header.stamp.to_sec()
            else:
                t = self.tf_buffer.lookup_transform(self.tf_map, self.tf_body, rospy.Time(0))
                p, q = self._tf_parts(t)
                stamp = t.header.stamp.to_sec()
        except errors as e:
            if self.tf_odom and isinstance(e, (tf2.LookupException, tf2.ConnectivityException)) \
               and self._frame_exists(self.tf_map) and not self._frame_exists(self.tf_odom):
                rospy.logwarn('Фрейм %s не найден, беру %s -> %s напрямую', self.tf_odom, self.tf_map, self.tf_body)
                self.tf_odom = None
                return
            msg = str(e).split('\n')[0]
            if msg != self._tf_err:
                rospy.logwarn_throttle(10, 'TF %s -> %s: %s' % (self.tf_map, self.tf_body, msg))
            self._tf_err = msg
            return
        if stamp <= 0:
            stamp = rospy.Time.now().to_sec()
        if stamp == self._tf_last:
            return
        self._tf_last = stamp
        self._tf_err = None
        self._feed_pose(stamp, p, q, 'enu')

    def _on_local_ned(self, msg):
        self._pos_seen = True
        stamp = self._stamp(msg.header)
        px, py, _ = self._to_plan(msg.point.y, msg.point.x)
        self.core.on_telemetry(stamp, x=px, y=py, agl=-msg.point.z, pos_t=stamp)

    def _try_subscribe_pose(self, timer):
        if self._pose_sub is not None:
            return
        try:
            import rostopic
            ttype, _, _ = rostopic.get_topic_type(self.pose_topic, blocking=False)
        except Exception:
            ttype = None
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        classes = {'geometry_msgs/PoseStamped': PoseStamped,
                   'geometry_msgs/PoseWithCovarianceStamped': PoseWithCovarianceStamped}
        try:
            from nav_msgs.msg import Odometry
            classes['nav_msgs/Odometry'] = Odometry
        except ImportError:
            pass
        if ttype in classes:
            self._pose_sub = rospy.Subscriber(self.pose_topic, classes[ttype], self._on_pose, queue_size=20)
            rospy.loginfo('Позиция: %s (%s, система %s)', self.pose_topic, ttype, self.pose_frame)
            timer.shutdown()
        elif ttype is not None:
            rospy.logerr('Топик %s имеет тип %s; нужен PoseStamped, PoseWithCovarianceStamped или Odometry',
                         self.pose_topic, ttype)
            timer.shutdown()

    def _on_pose(self, msg):
        pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
        p, q = pose.position, pose.orientation
        self._feed_pose(self._stamp(msg.header), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.pose_frame)

    def _check_local_source(self, _):
        if not self._pos_seen:
            src = {'tf': 'TF %s -> %s' % (getattr(self, 'tf_map', ''), getattr(self, 'tf_body', '')),
                   'pose': getattr(self, 'pose_topic', '')}.get(self.local_source, '/mavlink/local_position')
            rospy.logwarn('Локальный режим: за 10 с не получено ни одной позиции из %s.', src)

    # ================= GPS-режим =================
    @staticmethod
    def _stamp(header):
        s = header.stamp.to_sec()
        return s if s > 0 else rospy.Time.now().to_sec()

    def _on_nav(self, msg):
        fields = {'lat': msg.latitude, 'lon': msg.longitude, 'amsl': msg.altitude}
        if not self._have_rel_alt and self.ground_alt_amsl is not None:
            fields['agl'] = msg.altitude - float(self.ground_alt_amsl)
        if not self._gps_info_seen:
            fields['fix'] = 3 if msg.status.status >= NavSatStatus.STATUS_FIX else 0
        self._last_nav_stamp = self._stamp(msg.header)
        self.core.on_telemetry(self._last_nav_stamp, **fields)

    def _aux_stamp(self):
        now = rospy.Time.now().to_sec()
        if self._last_nav_stamp is not None and now - self._last_nav_stamp < 0.2:
            return self._last_nav_stamp
        return now

    def _on_rel_alt(self, msg):
        self._have_rel_alt = True
        self.core.on_telemetry(self._aux_stamp(), agl=float(msg.data))

    def _on_gps_info(self, msg):
        if len(msg.data) >= 2:
            self._gps_info_seen = True
            self.core.on_telemetry(self._aux_stamp(), fix=int(msg.data[0]), sats=int(msg.data[1]))

    def _check_alt_source(self, _):
        if not self._have_rel_alt and self.ground_alt_amsl is None:
            rospy.logwarn('Нет /mavlink/rel_alt и не задан ~ground_alt_amsl: высота AGL неизвестна.')

    # ================= общее =================
    def _on_att(self, msg):
        roll, pitch, yaw = quat_to_euler_deg(msg.orientation)
        if self.mode == 'local':
            yaw = self._to_plan(0.0, 0.0, yaw)[2]
        self.core.on_telemetry(self._stamp(msg.header), roll=roll, pitch=pitch, yaw=yaw)

    def _on_vel(self, msg):
        v = msg.twist.linear
        if self.mode == 'local' and self._vel_enu:
            vn, ve, vd = v.y, v.x, -v.z
        else:
            vn, ve, vd = v.x, v.y, v.z
        self.core.on_telemetry(self._stamp(msg.header), vn=vn, ve=ve, vd=vd)

    def _on_det(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            rospy.logwarn_throttle(5, 'Некорректный JSON в /fire/detections')
            return
        try:
            for rec in self.core.on_detections(payload):
                if rec['geo_status'] != 'ok':
                    continue
                if self.mode == 'gps':
                    rospy.loginfo_throttle(1, '%s %.2f -> %.6f, %.6f (%.0f м)', rec['cls'], rec['conf'],
                                           rec['lat'], rec['lon'], rec['ground_dist'])
                else:
                    rospy.loginfo_throttle(1, '%s %.2f -> X=%.2f Y=%.2f м', rec['cls'], rec['conf'],
                                           rec['x'], rec['y'])
        except Exception as e:
            rospy.logwarn_throttle(5, 'Ошибка обработки детекций: %r' % e)

    def _shutdown(self):
        self.server.stop()
        self.store.close()


if __name__ == '__main__':
    try:
        GeoDashboardNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
