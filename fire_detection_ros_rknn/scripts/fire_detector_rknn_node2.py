#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fire Detector Node with RKNN NPU (Orange Pi 5 / RK3588)
- Инференс в отдельном потоке, всегда на самом свежем кадре (без накопления задержки)
- Векторная постобработка YOLOv8
- Нормализация входа /255 на CPU (модель сконвертирована без mean/std)
"""

import sys
import os
import time
import json
import logging
import threading

# ============================================================
# ФИКС ЛОГИРОВАНИЯ ROS NOETIC НА ARM64
# ============================================================
os.environ['ROS_PYTHON_LOG_CONFIG'] = '/dev/null'

import rospy
import rospy.core
import rosgraph.roslogging

rospy.core.configure_logging = lambda *args, **kwargs: None
rosgraph.roslogging.configure_logging = lambda *args, **kwargs: None

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s', stream=sys.stdout)
rospy.loginfo = logging.info
rospy.logwarn = logging.warning
rospy.logerr = logging.error
rospy.logfatal = logging.critical
rospy.logdebug = logging.debug
# ============================================================

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from rknnlite.api import RKNNLite
except ImportError:
    print("[CRITICAL] rknnlite не установлен! pip install rknn-toolkit-lite2")
    sys.exit(1)


class FireDetectorRKNNNode:
    def __init__(self):
        rospy.init_node('fire_detector_rknn_node')

        self.model_path = rospy.get_param('~model_path',
            '/home/orangepi/catkin_ws/src/fire_detection_ros_rknn/models/fire-cars_RK3588_640x640.rknn')
        self.input_width = rospy.get_param('~input_width', 640)
        self.input_height = rospy.get_param('~input_height', 640)
        self.conf_thresh = rospy.get_param('~conf_thresh', 0.25)
        self.iou_thresh = rospy.get_param('~iou_thresh', 0.45)
        self.min_detection_area = rospy.get_param('~min_detection_area', 0)
        self.use_all_npu_cores = rospy.get_param('~use_all_npu_cores', True)
        self.jpeg_quality = rospy.get_param('~jpeg_quality', 80)
        self.stats_every = rospy.get_param('~stats_every', 100)
        self.verbose = rospy.get_param('~verbose', False)

        self.class_names = rospy.get_param('~class_names', ['fire', 'car'])
        self.class_colors = [tuple(c) for c in
                             rospy.get_param('~class_colors', [[0, 0, 255], [0, 255, 255]])]

        # ---------- NPU ----------
        print(f"[INFO] Загрузка RKNN модели: {self.model_path}")
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(self.model_path) != 0:
            print("[CRITICAL] Ошибка загрузки RKNN модели")
            sys.exit(1)

        core_mask = RKNNLite.NPU_CORE_0_1_2 if self.use_all_npu_cores else RKNNLite.NPU_CORE_0
        if self.rknn.init_runtime(core_mask=core_mask) != 0:
            print("[CRITICAL] Ошибка инициализации NPU runtime")
            sys.exit(1)
        print("[INFO] NPU инициализирован")

        # ---------- состояние ----------
        self._lock = threading.Lock()
        self._new_frame = threading.Event()
        self._latest_msg = None

        self.frame_count = 0
        self.det_count = 0
        self.dropped = 0
        self._t_infer = 0.0
        self._t_total = 0.0
        self._latency = 0.0

        # ---------- ROS ----------
        self.pub_det = rospy.Publisher('/fire/detections', String, queue_size=1)
        self.pub_img = rospy.Publisher('/fire/annotated', CompressedImage, queue_size=1)

        # queue_size=1 + большой buff_size: без этого rospy копит старые кадры в сокете
        rospy.Subscriber('/thermal/image_raw', CompressedImage, self._on_image,
                         queue_size=1, buff_size=2**24, tcp_nodelay=True)

        threading.Thread(target=self._worker, daemon=True).start()
        print(f"[INFO] Fire Detector RKNN готов (conf={self.conf_thresh}, iou={self.iou_thresh}, "
              f"min_area={self.min_detection_area})")

    # ---------- приём: только запоминаем последний кадр ----------
    def _on_image(self, msg):
        with self._lock:
            if self._latest_msg is not None:
                self.dropped += 1          # предыдущий не успели обработать — выбрасываем
            self._latest_msg = msg
        self._new_frame.set()

    # ---------- обработка в отдельном потоке ----------
    def _worker(self):
        while not rospy.is_shutdown():
            if not self._new_frame.wait(0.5):
                continue
            self._new_frame.clear()
            with self._lock:
                msg, self._latest_msg = self._latest_msg, None
            if msg is None:
                continue
            try:
                self._process(msg)
            except Exception as e:
                print(f"[WARNING] Ошибка обработки кадра: {e!r}")

    def _process(self, msg):
        t0 = time.perf_counter()

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]

        # Предобработка: RGB, NHWC, float32 0..1 (модель без встроенной нормализации)
        img = cv2.resize(frame, (self.input_width, self.input_height))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) * (1.0 / 255.0)
        input_data = np.expand_dims(img, axis=0)

        t1 = time.perf_counter()
        outputs = self.rknn.inference(inputs=[input_data])
        t2 = time.perf_counter()

        detections = self._postprocess(outputs[0], w, h)

        # ---------- детекции (JSON) ----------
        det_msg = String()
        det_msg.data = json.dumps({
            'timestamp': msg.header.stamp.to_sec(),
            'frame_width': w,
            'frame_height': h,
            'detections_count': len(detections),
            'detections': detections,
        }, ensure_ascii=False)
        self.pub_det.publish(det_msg)

        # ---------- аннотированное изображение ----------
        if self.pub_img.get_num_connections() > 0:
            self._draw(frame, detections)
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                ann = CompressedImage()
                ann.header = msg.header
                ann.format = "jpeg"
                ann.data = buf.tobytes()
                self.pub_img.publish(ann)

        # ---------- статистика ----------
        t3 = time.perf_counter()
        self.frame_count += 1
        self.det_count += len(detections)
        self._t_infer += (t2 - t1)
        self._t_total += (t3 - t0)
        if msg.header.stamp.to_sec() > 0:
            self._latency += (rospy.Time.now() - msg.header.stamp).to_sec()

        if self.verbose and detections:
            print(f"[INFO] Кадр {self.frame_count}: {len(detections)} объектов")

        if self.frame_count % self.stats_every == 0:
            n = self.stats_every
            print(f"[INFO] Статистика: кадров={self.frame_count}, детект/кадр={self.det_count / self.frame_count:.2f}, "
                  f"NPU={1000 * self._t_infer / n:.1f} мс, весь кадр={1000 * self._t_total / n:.1f} мс, "
                  f"задержка от камеры={1000 * self._latency / n:.0f} мс, пропущено кадров={self.dropped}")
            self._t_infer = self._t_total = self._latency = 0.0

    # ---------- постобработка YOLOv8 [1, 4+nc, 8400] (векторная) ----------
    def _postprocess(self, output, orig_w, orig_h):
        out = np.squeeze(output)
        if out.shape[0] < out.shape[1]:
            out = out.T                                  # [8400, 4+nc]

        scores = out[:, 4:]
        cls_ids = scores.argmax(axis=1)
        confs = scores.max(axis=1)
        keep = confs > self.conf_thresh
        if not keep.any():
            return []
        b, cls_ids, confs = out[keep, :4], cls_ids[keep], confs[keep]

        xs = orig_w / self.input_width
        ys = orig_h / self.input_height
        bw = b[:, 2] * xs
        bh = b[:, 3] * ys
        left = b[:, 0] * xs - bw / 2
        top = b[:, 1] * ys - bh / 2

        if self.min_detection_area > 0:
            big = (bw * bh) >= self.min_detection_area
            if not big.any():
                return []
            left, top, bw, bh = left[big], top[big], bw[big], bh[big]
            cls_ids, confs = cls_ids[big], confs[big]

        boxes = np.stack([left, top, bw, bh], axis=1).astype(int).tolist()
        idx = cv2.dnn.NMSBoxes(boxes, confs.astype(float).tolist(), self.conf_thresh, self.iou_thresh)
        if len(idx) == 0:
            return []

        results = []
        for i in np.array(idx).flatten():
            x, y, bw_i, bh_i = boxes[i]
            c = int(cls_ids[i])
            results.append({
                'cx': float(x + bw_i / 2), 'cy': float(y + bh_i / 2),
                'w': float(bw_i), 'h': float(bh_i),
                'class_id': c,
                'class_name': self.class_names[c] if c < len(self.class_names) else f'class_{c}',
                'conf': float(confs[i]),
            })
        return results

    def _draw(self, frame, detections):
        for d in detections:
            x1, y1 = int(d['cx'] - d['w'] / 2), int(d['cy'] - d['h'] / 2)
            x2, y2 = int(d['cx'] + d['w'] / 2), int(d['cy'] + d['h'] / 2)
            color = self.class_colors[d['class_id'] % len(self.class_colors)]
            label = f"{d['class_name']} {d['conf']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


if __name__ == '__main__':
    node = None
    try:
        node = FireDetectorRKNNNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"[ERROR] Критическая ошибка при запуске: {e}")
    finally:
        if node is not None and hasattr(node, 'rknn'):
            node.rknn.release()
            print("[INFO] NPU освобождён")
