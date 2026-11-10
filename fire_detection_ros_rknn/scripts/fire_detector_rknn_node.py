#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fire Detector Node with RKNN NPU (Orange Pi 5 / RK3588)
Финальная версия с исправленным логированием, фильтром площади и корректным JSON.
"""

import sys
import os
import logging

# ============================================================
# 1. ФИКС БАГА ЛОГИРОВАНИЯ ROS NOETIC НА ARM64
# ============================================================
os.environ['ROS_PYTHON_LOG_CONFIG'] = '/dev/null'

import rospy
import rospy.core
import rosgraph.roslogging

# Заменяем сломанные функции настройки ROS на пустые заглушки
rospy.core.configure_logging = lambda *args, **kwargs: None
rosgraph.roslogging.configure_logging = lambda *args, **kwargs: None

# Настраиваем простой и надежный логгер Python
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s',
    stream=sys.stdout
)

# Перенаправляем вызовы rospy.log... на стандартный logging
rospy.loginfo = logging.info
rospy.logwarn = logging.warning
rospy.logerr = logging.error
rospy.logfatal = logging.critical
rospy.logdebug = logging.debug
# ============================================================

import cv2
import numpy as np
import json
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
        
        # Чтение параметров
        self.model_path = rospy.get_param('~model_path', 
            '/home/orangepi/catkin_ws/src/fire_detection_ros_rknn/models/fire-cars_RK3588_640x640.rknn')
        self.input_width = rospy.get_param('~input_width', 640)
        self.input_height = rospy.get_param('~input_height', 640)
        self.conf_thresh = rospy.get_param('~conf_thresh', 0.60)  # Повышен по умолчанию для тепловизора
        self.iou_thresh = rospy.get_param('~iou_thresh', 0.60)
        self.process_hz = rospy.get_param('~process_hz', 10)
        self.use_all_npu_cores = rospy.get_param('~use_all_npu_cores', True)
        self.verbose = rospy.get_param('~verbose', False)
        
        # КРИТИЧЕСКИ ВАЖНЫЙ ПАРАМЕТР для тепловизоров: фильтр мелких детектов
        self.min_detection_area = rospy.get_param('~min_detection_area', 800)
        
        self.class_names = rospy.get_param('~class_names', ['fire', 'car'])
        self.class_colors = rospy.get_param('~class_colors', [[0, 0, 255], [0, 255, 255]])
        
        print(f"[INFO]Загрузка RKNN модели: {self.model_path}")
        self.rknn = RKNNLite()
        
        ret = self.rknn.load_rknn(self.model_path)
        if ret != 0:
            print(f"[CRITICAL] Ошибка загрузки RKNN модели: {ret}")
            sys.exit(1)
        
        if self.use_all_npu_cores:
            core_mask = RKNNLite.NPU_CORE_0_1_2
            print("[INFO] Используем все 3 ядра NPU")
        else:
            core_mask = RKNNLite.NPU_CORE_0
            print("[INFO] Используем 1 ядро NPU")
        
        ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            print(f"[CRITICAL]Ошибка инициализации NPU runtime: {ret}")
            sys.exit(1)
        
        print("[INFO] NPU инициализирован успешно")
        
        # ROS publishers
        self.pub_det = rospy.Publisher('/fire/detections', String, queue_size=10)
        self.pub_img = rospy.Publisher('/fire/annotated', CompressedImage, queue_size=1)
        
        # Счётчики статистики
        self.frame_count = 0
        self.det_count = 0
        
        # ROS subscriber
        rospy.Subscriber('/thermal/image_raw', CompressedImage, self._on_image)
        print(f"[INFO] Fire Detector RKNN готов (FPS target: {self.process_hz}, Min Area: {self.min_detection_area})")
    
    def _on_image(self, msg: CompressedImage):
        try:
            # Декодирование JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            
            h, w = frame.shape[:2]
            
            # Предобработка для RKNN (uint8, без нормализации, как требует INT8 модель)
            input_img = cv2.resize(frame, (self.input_width, self.input_height))
            input_data = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            input_data = np.expand_dims(input_data, axis=0)
            
            # Инференс на NPU
            outputs = self.rknn.inference(inputs=[input_data])
            
            # Постобработка
            detections = self._postprocess(outputs[0], w, h)
            
            # Публикация детектов в формате JSON (БЕЗ det_msg.header, так как у String его нет)
            det_msg = String()
            det_msg.data = json.dumps({
                'timestamp': msg.header.stamp.to_sec(),
                'frame_width': w,
                'frame_height': h,
                'detections_count': len(detections),
                'detections': detections
            }, ensure_ascii=False)
            self.pub_det.publish(det_msg)
            
            # Аннотированное изображение (с рамками)
            self._draw(frame, detections)
            ann_msg = CompressedImage()
            ann_msg.header = msg.header
            ann_msg.format = "jpeg"
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            ann_msg.data = buf.tobytes()
            self.pub_img.publish(ann_msg)
            
            # Статистика
            self.frame_count += 1
            if detections:
                self.det_count += len(detections)
            
            if self.verbose and detections:
                print(f"[INFO] Кадр {self.frame_count}: найдено {len(detections)} объектов")
            
            if self.frame_count % 100 == 0:
                print(f"[INFO] Статистика: кадров={self.frame_count}, детектов={self.det_count}, "
                      f"среднее={self.det_count/max(1,self.frame_count):.2f}/кадр")
        
        except Exception as e:
            print(f"[WARNING] Ошибка обработки кадра: {e}")
    
    def _postprocess(self, output, orig_w, orig_h):
        """
        Постобработка выхода YOLOv8 для RKNN с фильтром минимальной площади.
        """
        output = np.squeeze(output)
        
        # Транспонирование если нужно (из [6, 8400] в [8400, 6])
        if output.shape[0] < output.shape[1]:
            output = output.T
        
        # Масштабирование координат из 640x640 в оригинальное разрешение
        xs = orig_w / self.input_width
        ys = orig_h / self.input_height
        
        boxes, confs, cls_ids = [], [], []
        
        for i in range(output.shape[0]):
            cx, cy, w, h = output[i, 0:4]
            class_scores = output[i, 4:]
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])
            
            if confidence > self.conf_thresh:
                left = int((cx - w / 2) * xs)
                top = int((cy - h / 2) * ys)
                width = int(w * xs)
                height = int(h * ys)
                
                # === КРИТИЧЕСКИЙ ФИЛЬТР: отбрасываем слишком мелкие рамки (тепловой шум) ===
                if (width * height) < self.min_detection_area:
                    continue
                
                boxes.append([left, top, width, height])
                confs.append(confidence)
                cls_ids.append(class_id)
        
        if not boxes:
            return []
        
        # NMS (Non-Maximum Suppression)
        idx = cv2.dnn.NMSBoxes(boxes, confs, self.conf_thresh, self.iou_thresh)
        if len(idx) == 0:
            return []
        
        flat = idx.flatten() if hasattr(idx, 'flatten') else idx
        
        # Формируем результат в виде списка словарей (для JSON)
        results = []
        for i in flat:
            results.append({
                'cx': float(boxes[i][0] + boxes[i][2] / 2),
                'cy': float(boxes[i][1] + boxes[i][3] / 2),
                'w': float(boxes[i][2]),
                'h': float(boxes[i][3]),
                'class_id': int(cls_ids[i]),
                'class_name': self.class_names[cls_ids[i]] if cls_ids[i] < len(self.class_names) else f'class_{cls_ids[i]}',
                'conf': float(confs[i])
            })
        
        return results
    
    def _draw(self, frame, detections):
        """Отрисовка bounding boxes на кадре"""
        for d in detections:
            x1 = int(d['cx'] - d['w'] / 2)
            y1 = int(d['cy'] - d['h'] / 2)
            x2 = int(d['cx'] + d['w'] / 2)
            y2 = int(d['cy'] + d['h'] / 2)
            
            cls_id = d['class_id']
            conf = d['conf']
            color = self.class_colors[cls_id % len(self.class_colors)]
            label = f"{d['class_name']} {conf:.2f}"
            
            # Рисуем рамку
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Рисуем подпись на контрастном фоне
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
        # Безопасное освобождение ресурсов NPU
        if node is not None and hasattr(node, 'rknn'):
            node.rknn.release()
            print("[INFO] NPU освобождён")
