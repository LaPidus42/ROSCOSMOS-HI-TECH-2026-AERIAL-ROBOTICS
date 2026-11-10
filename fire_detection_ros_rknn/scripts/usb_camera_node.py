#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB Camera Node — захват видео с USB-камеры (UVC) и публикация в JPEG (CompressedImage).

Параметры (~):
  device        путь к камере; лучше постоянный /dev/v4l/by-path/...-video-index0
  width, height разрешение
  target_fps    частота публикации
  fourcc        формат с камеры: MJPG (сжатый, мало занимает полосу USB) или YUYV (несжатый)
  jpeg_quality  качество публикуемого JPEG
  topic         топик публикации
  read_timeout_s сколько ждать первый кадр при открытии, прежде чем переподключаться

Почему MJPG по умолчанию: несжатый YUYV 640x480 резервирует большую часть полосы USB 2.0.
Если на том же USB-контроллере уже работает другая камера (тепловизор), веб-камере не хватает
полосы и кадры не приходят (в логе OpenCV: select() timeout).
"""
import time

import cv2
import rospy
from sensor_msgs.msg import CompressedImage


class USBCameraNode:
    def __init__(self):
        rospy.init_node('usb_camera_node')

        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = int(rospy.get_param('~width', 640))
        self.height = int(rospy.get_param('~height', 480))
        self.fps = float(rospy.get_param('~target_fps', 15))
        self.fourcc = str(rospy.get_param('~fourcc', 'MJPG')).upper()[:4]
        self.jpeg_q = int(rospy.get_param('~jpeg_quality', 80))
        self.topic = rospy.get_param('~topic', '/course_camera/image_raw')
        self.read_timeout = float(rospy.get_param('~read_timeout_s', 5.0))

        self.pub = rospy.Publisher(self.topic, CompressedImage, queue_size=1)
        self.cap = None
        self._fail_logged = False

        self.rate = rospy.Rate(self.fps)
        rospy.on_shutdown(self._cleanup)
        rospy.loginfo('USB Camera node: %s, %dx%d @ %.0f к/с, формат %s',
                      self.device, self.width, self.height, self.fps, self.fourcc)

    # ---------- открытие камеры ----------
    def _open(self):
        """Открывает камеру и ждёт первый кадр не дольше read_timeout. True — камера работает."""
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            if not self._fail_logged:
                rospy.logerr('Не удалось открыть %s (камера не подключена или занята другой программой)',
                             self.device)
                self._fail_logged = True
            cap.release()
            return False

        # Формат задаём ДО разрешения: иначе драйвер может выбрать размер под YUYV
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Что на самом деле согласовал драйвер
        got = int(cap.get(cv2.CAP_PROP_FOURCC))
        got_fourcc = ''.join(chr((got >> (8 * i)) & 0xFF) for i in range(4)).strip('\x00') or '?'
        got_w, got_h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Ждём первый кадр ограниченное время (а не 10 кадров по 10 с таймаута драйвера)
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.read_timeout and not rospy.is_shutdown():
            ok, frame = cap.read()
            if ok and frame is not None:
                self.cap = cap
                self._fail_logged = False
                rospy.loginfo('Camera ready: %s (%s %dx%d)', self.device, got_fourcc, got_w, got_h)
                return True
        cap.release()
        if not self._fail_logged:
            rospy.logwarn('Камера %s открылась (%s %dx%d), но не отдаёт кадры. Частая причина — не хватает '
                          'полосы USB: другая камера на том же контроллере. Попробуйте fourcc=MJPG, меньшее '
                          'разрешение/частоту или другой USB-разъём.', self.device, got_fourcc, got_w, got_h)
            self._fail_logged = True
        return False

    # ---------- основной цикл ----------
    def run(self):
        while not rospy.is_shutdown():
            if self.cap is None:
                if not self._open():
                    rospy.sleep(2.0)            # пауза перед новой попыткой, без «молотилки»
                    continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                rospy.logwarn('Камера %s перестала отдавать кадры, переподключаюсь', self.device)
                self._cleanup()
                rospy.sleep(0.5)
                continue

            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.pub.publish(msg)

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _cleanup(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


if __name__ == '__main__':
    try:
        USBCameraNode().run()
    except rospy.ROSInterruptException:
        pass

