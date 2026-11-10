# -*- coding: utf-8 -*-
"""Конвертация sensor_msgs/Image в JPEG без cv_bridge (numpy + OpenCV)."""

import cv2
import numpy as np

_BAYER = {
    'bayer_rggb8': cv2.COLOR_BayerBG2BGR,   # в OpenCV имена шаблонов сдвинуты
    'bayer_bggr8': cv2.COLOR_BayerRG2BGR,
    'bayer_gbrg8': cv2.COLOR_BayerGR2BGR,
    'bayer_grbg8': cv2.COLOR_BayerGB2BGR,
}


def _rows(msg, channels, dtype=np.uint8):
    """Срезает выравнивание строк (step) и возвращает массив h x w x channels."""
    itemsize = np.dtype(dtype).itemsize
    buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    buf = buf[:, :msg.width * channels * itemsize]
    dt = np.dtype(dtype)
    if itemsize > 1:                                   # байты в порядке, указанном в сообщении
        dt = dt.newbyteorder('>' if msg.is_bigendian else '<')
    arr = np.ascontiguousarray(buf).view(dt).astype(dtype, copy=False)
    return arr.reshape(msg.height, msg.width, channels) if channels > 1 else arr.reshape(msg.height, msg.width)


def _to_8bit(img):
    lo, hi = np.percentile(img, (1, 99))
    if hi <= lo:
        hi = lo + 1
    return np.clip((img.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)


def image_msg_to_bgr(msg):
    enc = (msg.encoding or '').lower()
    if enc in ('bgr8', '8uc3'):
        return _rows(msg, 3)
    if enc == 'rgb8':
        return cv2.cvtColor(_rows(msg, 3), cv2.COLOR_RGB2BGR)
    if enc in ('bgra8', '8uc4'):
        return cv2.cvtColor(_rows(msg, 4), cv2.COLOR_BGRA2BGR)
    if enc == 'rgba8':
        return cv2.cvtColor(_rows(msg, 4), cv2.COLOR_RGBA2BGR)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(_rows(msg, 1), cv2.COLOR_GRAY2BGR)
    if enc in ('mono16', '16uc1'):
        return cv2.cvtColor(_to_8bit(_rows(msg, 1, np.uint16)), cv2.COLOR_GRAY2BGR)
    if enc in ('yuv422', 'uyvy'):                       # в ROS yuv422 = UYVY
        return cv2.cvtColor(_rows(msg, 2), cv2.COLOR_YUV2BGR_UYVY)
    if enc in ('yuyv', 'yuv422_yuy2', 'yuy2'):
        return cv2.cvtColor(_rows(msg, 2), cv2.COLOR_YUV2BGR_YUYV)
    if enc in _BAYER:
        return cv2.cvtColor(_rows(msg, 1), _BAYER[enc])
    raise ValueError('неподдерживаемая кодировка изображения: %r' % msg.encoding)


def encode_jpeg(bgr, max_width=640, quality=75):
    h, w = bgr.shape[:2]
    if max_width and w > max_width:
        bgr = cv2.resize(bgr, (max_width, int(round(h * max_width / w))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buf.tobytes() if ok else None


def compressed_to_jpeg(data, max_width=640, quality=75):
    """CompressedImage: если уже JPEG нужного размера — отдаём как есть."""
    data = bytes(data)
    if data[:2] == b'\xff\xd8' and not max_width:
        return data
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return None
    if data[:2] == b'\xff\xd8' and img.shape[1] <= max_width:
        return data
    return encode_jpeg(img, max_width, quality)
