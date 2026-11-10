#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import subprocess
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge

class ThermalCameraNode:
    def __init__(self):
        rospy.init_node('thermal_camera_node')
        
        self.dev_id = rospy.get_param('~device_search_id', 'Camera (usb-xhci-hcd')
        self.width = rospy.get_param('~width', 256)
        self.height = rospy.get_param('~height', 192)
        self.fps = rospy.get_param('~target_fps', 5)
        
        self.out_w = rospy.get_param('~output_width', 640)
        self.out_h = rospy.get_param('~output_height', 480)
        self.jpeg_q = rospy.get_param('~jpeg_quality', 85)
        
        self.pub_img = rospy.Publisher('/thermal/image_raw', CompressedImage, queue_size=1)
        
        self.cap = None
        self._init_camera()
        
        self.rate = rospy.Rate(self.fps)
        rospy.on_shutdown(self._cleanup)
        rospy.loginfo(f"Thermal camera node started @ {self.fps} FPS")
    
    def _find_device(self):
        try:
            out = subprocess.run(['v4l2-ctl', '--list-devices'],
                                 capture_output=True, text=True).stdout
            in_block, devs = False, []
            for line in out.split('\n'):
                if line and not line.startswith(('\t', ' ')):
                    in_block = self.dev_id in line
                elif in_block and line.strip().startswith('/dev/video'):
                    devs.append(line.strip())
            return devs
        except Exception as e:
            rospy.logwarn(f"v4l2-ctl error: {e}")
            return []
    
    def _init_camera(self):
        for dev in self._find_device():
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                continue
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Прогрев
            good = 0
            for _ in range(70):
                ret, f = cap.read()
                if ret and f is not None and f.max() > 0:
                    good += 1
                    if good >= 10:
                        self.cap = cap
                        rospy.loginfo(f"Camera ready: {dev}")
                        return
            cap.release()
        rospy.logerr("Camera not found!")
    
    def _process_frame(self, frame):
        return cv2.resize(frame, (self.out_w, self.out_h),
                          interpolation=cv2.INTER_LINEAR)
    
    def run(self):
        while not rospy.is_shutdown():
            if self.cap is None or not self.cap.isOpened():
                rospy.logwarn("Reconnecting camera...")
                self._init_camera()
                if self.cap is None:
                    rospy.sleep(2.0)
                    continue
            
            ret, frame = self.cap.read()
            if not ret:
                self.cap.release()
                self.cap = None
                rospy.sleep(0.5)
                continue
            
            processed = self._process_frame(frame)
            
            # JPEG-сжатие
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            _, buf = cv2.imencode('.jpg', processed,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
            msg.data = buf.tobytes()
            self.pub_img.publish(msg)
            
            self.rate.sleep()
    
    def _cleanup(self):
        if self.cap:
            self.cap.release()

if __name__ == '__main__':
    node = ThermalCameraNode()
    node.run()
