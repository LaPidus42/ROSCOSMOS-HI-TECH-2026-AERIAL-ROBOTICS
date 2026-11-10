#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP Video Node — приём RTP/H.264 потока курсовой камеры (udpsrc port=5600)
и публикация готовых JPEG в sensor_msgs/CompressedImage.
Декодирование H.264 и кодирование JPEG делает GStreamer (вне Python).
"""
import threading
import rospy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from sensor_msgs.msg import CompressedImage


class UdpVideoNode:
    def __init__(self):
        rospy.init_node('udp_video_node')
        self.topic = rospy.get_param('~topic', '/course_camera/image_raw')
        self.fps = int(rospy.get_param('~max_fps', 15))
        self.quality = int(rospy.get_param('~jpeg_quality', 80))
        self.pipeline_str = rospy.get_param('~pipeline', self._default_pipeline())
        self.pub = rospy.Publisher(self.topic, CompressedImage, queue_size=1)
        self._pipe = None
        Gst.init(None)
        threading.Thread(target=self._run, daemon=True).start()
        rospy.on_shutdown(self._stop)
        rospy.loginfo('UDP video: port=5600 (H.264) -> %s @ %d fps', self.topic, self.fps)


    def _default_pipeline(self):
        return (
            'udpsrc port=5600 '
            'caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,'
            'encoding-name=(string)H264,payload=(int)96" ! '
            'rtpjitterbuffer latency=100 ! '
            'rtph264depay ! h264parse ! '
            'mppvideodec ! '
            'videoconvert ! videorate ! '
            'video/x-raw,framerate=%d/1 ! '
            'jpegenc quality=%d ! appsink name=sink emit-signals=true '
            'max-buffers=1 drop=true' % (self.fps, self.quality))


    def _run(self):
        while not rospy.is_shutdown():
            try:
                pipe = Gst.parse_launch(self.pipeline_str)
            except Exception as e:
                rospy.logerr_throttle(5, 'GStreamer: ошибка сборки пайплайна: %s', e)
                rospy.sleep(3.0)
                continue
            self._pipe = pipe
            pipe.get_by_name('sink').connect('new-sample', self._on_sample)
            pipe.set_state(Gst.State.PLAYING)
            bus = pipe.get_bus()
            err = None
            while not rospy.is_shutdown():
                msg = bus.timed_pop_filtered(500 * Gst.MSECOND,
                                             Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if msg is None:
                    continue
                if msg.type == Gst.MessageType.ERROR:
                    e, dbg = msg.parse_error()
                    err = '%s (%s)' % (e, dbg)
                break
            pipe.set_state(Gst.State.NULL)
            self._pipe = None
            if rospy.is_shutdown():
                break
            rospy.logwarn_throttle(5, 'Поток прерван (%s), переподключение...', err or 'EOS')
            rospy.sleep(2.0)

    def _on_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        ok, mp = buf.map(Gst.MapMode.READ)
        if ok:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            msg.data = bytes(mp.data)
            self.pub.publish(msg)
            buf.unmap(mp)
        return Gst.FlowReturn.OK

    def _stop(self):
        if self._pipe is not None:
            self._pipe.set_state(Gst.State.NULL)


if __name__ == '__main__':
    try:
        UdpVideoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
