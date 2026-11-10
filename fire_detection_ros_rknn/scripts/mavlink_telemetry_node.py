#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAVLink Telemetry Node с полным кэшированием данных (как в skat-04-borki.py).
Данные не сбрасываются в 0 между вызовами, что критично для стабильности.
"""

import rospy
import math
import time
from pymavlink import mavutil

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, BatteryState
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import UInt16, Float32, Int32, UInt16MultiArray


class MavlinkTelemetryNode:
    def __init__(self):
        rospy.init_node('mavlink_telemetry_node')
        
        self.port = rospy.get_param('~port', '/dev/ttyS3')
        self.baud = rospy.get_param('~baud', 57600)
        self.rc_channel = rospy.get_param('~rc_channel', 9)
        self.servo_id = rospy.get_param('~servo_id', 8)
        
        # === КЭШ ТЕЛЕМЕТРИИ (не сбрасывается в 0 между вызовами) ===
        self._tel_lat = 0.0
        self._tel_lon = 0.0
        self._tel_alt_agl = 0.0
        self._tel_alt_amsl = 0.0
        self._tel_yaw = 0.0
        self._tel_roll = 0.0
        self._tel_pitch = 0.0
        self._tel_rollspeed = 0.0
        self._tel_pitchspeed = 0.0
        self._tel_yawspeed = 0.0
        self._tel_vx = 0.0
        self._tel_vy = 0.0
        self._tel_vz = 0.0
        
        # Локальная позиция (LOCAL_POSITION_NED) — есть, когда автопилот получает
        # позицию от маркеров ArUco / VISION_POSITION_ESTIMATE
        self._tel_local = None          # (x_north, y_east, z_down)
        self._tel_local_new = False
        
        self._tel_gps_fix_type = 0
        self._tel_gps_sats = 0
        
        self._last_rc_value_cached = 1500
        self._last_wp_value_cached = 0
        self._last_servo_pwm_cached = 1500
        
        # Батарея: SYS_STATUS (общее напряжение, остаток) и BATTERY_STATUS (банки)
        self._batt_v = None          # В
        self._batt_remaining = None  # 0..1
        self._batt_cells = []        # В по банкам
        
        # Временные метки для троттлинга
        self._last_rc_time = 0
        self._last_wp_time = 0
        self.rc_interval = 0.25
        self.wp_interval = 2.0
        
        # Publishers
        self.pub_nav = rospy.Publisher('/mavlink/navsat', NavSatFix, queue_size=10)
        self.pub_imu = rospy.Publisher('/mavlink/attitude', Imu, queue_size=10)
        self.pub_vel = rospy.Publisher('/mavlink/velocity', TwistStamped, queue_size=10)
        self.pub_rc = rospy.Publisher('/mavlink/rc', UInt16MultiArray, queue_size=10)
        self.pub_wp = rospy.Publisher('/mavlink/waypoint', UInt16, queue_size=10)
        self.pub_servo = rospy.Publisher('/mavlink/servo', UInt16, queue_size=10)
        self.pub_battery = rospy.Publisher('/mavlink/battery', BatteryState, queue_size=5)
        # Для геопривязки детекций (geo_dashboard_node)
        self.pub_rel_alt = rospy.Publisher('/mavlink/rel_alt', Float32, queue_size=10)
        self.pub_gps_info = rospy.Publisher('/mavlink/gps_info', UInt16MultiArray, queue_size=10)
        self.pub_local = rospy.Publisher('/mavlink/local_position', PointStamped, queue_size=10)
        
        self.master = None
        self._connect()
        rospy.loginfo("MAVLink telemetry node started")
    
    def _connect(self):
        while not rospy.is_shutdown():
            try:
                self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
                self.master.wait_heartbeat(timeout=5)
                rospy.loginfo(f"Connected to Pixhawk: sysid={self.master.target_system}")
                return
            except Exception as e:
                rospy.logwarn(f"MAVLink connect failed: {e}. Retry in 5s...")
                rospy.sleep(5)
    
    def poll_messages(self):
        """Читает ВСЕ доступные сообщения из буфера и обновляет кэш."""
        if not self.master:
            return
        
        try:
            while True:
                msg = self.master.recv_match(blocking=False, timeout=0.01)
                if msg is None:
                    break
                
                msg_type = msg.get_type()
                
                if msg_type == 'GLOBAL_POSITION_INT':
                    self._tel_lat = msg.lat / 1e7
                    self._tel_lon = msg.lon / 1e7
                    self._tel_alt_agl = msg.relative_alt / 1e3
                    self._tel_alt_amsl = msg.alt / 1e3
                    self._tel_vx = msg.vx / 100.0
                    self._tel_vy = msg.vy / 100.0
                    self._tel_vz = msg.vz / 100.0
                
                elif msg_type == 'ATTITUDE':
                    yaw = math.degrees(msg.yaw)
                    if yaw < 0:
                        yaw += 360.0
                    self._tel_yaw = yaw
                    self._tel_roll = math.degrees(msg.roll)
                    self._tel_pitch = math.degrees(msg.pitch)
                    self._tel_rollspeed = msg.rollspeed
                    self._tel_pitchspeed = msg.pitchspeed
                    self._tel_yawspeed = msg.yawspeed
                
                elif msg_type == 'LOCAL_POSITION_NED':
                    self._tel_local = (msg.x, msg.y, msg.z)
                    self._tel_local_new = True
                
                elif msg_type == 'GPS_RAW_INT':
                    self._tel_gps_fix_type = msg.fix_type
                    self._tel_gps_sats = msg.satellites_visible
                    
                    if self._tel_gps_fix_type < 3:
                        rospy.logwarn_throttle(5, f"GPS: Fix={self._tel_gps_fix_type}, Sats={self._tel_gps_sats} (нет 3D-фиксации)")
                    else:
                        rospy.loginfo_throttle(10, f"GPS: Fix={self._tel_gps_fix_type}, Sats={self._tel_gps_sats}")
                
                elif msg_type == 'RC_CHANNELS':
                    try:
                        self._last_rc_value_cached = getattr(msg, f'chan{self.rc_channel}_raw', 1500)
                    except AttributeError:
                        pass
                
                elif msg_type == 'SYS_STATUS':
                    if msg.voltage_battery not in (0, 65535):
                        self._batt_v = msg.voltage_battery / 1000.0
                    self._batt_remaining = msg.battery_remaining / 100.0 if msg.battery_remaining >= 0 else None
                
                elif msg_type == 'BATTERY_STATUS' and getattr(msg, 'id', 0) == 0:
                    cells = [v / 1000.0 for v in msg.voltages if v not in (0, 65535)]
                    cells += [v / 1000.0 for v in (getattr(msg, 'voltages_ext', None) or []) if v not in (0, 65535)]
                    # одно значение на всю батарею (автопилот не меряет банки) — это не банка
                    self._batt_cells = cells if len(cells) > 1 else []
                    if msg.battery_remaining >= 0:
                        self._batt_remaining = msg.battery_remaining / 100.0
                
                elif msg_type == 'MISSION_CURRENT':
                    self._last_wp_value_cached = msg.seq
                
                elif msg_type == 'SERVO_OUTPUT_RAW':
                    try:
                        self._last_servo_pwm_cached = getattr(msg, f'servo{self.servo_id + 1}_raw', 1500)
                    except AttributeError:
                        pass
        
        except Exception as e:
            rospy.logwarn_throttle(5, f"poll_messages error: {e}")
    
    def _publish_all(self, now):
        # 1. NavSatFix
        nav = NavSatFix()
        nav.header.stamp = now
        nav.header.frame_id = "gps"
        nav.latitude = self._tel_lat
        nav.longitude = self._tel_lon
        nav.altitude = self._tel_alt_amsl
        
        if self._tel_gps_fix_type >= 3:
            nav.status.status = NavSatStatus.STATUS_FIX
        elif self._tel_gps_fix_type >= 1:
            nav.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            nav.status.status = -1
        
        nav.status.service = NavSatStatus.SERVICE_GPS
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_nav.publish(nav)
        
        # 2. IMU (Ориентация + Угловые скорости)
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        
        q = self._euler_to_quat(self._tel_roll, self._tel_pitch, self._tel_yaw)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        
        imu.angular_velocity.x = self._tel_rollspeed
        imu.angular_velocity.y = self._tel_pitchspeed
        imu.angular_velocity.z = self._tel_yawspeed
        
        self.pub_imu.publish(imu)
        
        # 3. Velocity (Линейные + Угловые скорости)
        vel = TwistStamped()
        vel.header.stamp = now
        vel.header.frame_id = "base_link"
        vel.twist.linear.x = self._tel_vx
        vel.twist.linear.y = self._tel_vy
        vel.twist.linear.z = self._tel_vz
        vel.twist.angular.x = self._tel_rollspeed
        vel.twist.angular.y = self._tel_pitchspeed
        vel.twist.angular.z = self._tel_yawspeed
        self.pub_vel.publish(vel)
        
        # 4. Высота над точкой взлёта (AGL) и состояние GPS
        self.pub_rel_alt.publish(Float32(self._tel_alt_agl))
        gps = UInt16MultiArray()
        gps.data = [int(self._tel_gps_fix_type), int(self._tel_gps_sats)]
        self.pub_gps_info.publish(gps)
        
        # 5. Локальная позиция — публикуем только свежую, чтобы потеря маркеров была видна
        if self._tel_local_new:
            self._tel_local_new = False
            pt = PointStamped()
            pt.header.stamp = now
            pt.header.frame_id = "local_ned"
            pt.point.x, pt.point.y, pt.point.z = self._tel_local
            self.pub_local.publish(pt)
    
    def _publish_aux(self, now):
        # RC
        rc_msg = UInt16MultiArray()
        rc_msg.data = [self._last_rc_value_cached]
        self.pub_rc.publish(rc_msg)
        
        # Waypoint
        self.pub_wp.publish(UInt16(self._last_wp_value_cached))
        
        # Servo
        self.pub_servo.publish(UInt16(self._last_servo_pwm_cached))
        
        # Battery
        if self._batt_v is not None:
            b = BatteryState()
            b.header.stamp = now
            b.voltage = self._batt_v
            b.percentage = self._batt_remaining if self._batt_remaining is not None else float('nan')
            b.cell_voltage = self._batt_cells
            b.present = True
            self.pub_battery.publish(b)
    
    @staticmethod
    def _euler_to_quat(roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w
    
    def run(self):
        rate = rospy.Rate(50)
        last_pub_time = 0.0
        last_aux_time = 0.0
        PUBLISH_INTERVAL = 0.1  # 10 Hz для основных данных
        AUX_INTERVAL = 0.5      # 2 Hz для вспомогательных
        
        rospy.loginfo("Main loop started")
        
        while not rospy.is_shutdown():
            try:
                self.poll_messages()
                
                now = rospy.Time.now()
                now_sec = now.to_sec()
                
                if now_sec - last_pub_time >= PUBLISH_INTERVAL:
                    self._publish_all(now)
                    last_pub_time = now_sec
                
                if now_sec - last_aux_time >= AUX_INTERVAL:
                    self._publish_aux(now)
                    last_aux_time = now_sec
                
                rate.sleep()
            
            except Exception as e:
                rospy.logwarn_throttle(5, f"Main loop error: {e}")
                rate.sleep()


if __name__ == '__main__':
    try:
        node = MavlinkTelemetryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
