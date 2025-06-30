#!/usr/bin/env python3
import rospy
import random
import math
from sensor_msgs.msg import NavSatFix
from red_zone_avoidance.msg import ForbiddenZone
from mavros_msgs.srv import CommandInt
from mavros_msgs.msg import AttitudeTarget
from geopy.distance import distance
import tf.transformations


class GotoTargetWithAvoidance:
    def __init__(self):
        rospy.init_node("goto_target_with_avoidance")

        # Konum bilgileri
        self.current_lat = None
        self.current_lon = None
        self.target_lat = None
        self.target_lon = None
        self.target_set = False

        # Durumlar
        self.state = "normal"  # normal, escaping, returning

        # Parametreler
        self.altitude = 50
        self.SAFETY_DISTANCE = 55  # metre
        self.forbidden_zones = []

        # ROS bağlantıları
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.position_callback)
        rospy.Subscriber("/forbidden_zone", ForbiddenZone, self.zone_callback)

        rospy.wait_for_service("/mavros/cmd/command_int")
        self.command_int_srv = rospy.ServiceProxy("/mavros/cmd/command_int", CommandInt)

        self.att_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

        rospy.loginfo("📡 Goto Target with Avoidance Node Başladı.")
        rospy.spin()

    def position_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        if None in (self.current_lat, self.current_lon):
            return

        current_pos = (self.current_lat, self.current_lon)

        if not self.target_set:
            self.set_random_target()
            return

        target_pos = (self.target_lat, self.target_lon)
        dist_to_target = distance(current_pos, target_pos).meters
        rospy.loginfo(f"📍 Mevcut hedefe mesafe: {dist_to_target:.1f} m")

        # Yasaklı alan kontrolü
        for zone in self.forbidden_zones:
            if not zone["active"]:
                continue

            zone_center = (zone["latitude"], zone["longitude"])
            dist_to_zone_edge = distance(current_pos, zone_center).meters - zone["radius"]

            rospy.loginfo(f"Yasaklı bölge {zone['name']} kenarına mesafe: {dist_to_zone_edge:.2f} m")

            if dist_to_zone_edge < self.SAFETY_DISTANCE and self.state == "normal":
                rospy.logwarn(f"⚠️ {zone['name']} bölgesine çok yaklaşıldı! Kaçış manevrası başlatılıyor.")
                self.state = "escaping"

                # Teğet yön belirle
                dx = zone["longitude"] - self.current_lon
                dy = zone["latitude"] - self.current_lat
                theta = math.atan2(dy, dx)
                tangent_angle = theta + math.pi / 2  # sağ teğet

                # Yönelme (yaw) ayarı
                self.publish_attitude_yaw(tangent_angle)

                # Kaçış hedefini konum olarak belirle
                ESCAPE_DISTANCE = 60
                delta_lat = (ESCAPE_DISTANCE / 111000.0) * math.cos(tangent_angle)
                delta_lon = (ESCAPE_DISTANCE / (111000.0 * math.cos(math.radians(self.current_lat)))) * math.sin(tangent_angle)

                self.escape_lat = self.current_lat + delta_lat
                self.escape_lon = self.current_lon + delta_lon

                self.send_waypoint(self.escape_lat, self.escape_lon, self.altitude)
                return

        # Kaçış sonrası hedefe dönüş
        if self.state == "escaping":
            dist_to_escape = distance(current_pos, (self.escape_lat, self.escape_lon)).meters
            if dist_to_escape < 10:
                rospy.loginfo("✅ Kaçış tamamlandı, hedefe dönülüyor...")
                self.send_waypoint(self.target_lat, self.target_lon, self.altitude)
                self.state = "returning"
            return

        # Hedefe ulaşıldıysa
        if dist_to_target < 10 and self.state in ["normal", "returning"]:
            rospy.loginfo("🎯 HEDEFE ULAŞILDI ✅")
            self.state = "done"

    def publish_attitude_yaw(self, yaw_rad):
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)

        att_msg = AttitudeTarget()
        att_msg.orientation.x = quat[0]
        att_msg.orientation.y = quat[1]
        att_msg.orientation.z = quat[2]
        att_msg.orientation.w = quat[3]
        att_msg.type_mask = 7  # Sadece orientation (roll, pitch, yaw)
        att_msg.thrust = 0.5  # Orta thrust

        self.att_pub.publish(att_msg)
        rospy.loginfo("🧭 Yönelme (yaw) gönderildi.")

    def set_random_target(self):
        delta_lat = random.uniform(0.001, 0.002)
        delta_lon = random.uniform(0.001, 0.002)
        self.target_lat = self.current_lat + delta_lat
        self.target_lon = self.current_lon + delta_lon
        self.target_set = True
        rospy.loginfo(f"🎯 Hedef Nokta Belirlendi: {self.target_lat:.6f}, {self.target_lon:.6f}")
        self.send_waypoint(self.target_lat, self.target_lon, self.altitude)

    def zone_callback(self, msg):
        zone = {
            "name": msg.name,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "radius": msg.radius,
            "active": msg.active
        }
        # Var olanı güncelle, yoksa ekle
        for i, z in enumerate(self.forbidden_zones):
            if z["name"] == zone["name"]:
                self.forbidden_zones[i] = zone
                return
        self.forbidden_zones.append(zone)

    def send_waypoint(self, lat, lon, alt):
        try:
            rospy.loginfo(f"📤 Waypoint gönderiliyor: {lat:.6f}, {lon:.6f}, {alt}")
            resp = self.command_int_srv(
                broadcast=0,
                frame=6,
                command=192,  # MAV_CMD_NAV_WAYPOINT
                current=0,
                autocontinue=1,
                param1=0, param2=0, param3=0, param4=0,
                x=int(lat * 1e7),
                y=int(lon * 1e7),
                z=alt
            )
            if resp.success:
                rospy.loginfo("✅ Waypoint başarıyla gönderildi.")
            else:
                rospy.logwarn("⚠️ Waypoint gönderilemedi!")
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ Servis hatası: {e}")


if __name__ == "__main__":
    try:
        GotoTargetWithAvoidance()
    except rospy.ROSInterruptException:
        pass

