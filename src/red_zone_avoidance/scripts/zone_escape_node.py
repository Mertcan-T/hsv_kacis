#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandInt
from geopy.distance import distance

class ZoneEscape:
    def __init__(self):
        rospy.init_node("zone_escape_node")

        # Yasaklı bölge merkezi ve yarıçapı
        self.forbidden_zone = {
            "lat": -35.360000,
            "lon": 149.162000,
            "radius": 100  # metre
        }

        # Drone konumu
        self.current_lat = None
        self.current_lon = None

        # Sadece bir kez kaçış yap
        self.escape_triggered = False

        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)

        rospy.loginfo("🛰️ ZoneEscape Node çalışıyor...")
        rospy.spin()

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        current_pos = (self.current_lat, self.current_lon)
        forbidden_center = (self.forbidden_zone["lat"], self.forbidden_zone["lon"])

        mesafe = distance(current_pos, forbidden_center).meters
        rospy.loginfo(f"🌍 Drone Konumu: {self.current_lat}, {self.current_lon} | Yasaklı bölgeye mesafe: {mesafe:.2f} m")

        if mesafe < self.forbidden_zone["radius"] and not self.escape_triggered:
            rospy.logwarn("🚨 Yasaklı bölgeye girildi! Kaçış başlatılıyor...")
            self.escape_triggered = True
            self.execute_escape()

    def execute_escape(self):
        escape_lat = self.current_lat + 0.0005  # 50m kuzey
        escape_lon = self.current_lon + 0.0005  # 50m doğu

        try:
            rospy.wait_for_service('/mavros/cmd/command_int')
            service = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)
            response = service(
                broadcast=0,
                frame=6,         # GLOBAL
                command=192,     # MAV_CMD_NAV_WAYPOINT
                current=0,
                autocontinue=1,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                x=int(escape_lat * 10**7),
                y=int(escape_lon * 10**7),
                z=100
            )
            rospy.loginfo("🚀 Kaçış komutu gönderildi.")
        except rospy.ServiceException as e:
            rospy.logerr("❌ Servis çağrısı başarısız: %s" % e)

if __name__ == '__main__':
    ZoneEscape()

