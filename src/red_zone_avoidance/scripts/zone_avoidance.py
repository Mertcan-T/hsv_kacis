#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from red_zone_avoidance.msg import ForbiddenZone
from mavros_msgs.srv import CommandInt
from geopy.distance import distance

ESCAPE_POINT = (-35.3600, 149.1600)  # Kaçış noktası
ESCAPE_ALTITUDE = 50  # Kaçış irtifası
SAFETY_DISTANCE = 55  # Yasaklı alana bu mesafede kaçış başlasın

class ZoneAvoidance:
    def __init__(self):
        rospy.init_node("zone_avoidance_node")

        self.latitude = None
        self.longitude = None
        self.forbidden_zones = []

        self.in_forbidden_zone = False
        self.inside_zone_timer = {}     # Toplam kalınan süreler
        self.zone_entry_time = {}       # Bölgeye giriş zamanları

        # Abonelikler
        self.global_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.global_position_callback)
        self.zone_sub = rospy.Subscriber("/forbidden_zone", ForbiddenZone, self.zone_callback)

        rospy.wait_for_service('/mavros/cmd/command_int')
        self.command_int_srv = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt)

    def zone_callback(self, msg):
        zone = {
            "name": msg.name,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "radius": msg.radius,
            "active": msg.active
        }
        # Aynı isimde varsa güncelle
        for i, z in enumerate(self.forbidden_zones):
            if z["name"] == zone["name"]:
                self.forbidden_zones[i] = zone
                return
        self.forbidden_zones.append(zone)

    def global_position_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        if self.latitude is None or self.longitude is None:
            return

        current_pos = (self.latitude, self.longitude)

        for zone in self.forbidden_zones:
            if not zone["active"]:
                continue

            zone_center = (zone["latitude"], zone["longitude"])
            dist = distance(current_pos, zone_center).meters
            dist_to_zone_edge = dist - zone["radius"]
            zone_name = zone["name"]

            if dist_to_zone_edge < 0:  # Yasaklı alanın içinde
                if zone_name not in self.zone_entry_time:
                    rospy.logwarn(f"{zone_name} bölgesine GİRİLDİ.")
                    self.zone_entry_time[zone_name] = rospy.Time.now()

                duration = (rospy.Time.now() - self.zone_entry_time[zone_name]).to_sec()
                total = self.inside_zone_timer.get(zone_name, 0) + duration

                if total >= 30:
                    rospy.logerr(f"{zone_name} içinde 30 saniyeden fazla kalındı!")
                else:
                    rospy.loginfo(f"{zone_name} içinde kalınan süre: {total:.1f} sn")

                if not self.in_forbidden_zone and dist_to_zone_edge > -SAFETY_DISTANCE:
                    rospy.logwarn(f"{zone_name} alanına çok yakınlaşıldı, kaçış başlatılıyor.")
                    self.in_forbidden_zone = True
                    self.escape()
                return  # Bu zonedeyiz, diğerlerine bakma

            else:
                # Eğer çıkıldıysa süreyi kaydet
                if zone_name in self.zone_entry_time:
                    duration = (rospy.Time.now() - self.zone_entry_time[zone_name]).to_sec()
                    self.inside_zone_timer[zone_name] = self.inside_zone_timer.get(zone_name, 0) + duration
                    rospy.loginfo(f"{zone_name} bölgesinden ÇIKILDI. Toplam süre: {self.inside_zone_timer[zone_name]:.1f} sn")
                    del self.zone_entry_time[zone_name]
                    self.in_forbidden_zone = False

    def escape(self):
        try:
            resp = self.command_int_srv(
                broadcast=0,
                frame=6,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=16,  # MAV_CMD_NAV_WAYPOINT
                param1=0, param2=0, param3=0, param4=0,
                x=ESCAPE_POINT[0],
                y=ESCAPE_POINT[1],
                z=ESCAPE_ALTITUDE
            )
            if resp.success:
                rospy.loginfo("Kaçış noktası komutu gönderildi.")
            else:
                rospy.logwarn("Kaçış komutu başarısız.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Kaçış komutu gönderilemedi: {e}")

if __name__ == "__main__":
    try:
        zone_avoidance = ZoneAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

