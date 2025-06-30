#!/usr/bin/env python3
import rospy
from red_zone_avoidance.msg import ForbiddenZone
from sensor_msgs.msg import NavSatFix

class StaticForbiddenZonePublisher:
    def __init__(self):
        rospy.init_node("static_forbidden_zone_publisher")
        self.pub = rospy.Publisher("/forbidden_zone", ForbiddenZone, queue_size=10)

        self.base_lat = None
        self.base_lon = None

        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.position_callback)
        rospy.loginfo("📡 Forbidden zone publisher hazır, konum bekleniyor...")
        rospy.spin()

    def position_callback(self, msg):
        if self.base_lat is None and self.base_lon is None:
            self.base_lat = msg.latitude
            self.base_lon = msg.longitude
            rospy.loginfo(f"📍 Mevcut konum: {self.base_lat}, {self.base_lon}")

            # 200 metre kuzeye yasaklı bölge ekle (yaklaşık 0.0018 derece)
            zone = ForbiddenZone()
            zone.name = "TestZone"
            zone.latitude = self.base_lat + 0.0018
            zone.longitude = self.base_lon
            zone.radius = 50.0  # metre
            zone.active = True

            # Yayın döngüsü
            rate = rospy.Rate(0.2)  # 5 saniyede bir
            while not rospy.is_shutdown():
                self.pub.publish(zone)
                rospy.loginfo(f"🛑 TestZone Yayınlandı -> Lat: {zone.latitude:.6f} Lon: {zone.longitude:.6f} R: {zone.radius:.1f}m")
                rate.sleep()

if __name__ == "__main__":
    try:
        StaticForbiddenZonePublisher()
    except rospy.ROSInterruptException:
        pass

