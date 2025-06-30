#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from geopy.distance import distance

class ZoneAvoidanceNode:
    def __init__(self):
        rospy.init_node('zone_avoidance_node')

        # Yasaklı bölge merkezi ve yarıçapı (örnek)
        self.forbidden_zone_center = (-35.3587, 149.1650)  # senin güncel konuma yakın
        self.forbidden_zone_radius = 30  # metre

        self.current_lat = None
        self.current_lon = None

        # Velocity komutları yayıncısı
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', TwistStamped, queue_size=10)

        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        self.rate = rospy.Rate(10)

        rospy.loginfo("Zone Avoidance Node başladı.")

        self.main_loop()

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_lat is None or self.current_lon is None:
                self.rate.sleep()
                continue

            # Yasaklı bölgede mi kontrolü
            dist = distance((self.current_lat, self.current_lon), self.forbidden_zone_center).meters

            if dist < self.forbidden_zone_radius:
                rospy.logwarn("Yasaklı bölgeye girildi! Kaçış manevrası başlatılıyor.")
                self.escape_maneuver()
            else:
                self.stop_drone()

            self.rate.sleep()

    def escape_maneuver(self):
        twist = TwistStamped()

        # Örnek: İleri 2 m/s hız, sağa dönüş 0.5 rad/s
        twist.twist.linear.x = 2.0
        twist.twist.angular.z = 0.5

        self.vel_pub.publish(twist)

    def stop_drone(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0
        twist.twist.angular.z = 0
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        ZoneAvoidanceNode()
    except rospy.ROSInterruptException:
        pass

