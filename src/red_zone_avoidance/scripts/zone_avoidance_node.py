#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from red_zone_avoidance.msg import ForbiddenZone
from geopy.distance import distance

class ZoneAvoidanceNode:
    def __init__(self):
        rospy.init_node('zone_avoidance_node')

        self.current_lat = None
        self.current_lon = None
        self.zone = None  # ForbiddenZone mesajı

        # Velocity publisher
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', TwistStamped, queue_size=10)

        # Subscriberlara bağlan
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        rospy.Subscriber('/forbidden_zone', ForbiddenZone, self.zone_callback)

        self.rate = rospy.Rate(2)  # 2 Hz

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def zone_callback(self, msg):
        self.zone = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.current_lat is None or self.current_lon is None or self.zone is None:
                self.rate.sleep()
                continue

            current_pos = (self.current_lat, self.current_lon)
            zone_center = (self.zone.latitude, self.zone.longitude)

            dist_m = distance(current_pos, zone_center).meters
            rospy.loginfo("🔍 Mesafe yasaklı bölgeye: %.2f metre", dist_m)

            if self.zone.active and dist_m < self.zone.radius:
                rospy.logwarn("⚠️ Yasaklı bölgeye girildi! Kaçış manevrası başlatılıyor.")
                self.avoid_zone(zone_center)
            else:
                self.stop_drone()

            self.rate.sleep()

    def avoid_zone(self, center):
        # Basit mantık: bölgeden uzaklaşmak için ileri ve sola dön
        twist = TwistStamped()
        twist.twist.linear.x = 1.5
        twist.twist.angular.z = 0.5
        self.vel_pub.publish(twist)

    def stop_drone(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = ZoneAvoidanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

