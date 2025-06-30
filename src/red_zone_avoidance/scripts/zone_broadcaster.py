#!/usr/bin/env python3
import rospy
from red_zone_avoidance.msg import ForbiddenZone

def publisher():
    rospy.init_node("zone_broadcaster_node")
    pub = rospy.Publisher("forbidden_zone", ForbiddenZone, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    msg = ForbiddenZone()
    msg.latitude = -35.358000
    msg.longitude = 149.164000
    msg.radius = 40.0

    rospy.loginfo("🚫 Yasaklı bölge yayını başladı...")

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

