#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.srv import CommandInt
from std_msgs.msg import Header

class WaypointSender:
    def __init__(self):
        rospy.init_node("goto_gps_node")

        # Hedef GPS konumu (örnek konum - güncelleyebilirsin)
        self.target_lat = -35.462000   # Derece cinsinden
        self.target_lon = 149.165500   # Derece cinsinden
        self.altitude = 100            # Hedef irtifa (metre)

        # MAVLink protokolü 1e7 dönüşümü
        self.target_lat_int = int(self.target_lat * 1e7)
        self.target_lon_int = int(self.target_lon * 1e7)

        rospy.wait_for_service("/mavros/cmd/command_int")
        try:
            goto_service = rospy.ServiceProxy("/mavros/cmd/command_int", CommandInt)
            rospy.loginfo("Servise bağlandı, konum gönderiliyor...")

            # CommandInt ile hedef waypoint gönder
            resp = goto_service(
                broadcast=0,
                frame=6,              # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=192,           # MAV_CMD_NAV_WAYPOINT
                current=0,
                autocontinue=1,
                param1=0,             # Hold time
                param2=0,             # Acceptance radius
                param3=0,             # Pass through waypoint
                param4=0,             # Yaw angle
                x=self.target_lat_int,
                y=self.target_lon_int,
                z=self.altitude
            )

            if resp.success:
                rospy.loginfo("✅ Hedef konum başarıyla gönderildi!")
            else:
                rospy.logwarn("❌ Hedef konum gönderilemedi!")
        except rospy.ServiceException as e:
            rospy.logerr(f"Servis çağrısı başarısız: {e}")

if __name__ == "__main__":
    try:
        WaypointSender()
    except rospy.ROSInterruptException:
        pass

