#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import os

def spawn_forbidden_zone_model(name, lat, lon, altitude):
    # Model dosyasının yolunu belirle
    model_path = os.path.join(
        os.path.expanduser('~'),
        'deneme1/models/forbidden_zone/model.sdf'
    )

    with open(model_path, 'r') as f:
        model_xml = f.read()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        pose = Pose()
        pose.position.x = lat   # Bu kısımlar basitleştirilmiş simülasyon içindir
        pose.position.y = lon
        pose.position.z = altitude
        pose.orientation = Quaternion(0, 0, 0, 1)

        resp = spawn_model(
            model_name=name,
            model_xml=model_xml,
            robot_namespace='/',
            initial_pose=pose,
            reference_frame='world'
        )
        rospy.loginfo(f"{name} spawn edildi!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn başarısız: {e}")

if __name__ == '__main__':
    rospy.init_node('forbidden_zone_spawner')

    # Örnek yasaklı bölgeleri spawn et
    spawn_forbidden_zone_model("zone_a", 5, 5, 0)
    spawn_forbidden_zone_model("zone_b", -3, 4, 0)
    spawn_forbidden_zone_model("zone_c", 2, -6, 0)

