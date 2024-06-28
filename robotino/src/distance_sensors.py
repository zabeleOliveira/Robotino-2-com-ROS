#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance():
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        rospy.Subscriber('/distance_sensors', PointCloud, self.pointcloud_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        # Distâncias mínimas desejadas para cada sensor
        self.min_distances = [0.4, 0.3, 0.07, 0.2, 0.3, 0.3, 0.2, 0.08, 0.3]
        self.obstacle_detected = False

    def pointcloud_callback(self, pointcloud_msg):
        if len(pointcloud_msg.points) > 0:
            distances = [np.sqrt(p.x**2 + p.y**2 + p.z**2) for p in pointcloud_msg.points]
            for i, distance in enumerate(distances):
                rospy.loginfo(f"Distância do Sensor {i + 1}: {distance}")

            self.twist.linear.x = 0.1  # Define uma velocidade linear padrão

            # Verifica se há algum obstáculo próximo
            self.obstacle_detected = False
            for i, (distance, min_distance) in enumerate(zip(distances, self.min_distances)):
                if distance < min_distance:
                    rospy.loginfo(f"Objeto próximo! Sensor {i + 1} detectou uma distância de {distance} (mínimo permitido: {min_distance}). Parando todos os movimentos.")
                    self.obstacle_detected = True
                    break  # Sai do loop após detectar um obstáculo

            if self.obstacle_detected:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.6
            else:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0

            self.cmd_pub.publish(self.twist)  # Publica o comando de velocidade

if __name__ == '__main__':
    obstacle_avoidance = ObstacleAvoidance()
    rospy.spin()
