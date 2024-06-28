#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
import numpy as np

class RobotController():
    def __init__(self):
        rospy.init_node('robot_controller_node')
        self.bumper_triggered = False
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bumper_sub = rospy.Subscriber('/bumper', Bool, self.bumper_callback, queue_size=10)
        self.distance_sub = rospy.Subscriber('/distance_sensors', PointCloud, self.pointcloud_callback, queue_size=10)
        self.twist = Twist()
        self.twist.linear.x = 1.0  # Inicia movimento para frente
        self.min_distance_sensor_1 = 0.4 # Ajuste conforme necessário
        self.min_distance_sensor_2 = 0.3
        self.min_distance_sensor_3 = 0.07
        self.min_distance_sensor_4 = 0.2
        self.min_distance_sensor_5 = 0.4
        self.min_distance_sensor_6 = 0.3
        self.min_distance_sensor_7 = 0.2
        self.min_distance_sensor_8 = 0.08
        self.min_distance_sensor_9 = 0.3
        self.control_robot()

    def bumper_callback(self, bumper_msg):
        # Manipulação da mensagem do bumper
        self.bumper_triggered = bumper_msg.data
        rospy.loginfo("Estado do Bumper: %s", self.bumper_triggered)

        if self.bumper_triggered:
            rospy.loginfo("Bumper Acionado - Parando o robô")
            self.twist.linear.x = 0.0  # Para o movimento linear
            self.cmd_pub.publish(self.twist)  # Publica o comando de parada
            rospy.signal_shutdown("Bumper Acionado - Comunicação Cortada")  # Encerra o nó

    def pointcloud_callback(self, pointcloud_msg):
        # Processa a nuvem de pontos para obter informações sobre os obstáculos
        if len(pointcloud_msg.points) > 0:
            # Obtém a distância do primeiro ponto na nuvem (sensor 1)
            distance_sensor_1 = np.sqrt(pointcloud_msg.points[0].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 1: {distance_sensor_1}")
            distance_sensor_2 = np.sqrt(pointcloud_msg.points[1].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 2: {distance_sensor_2}")
            distance_sensor_3 = np.sqrt(pointcloud_msg.points[2].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 3: {distance_sensor_3}")
            distance_sensor_4 = np.sqrt(pointcloud_msg.points[3].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 4: {distance_sensor_4}")
            distance_sensor_5 = np.sqrt(pointcloud_msg.points[4].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 5: {distance_sensor_5}")
            distance_sensor_6 = np.sqrt(pointcloud_msg.points[5].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 6: {distance_sensor_6}")
            distance_sensor_7 = np.sqrt(pointcloud_msg.points[6].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 7: {distance_sensor_7}")
            distance_sensor_8 = np.sqrt(pointcloud_msg.points[7].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 8: {distance_sensor_8}")
            distance_sensor_9 = np.sqrt(pointcloud_msg.points[8].x**2 + pointcloud_msg.points[0].y**2 + pointcloud_msg.points[0].z**2)
            rospy.loginfo(f"Distância do Sensor 9: {distance_sensor_9}")

            # Define uma velocidade linear padrão
            self.twist.linear.x = 0.4

            # Define velocidades angulares com base na distância do sensor 1
            if distance_sensor_1 < self.min_distance_sensor_1:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0
            if distance_sensor_2 < self.min_distance_sensor_2:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_3 < self.min_distance_sensor_3:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_4 < self.min_distance_sensor_4:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_5 < self.min_distance_sensor_5:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_6 < self.min_distance_sensor_6:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_7 < self.min_distance_sensor_7:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_8 < self.min_distance_sensor_8:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0  
            if distance_sensor_9 < self.min_distance_sensor_9:
                rospy.loginfo("Objeto próximo! Parando todos os movimentos.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para garantir a parada completa
                rospy.loginfo("Iniciando mudança de trajetória.")
                # Após parar, vire para evitar o obstáculo
                self.twist.angular.z = 0.7  # Ajuste conforme necessário
                self.cmd_pub.publish(self.twist)
                rospy.sleep(0.5)  # Adiciona um atraso de 1 segundo para permitir o giro
                rospy.loginfo("Retornando à trajetória normal.")
                # Volte a uma velocidade linear padrão e pare de girar
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0        
            else:
                # Caso contrário, mantenha a direção atual
                rospy.loginfo("Distância segura. Mantendo trajetória atual.")
                self.twist.angular.z = 0.0

            # Publica o comando de velocidade
            self.cmd_pub.publish(self.twist)

    def control_robot(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
    except rospy.ROSInterruptException:
        pass
