import os
import cv2
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import pygame

from cv_bridge import CvBridge, CvBridgeError
from example_interfaces.srv import SetBool


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        self.subscription_img = self.create_subscription(
            Image,
            'usb_cam/image_raw',
            self.image_callback,
            10
        )

        self.subscription_control = self.create_subscription(
            Twist,
            'cmd_vel',
            self.control_callback,
            10
        )

        # self.cam = cv2.VideoCapture(2)

        self.command = Twist()
        self.command.linear.x = 0.0 
        self.command.linear.y = 0.0 
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

        self.linear_speed = []
        self.angular_speed = []
        self.image_timestamps = []
        self.image_names = []
        self.command_timestamps = []

        self.bridge = CvBridge()

        self.path_output = '/home/canveo/carla_ws/turtlebot_data'

        # Ruta donde quieres guardar las imágenes
        self.image_path = os.path.join(self.path_output, 'images')

        # Ruta para guardar los datos en archivos .npy
        self.output_image_file = os.path.join(self.path_output, 'images.npy')
        self.output_control_file = os.path.join(self.path_output, 'annotations.npy')

        # Ruta para guardar los datos en CSV
        self.output_data_file = os.path.join(self.path_output, 'data.csv')

        pygame.init()

        self.count = 0

        self.recording_enabled = True   # inicia grabacion por defecto

        self.bag_control_service = self.create_service(SetBool, 'control_bag_recording', self.control_bag_recording)

    def image_callback(self, msg):
        # ret_val, img = self.cam.read()
        if not self.recording_enabled:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Generar un nombre de archivo único basado en el marca de tiempo
        timestamp = self.get_timestamp()
        image_name = f"frame{self.count:06}.png"
        image_path = os.path.join(self.image_path, image_name)
        cv2.imwrite(image_path, cv_image)

        # Almacenar los datos asociados a la imagen
        self.image_timestamps.append(timestamp)
        self.image_names.append(image_name)

        self.count += 1

    def control_callback(self, msg):
        # Asociar los valores de dirección, aceleración y frenado con la última imagen registrada
        if not self.recording_enabled:
            return
        
        timestamp = self.get_timestamp()
        self.command_timestamps.append(timestamp)
        self.linear_speed.append(msg.linear.x)
        self.angular_speed.append(msg.angular.z)

    def get_timestamp(self):
        # Obtener el tiempo actual del sistema
        now = rclpy.clock.Clock().now()
        # Convertir el tiempo a nanosegundos y retornar
        return now.nanoseconds
       
    def save_data(self):
        # Guardar los datos como archivos .npy
        np.save(self.output_image_file, {'timestamps': self.image_timestamps, 'names': self.image_names})
        np.save(self.output_control_file, {'timestamps': self.command_timestamps, 'steer': self.angular_speed,
                                           'throttle': self.linear_speed})

        # Cargar los datos de los archivos .npy
        image_data = np.load(self.output_image_file, allow_pickle=True).item()
        control_data = np.load(self.output_control_file, allow_pickle=True).item()

        # Crear DataFrames con los datos
        image_df = pd.DataFrame({'timestamp': image_data['timestamps'], 'image_name': image_data['names']})
        control_df = pd.DataFrame({'timestamp': control_data['timestamps'], 'steer': control_data['steer'],
                                   'throttle': control_data['throttle']})
        
        # image_df['timestamp'] = image_df['timestamp'].astype('float64')


        # Fusionar los DataFrames basándose en los timestamps
        merged_df = pd.merge_asof(image_df, control_df, on='timestamp', direction='forward')

        # Guardar el DataFrame fusionado como archivo CSV
        merged_df.to_csv(self.output_data_file, index=False)

        print("Datos guardados en archivos .npy y CSV")

    def control_bag_recording(self, request, response):
        self.recording_enabled = request.data
        response = SetBool.Response()
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
