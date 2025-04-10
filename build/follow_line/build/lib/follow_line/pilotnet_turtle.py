from json import load
from os import error
import cv2
import numpy as np
import time
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

from tensorflow.keras.models import load_model
import random



# RGB_LOW = (0, 0, 100)
# RGB_HIGH = (100, 100, 255)

RGB_LOW = (0, 0, 0)
RGB_HIGH = (50, 50, 50)


FPS = 15.0
DISPLAY_FPS = 5.0
DISPLAY_IMG = True
IMG_W = 640
IMG_H = 480
# K = self.ANGULAR_VEL * 2

class WebcamControl(Node):
    def __init__(self):
        super().__init__('webcam_control')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, 'usb_cam/image_raw', 1)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.cam = cv2.VideoCapture(2) 

        self.command = Twist()
        self.command.linear.y = 0.0 
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0

        self.fps_t = 0
        self.display_fps_t = 0

        # Definiccón de constantes
        self.LINEAR_VEL = 0.15
        self.ANGULAR_VEL = 0.15


        # Ganancias PID
        self.kp = 1.2
        self.ki = 0.2
        self.kd = 0.09

        self.prev_error = 0.0
        self.integral = 0.0

        self.timer = self.create_timer(1.0 / FPS, self.follow_line)

        self.get_line_pos()

        # model train
        path_model = "/home/canveo/carla_ws/turtlebot_data/models/2024-04-06 08:00:43.124310_pilotnet_2y_albm_epoch_101.h5"
        self.model = load_model(path_model)

    def get_error(self):
        cx, img = self.get_line_pos()
        error = IMG_W / 2 - cx
        error_norm = error / (IMG_W / 2)
        return error_norm
    
    # def pid_control(self, error):
    #     P = self.kp * error

    #     self.integral += error
    #     I = self.ki * self.integral

    #     derivative = error - self.prev_error
    #     D = self.kd * derivative

    #     control = P + D + I

    #     self.prev_error = error

    #     return control        


    def follow_line(self):
        cx, img = self.get_line_pos()
        if img is None:
            return
        
        img_resize = cv2.resize(img[200:-1, :], (200, 66))
        prediction = self.model.predict(np.expand_dims(img_resize, axis=0))

        # angular_noise = random.uniform(-0.1, 0.1) 

        error = self.get_error()

        predicted_steer = prediction[0][0]
        predicted_throttle = prediction[0][1]

        # self.command.linear.x = float(predicted_throttle)
        # self.command.angular.z = float(predicted_steer)  

        if cx != -1:  # Drive straight behavior
       

            self.command.linear.x = float(predicted_throttle)
            self.command.angular.z = float(predicted_steer)  
        else:  # Turn around behavior
            self.command.linear.x = 0.0
            self.command.angular.z = self.ANGULAR_VEL

        self.pub.publish(self.command)

        if DISPLAY_IMG and time.time() - self.display_fps_t >= 1 / DISPLAY_FPS:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            except CvBridgeError as e:
                self.get_logger().error(e)
                return

            self.img_pub.publish(img_msg)
            self.display_fps_t = time.time()

        cv2.imshow("video feed", img)
        cv2.waitKey(1)

    def get_line_pos(self):
        ret_val, img = self.cam.read()

        if time.time() - self.fps_t < 1 / FPS:
            return None, None

        H, W, _ = img.shape
        img = img[int((H - IMG_H) / 2):int((H + IMG_H) / 2), int((W - IMG_W) / 2):int((W + IMG_W) / 2)]  # crop

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(img, RGB_LOW, RGB_HIGH)
        line_img = gray & mask

        contours, _ = cv2.findContours(np.uint8(line_img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            if cv2.contourArea(line) > 3000:
                moments = cv2.moments(line)
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])

                if DISPLAY_IMG:
                    cv2.drawContours(img, contours, -1, (180, 255, 180), 3)
                    cv2.drawContours(img, [line], -1, (0, 255, 0), 3)
                    cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)
                    cv2.rectangle(img, (180,320),(460,380),(0,0,255),3)
                    cv2.line(img,(320,320),(320,380),(0,0,255),5)
                    cv2.line(img,(cx,320),(cx,380),(255,0,0),5)
                    cv2.putText(img, f'Steering Control: {self.command.angular.z}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(img, f'Throttle Control: {self.command.linear.x}', (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
          
            else:
                cx = -1
        else:
            cx = -1

        return cx, img

def main(args=None):
    rclpy.init(args=args)
    node = WebcamControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cam.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
