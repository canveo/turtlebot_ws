from os import error
from urllib import response
import cv2
from flask import request
import numpy as np
import time
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class WebcamControl(Node):
    def __init__(self):
        super().__init__('webcam_control')

        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, 'usb_cam/image_raw', 1)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.cam = cv2.VideoCapture(2) 

        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0 
        self.command.linear.z = 0.0
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0

        self.fps_t = 0
        self.display_fps_t = 0

        # Definiccón de constantes
        self.LINEAR_VEL = 0.15
        self.ANGULAR_VEL = 0.5

        # RGB_LOW = (0, 0, 100)
        # RGB_HIGH = (100, 100, 255)

        self.RGB_LOW = (0, 0, 0)
        self.RGB_HIGH = (50, 50, 50)


        self.FPS = 15.0
        self.DISPLAY_FPS = 5.0
        self.DISPLAY_IMG = True
        self.IMG_W = 640  # 200
        self.IMG_H = 480  # 66
        self.K = self.ANGULAR_VEL * 2

        # Ganancias PID
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.5

        self.prev_error = 0.0
        self.integral = 0.0

        # Antiwindup
        self.MAX_OUTPUT = 0.7
        self.MIN_OUTPUT = -0.7

        self.timer = self.create_timer(1.0 / self.FPS, self.follow_line)

        self.get_line_pos()

        self.paused = False
        self.start_robot_service = self.create_service(SetBool, 'control_robot', self.control_robot)

    def get_error(self):
        cx, _ = self.get_line_pos()
        error = self.IMG_W / 2 - cx
        error_norm = error / (self.IMG_W / 2)
        return error_norm
    
    def pid_control(self, error):
        P = self.kp * error

        self.integral += error
        I = self.ki * self.integral

        derivative = error - self.prev_error
        D = self.kd * derivative

        control = P + D + I

        # Control de windup
        if control > self.MAX_OUTPUT:
            control = self.MAX_OUTPUT
            self.integral -= error  # Eliminar la parte integral acumulativa
        elif control < self.MIN_OUTPUT:
            control = self.MIN_OUTPUT
            self.integral -= error  # Eliminar la parte integral acumulativa


        self.prev_error = error

        return control        


    def follow_line(self):
        if not self.paused:
            cx, img = self.get_line_pos()
            if img is None:
                return

            if cx != -1:  
                error = self.get_error()

                control_signal = self.pid_control(error)

                self.command.linear.x = self.LINEAR_VEL * (1 - np.abs(error))
                self.command.angular.z = control_signal          
                
            else:  # Turn around behavior
                self.command.linear.x = 0.0
                self.command.angular.z = self.ANGULAR_VEL

            self.pub.publish(self.command)

            if self.DISPLAY_IMG and time.time() - self.display_fps_t >= 1 / self.DISPLAY_FPS:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                except CvBridgeError as e:
                    self.get_logger().error(e)
                    return

                self.display_fps_t = time.time()

            cv2.imshow("video feed", img)
            cv2.waitKey(1)

    def get_line_pos(self):
        ret_val, img = self.cam.read()

        if time.time() - self.fps_t < 1 / self.FPS:
            return None, None

        H, W, _ = img.shape
        img = img[int((H - self.IMG_H) / 2):int((H + self.IMG_H) / 2), int((W - self.IMG_W) / 2):int((W + self.IMG_W) / 2)]  # crop
        # img = img[200:266, 220:420]

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(img, self.RGB_LOW, self.RGB_HIGH)
        line_img = gray & mask

        # _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(np.uint8(line_img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
     
        if len(contours) > 0:
            line = max(contours, key=cv2.contourArea)
            if cv2.contourArea(line) > 3000:
                moments = cv2.moments(line)
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])

                if self.DISPLAY_IMG:
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
    
    def control_robot(self, request, response):
        self.paused = not request.data
        response = SetBool.Response()
        response.success = True
        return response
    
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
