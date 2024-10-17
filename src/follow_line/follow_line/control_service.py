import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
# from PyQt5.uic import loadUi
from example_interfaces.srv import SetBool
import rclpy

from PyQt5 import uic

ui_MainWindow, QtBaseClass = uic.loadUiType("/home/canveo/carla_ws/src/follow_line/follow_line/ui/main_window.ui")

class MainWindow(QMainWindow, ui_MainWindow):
    def __init__(self):
        # super().__init__()
        # loadUi("/home/canveo/carla_ws/src/follow_line/follow_line/ui/main_window_ui.py", self) 
        QMainWindow.__init__(self)
        ui_MainWindow.__init__(self)
        self.setupUi(self)

        # Conectar señales a ranuras
        self.control_robot_button.clicked.connect(self.control_robot)
        self.record_bag_button.clicked.connect(self.record_bag)

        # Crear nodo ROS
        rclpy.init()
        self.node = rclpy.create_node("qt_ros_interface")

    def control_robot(self):
        # Llamar al servicio de control de robot
        service_client = self.node.create_client(SetBool, "control_robot")
        request = SetBool.Request()
        request.data = True  # Establecer el valor deseado
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            print("Servicio de control de robot llamado exitosamente")
        else:
            print("Error al llamar al servicio de control de robot")

    def record_bag(self):
        # Llamar al servicio de grabación de bag
        service_client = self.node.create_client(SetBool, "control_bag_recording")
        request = SetBool.Request()
        request.data = True  # Establecer el valor deseado
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            print("Servicio de grabación de bag llamado exitosamente")
        else:
            print("Error al llamar al servicio de grabación de bag")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
