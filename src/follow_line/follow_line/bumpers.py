import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import BumperEvent  # Tipo de mensaje correcto

class BumperListener(Node):
    def __init__(self):
        super().__init__('bumper_listener')
        # Crear suscripción al tópico /events/bumper
        self.subscription = self.create_subscription(
            BumperEvent,  # Tipo correcto del mensaje
            '/events/bumper',
            self.bumper_callback,
            10
        )
        self.get_logger().info("Nodo iniciado. Escuchando /events/bumper...")

    def bumper_callback(self, msg):
        """Callback para imprimir los valores del bumper y su estado."""
        bumper_name = {0: "Izquierdo", 1: "Central", 2: "Derecho"}.get(msg.bumper, "Desconocido")
        state = "PRESIONADO" if msg.state == 1 else "LIBRE"

        self.get_logger().info(f"Bumper: {bumper_name} | Estado: {state}")

def main(args=None):
    rclpy.init(args=args)
    node = BumperListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
