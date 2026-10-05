import rclpy
from rclpy.node import Node
from hardware_status_interfaces.msg import HardwareStatus

class EmulatorNode(Node):
    def __init__(self):
        super().__init__('emulator_node')
        self.declare_parameter('temperatura', 45)
        self.declare_parameter('motores_listos', True)
        self.declare_parameter('debug_msg', 'Sistema operando normalmente')

        self.publisher_ = self.create_publisher(HardwareStatus, 'hardware_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Nodo Emulador iniciado.')

    def publish_status(self):
        msg = HardwareStatus()
        msg.temperatura = self.get_parameter('temperatura').get_parameter_value().integer_value
        msg.motores_listos = self.get_parameter('motores_listos').get_parameter_value().bool_value
        msg.debug_msg = self.get_parameter('debug_msg').get_parameter_value().string_value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EmulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
