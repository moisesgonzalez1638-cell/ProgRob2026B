import rclpy
from rclpy.node import Node
from hardware_status_interfaces.srv import CheckSystem

class OperatorNode(Node):
    def __init__(self):
        super().__init__('operator_node')
        self.cli = self.create_client(CheckSystem, 'check_system')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio check_system...')
        
        self.timer = self.create_timer(2.0, self.send_request)
        self.get_logger().info('Nodo Operador iniciado.')

    def send_request(self):
        req = CheckSystem.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Diagnostico recibido -> Seguro: {response.seguro} | Mensaje: "{response.mensaje}"')
        except Exception as e:
            self.get_logger().error(f'Fallo la llamada al servicio: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OperatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
