import rclpy
from rclpy.node import Node
from hardware_status_interfaces.msg import HardwareStatus
from hardware_status_interfaces.srv import CheckSystem

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.declare_parameter('temp_limite', 70)
        
        self.subscription = self.create_subscription(
            HardwareStatus,
            'hardware_status',
            self.status_callback,
            10
        )
        self.srv = self.create_service(CheckSystem, 'check_system', self.check_system_callback)
        self.current_status = None
        self.get_logger().info('Nodo Monitor iniciado.')

    def status_callback(self, msg):
        self.current_status = msg

    def check_system_callback(self, request, response):
        temp_limite = self.get_parameter('temp_limite').get_parameter_value().integer_value
        if self.current_status is None:
            response.seguro = False
            response.mensaje = 'Sin datos de hardware aun'
        else:
            es_seguro = (self.current_status.temperatura <= temp_limite) and self.current_status.motores_listos
            response.seguro = es_seguro
            if es_seguro:
                response.mensaje = f'Sistema seguro. Temp: {self.current_status.temperatura} C'
            else:
                response.mensaje = f'ALERTA: Sistema inseguro. Temp: {self.current_status.temperatura} C, Motores: {self.current_status.motores_listos}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
