import rclpy
from rclpy.node import Node
from warehouse_robot_interfaces.srv import CheckStock

class StockCheckerServer(Node):
    def __init__(self):
        super().__init__("stock_checker_server")
        self._service = self.create_service(CheckStock, 'check_stock', self._handle_stock_check)
        self._stock_levels = {'item1': 10, 'item2': 5}

    def _handle_stock_check(self, request, response):
        item_name = request.item_name
        if item_name in self._stock_levels:
            response.stock_level = self._stock_levels[item_name]
        else:
            response.stock_level = 0
            self.get_logger().warn(f'Item not found: {item_name}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server_node = StockCheckerServer()
    rclpy.spin(server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
