import rclpy
from rclpy.node import Node
from warehouse_robot_interfaces.srv import CheckStock
import matplotlib.pyplot as plt

class StockCheckerClient(Node):

    def __init__(self):
        super().__init__('stock_checker_client_node')
        self._client = self.create_client(CheckStock, 'check_stock')

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for StockChecker service to be available...')

    def _request_stock(self, item_name):
        request = CheckStock.Request()
        request.item_name = item_name
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().stock_level

    def get_stock_levels(self, items):
        levels = {}
        for item in items:
            levels[item] = self._request_stock(item)
        return levels

    def display_stock_levels(self, levels):
        items = list(levels.keys())
        stock_values = list(levels.values())

        plt.figure(figsize=(10, 6))
        plt.bar(items, stock_values, color='skyblue')
        plt.xlabel('Items')
        plt.ylabel('Stock Level')
        plt.title('Item Stock Levels')
        plt.ylim(0, max(stock_values) + 5)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    client_node = StockCheckerClient()

    item_names = ['item1', 'item2']
    stock_levels = client_node.get_stock_levels(item_names)

    client_node.display_stock_levels(stock_levels)

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
