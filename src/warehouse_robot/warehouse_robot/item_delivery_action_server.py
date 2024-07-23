import rclpy
import time
from rclpy.node import Node
from warehouse_robot_interfaces.action import DeliverItem
from warehouse_robot_interfaces.srv import CheckStock
from rclpy.action import ActionServer
from warehouse_robot.stock_checker_service_client import StockCheckerClient

class DeliveryActionServer(Node):

    def __init__(self):
        super().__init__('delivery_action_server')
        self._action_server = ActionServer(
            self,
            DeliverItem,
            'deliver_item',
            self._handle_goal
        )
        self._stock_checker_client = StockCheckerClient()

    def _handle_goal(self, goal_handle):
        item_name = goal_handle.request.item_name
        quantity = goal_handle.request.quantity

        stock_level = self._stock_checker_client.check_stock(item_name)
        self.get_logger().info(f"Stock level for '{item_name}': {stock_level}")

        if stock_level <= 0:
            goal_handle.abort()
            result = DeliverItem.Result()
            result.success = False
            result.message = f"Item {item_name} not available."
            return result

        if stock_level < quantity:
            goal_handle.abort()
            result = DeliverItem.Result()
            result.success = False
            result.message = f"Insufficient stock for {item_name}. Requested: {quantity}, Available: {stock_level}"
            return result
        
        self.get_logger().info('Goal accepted')
        for i in range(1, quantity + 1):
            time.sleep(1)
            feedback = DeliverItem.Feedback()
            feedback.status = f'Delivered {i}/{quantity} {item_name}'
            goal_handle.publish_feedback(feedback)
        
        goal_handle.succeed()
        result = DeliverItem.Result()
        result.success = True
        result.message = 'Delivery completed successfully'
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
