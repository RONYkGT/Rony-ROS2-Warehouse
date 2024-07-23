import rclpy
from rclpy.node import Node
from warehouse_robot_interfaces.action import DeliverItem
from rclpy.action import ActionClient

class ItemDeliveryClient(Node):

    def __init__(self):
        super().__init__('item_delivery_client')
        self._action_client = ActionClient(self, DeliverItem, 'deliver_item')
        self.get_logger().info('Action client initialized')

    def send_goal(self, item_name, quantity):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not reachable')
            return

        goal_msg = DeliverItem.Goal()
        goal_msg.item_name = item_name
        goal_msg.quantity = quantity

        self.get_logger().info(f'Sending goal: {item_name}, quantity: {quantity}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was not accepted')
            return

        self._get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)
        result = self._get_result_future.result().result

        if result.success:
            self.get_logger().info(f'Action succeeded: {result.message}')
        else:
            self.get_logger().error(f'Action failed: {result.message}')
        
    def _feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback.status}')

def main(args=None):
    rclpy.init(args=args)
    client = ItemDeliveryClient()

    client.send_goal('item1', 3)
    
    rclpy.spin(client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
