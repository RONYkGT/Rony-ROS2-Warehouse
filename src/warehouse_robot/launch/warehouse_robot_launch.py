from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_robot',
            executable='stock_checker_server',
            name='stock_checker_server'
        ),
        Node(
            package='warehouse_robot',
            executable='item_delivery_server',
            name='item_delivery_server'
        ),
        Node(
            package='warehouse_robot',
            executable='item_delivery_client',
            name='item_delivery_client'
        ),
        Node(
            package='warehouse_robot',
            executable='stock_checker_client',
            name='stock_checker_client'
        ),
    ])
