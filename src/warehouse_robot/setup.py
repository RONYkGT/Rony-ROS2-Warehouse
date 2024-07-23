from setuptools import find_packages, setup

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/warehouse_robot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony.bassil3@net.usj.edu.lb',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stock_checker_server = warehouse_robot.stock_checker_service_server:main',
            'stock_checker_client = warehouse_robot.stock_checker_service_client:main',
            'item_delivery_server = warehouse_robot.item_delivery_action_server:main',
            'item_delivery_client = warehouse_robot.item_delivery_action_client:main'
        ],
    },
)
