## warehouse_robot: A ROS 2 Package for Warehouse Automation

This document provides an overview of the `warehouse_robot` package, designed for managing warehouse operations using ROS 2. 

### Overview
The package is structured into two primary components:

* **warehouse_robot:** Houses the core functionalities of the robot, including:
    * Action servers and clients for coordinating item deliveries.
    * Service servers and clients for managing stock information.
    * Launch files for configuring and starting the robot system.
    * Necessary scripts and resources for the robot's operation.
* **warehouse_robot_interfaces:** Defines the communication protocols between ROS 2 nodes:
    * Custom messages and services tailored to warehouse operations (e.g., item details, stock levels, delivery requests).

The package also includes the standard ROS 2 build and installation infrastructure. 

### Node Structure
The warehouse robot system comprises four primary nodes:
* Item Delivery Action Server: Manages item delivery requests, assigns tasks, and monitors delivery status.
* Item Delivery Action Client: Initiates item delivery requests based on user or system inputs.
* Stock Checker Service Server: Provides real-time stock information upon request.
* Stock Checker Service Client: Queries the stock level of specific items.

### Usage
To integrate the `warehouse_robot` package into your ROS 2 workspace:

1. Build the package: Use `colcon build` within your ROS 2 workspace to compile the code.
2. Source the setup script: Run `source install/setup.bash` to make the package accessible to ROS 2 tools.
3. Launch the system: Employ the provided launch files to start the necessary nodes.
4. Interact with the system: Utilize the action and service clients to interact with the robot for item delivery and stock management.
