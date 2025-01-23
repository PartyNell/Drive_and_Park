from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joystick",
        executable="joystick_ros2.py",
        emulate_tty=True
    )

    joystick_to_cmd_node = Node(
        package="joystick",
        executable="joystick_to_cmd",
        emulate_tty=True
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node",
        emulate_tty=True
    )

    config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
        emulate_tty=True
    )

    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True
    )

    obstacle_detection_node = Node(
        package="obstacle_detection",
        executable="obstacle_detection",
        emulate_tty=True
    )

    car_command_node = Node(
        package="car_control",
        executable="car_command_node",
        emulate_tty=True
    )

    init_autonomous_node = Node(
        package="drive_autonomously",
        executable="init_autonomous_node",
        emulate_tty=True
    )

    autonomous_driving_node = Node(
        package="drive_autonomously",
        executable="autonomous_driving",
        emulate_tty=True
    )

    search_parking_node = Node(
        package="parkingspace_detection",
        executable="parking_space",
        emulate_tty=True
    )

    parking_node = Node(
        package="drive_autonomously",
        executable="auto_parking",
        emulate_tty=True
    )

    leaving_node = Node(
        package="drive_autonomously",
        executable="auto_leaving",
        emulate_tty=True
    ) 

    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(system_check_node)
    ld.add_action(obstacle_detection_node)
    ld.add_action(car_command_node)
    ld.add_action(init_autonomous_node)
    ld.add_action(autonomous_driving_node)
    ld.add_action(search_parking_node)
    ld.add_action(parking_node)
    ld.add_action(leaving_node)

    return ld