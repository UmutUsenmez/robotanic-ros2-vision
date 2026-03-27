import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robotanik_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotanik.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'robotanik_sera.world')

    # URDF dosyasının içeriğini okuyup değişkene atıyoruz (RSP için gerekli)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher Düğümü
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True  # Zaman senkronizasyonu
        }]
    )

    # 2. Gazebo'yu başlatma komutu 
    # DİKKAT: '-s', 'libgazebo_ros_init.so' eklentisi buraya eklendi! (Zamanı yayınlar)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    # 3. Robotu Gazebo dünyasına ışınlama
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robotanik', 
            '-file', urdf_path,
            '-x', '0.6',
            '-y', '1.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity
    ])
