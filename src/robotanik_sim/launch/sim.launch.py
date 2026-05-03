import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robotanik_sim')
    urdf_path        = os.path.join(pkg_share, 'urdf',   'robotanik.urdf')
    world_path       = os.path.join(pkg_share, 'worlds', 'robotanik_sera.world')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    map_yaml_path    = os.path.join(pkg_share, 'maps',   'robotanik_sera.yaml')
    ekf_config_path  = os.path.join(pkg_share, 'config', 'ekf.yaml')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    # 3. Robot spawn (5sn bekle)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robotanik',
                '-file', urdf_path,
                '-x', '10.45',
                '-y', '1.0',
                '-z', '0.2275', # Şef Notu: URDF z yüksekliği ile eşitledim, havadan düşmesin
                '-Y', '1.5708'
            ],
            output='screen'
        )]
    )

    # 4. EKF — Sadece odom drift'ini filtreler (odom -> base_footprint)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # =====================================================================
    # ŞEFİN MÜDAHALESİ: static_tf_node BURADAN TAMAMEN SİLİNDİ!
    # Neden? Çünkü Nav2 (AMCL) zaten map->odom yayınlıyor. Çakışmayı bitirdik.
    # =====================================================================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # 5. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Nav2 (10sn bekle)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_node = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_path,
                'params_file': nav2_params_path,
                'use_sim_time': 'true',
                'use_composition': 'False'
            }.items(),
        )]
    )

    # 7. FSM (18sn bekle)
    row_fsm_node = TimerAction(
        period=20.0,
        actions=[Node(
            package='robotanik_sim',
            executable='row_fsm.py',
            name='robotanik_row_fsm',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity,
        ekf_node,          # EKF Aktif (Odom->Base_link)
        static_tf_node,
        rviz_node,
        nav2_node,         # Nav2/AMCL Aktif (Map->Odom)
        row_fsm_node,
    ])
