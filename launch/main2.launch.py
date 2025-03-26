import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rover_desc = get_package_share_directory('rover_desc_description')
#    world_path = os.path.join(pkg_mine_rover, 'worlds', 'myworld.world')
  #  world_path = os.path.join(pkg_mine_rover, 'worlds', 'world_1.world')
    urdf_dir = os.path.join(pkg_rover_desc, 'urdf')

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_gui = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Launch joint state publisher GUI'
    )

    show_gui = LaunchConfiguration('gui')
    
    # Process URDF and Xacro
    xacro_file = os.path.join(urdf_dir, 'rover_desc.xacro.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # RViz configuration file
    rviz_config_file = os.path.join(pkg_rover_desc, 'config', 'display.rviz')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : '-r -v 4 empty.sdf'
        }.items()
    )
    
    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'rover_desc', '-z', '0.32'],
        output='screen'
    )
    
    # Bridge ROS 2 and Gazebo Topics

    bridge_params = os.path.join(pkg_rover_desc, 'config', 'ros_gz_bridge_gazebo.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/model/mine_rover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/default/model/rover_desc/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/rover_desc/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/world/default/model/rover_desc/link/imu_link/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/empty/model/rover_desc/link/base_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/empty/model/rover_desc/link/base_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/empty/model/rover_desc/link/base_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'

        ],        
        output='screen'
    )  
       
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        gazebo,
        robot_state_publisher,       
        spawn_robot,
        bridge,
        rviz_node
    ])

