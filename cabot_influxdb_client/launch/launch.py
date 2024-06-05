from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_ROBOT_NAME', default_value='cabot')
        ),
        DeclareLaunchArgument(
            'battery_topic',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_BATTERY_TOPIC', default_value='/cabot/battery')
        ),
        DeclareLaunchArgument(
            'image_left_topic',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_IMAGE_LEFT_TOPIC', default_value='')
        ),
        DeclareLaunchArgument(
            'image_center_topic',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_IMAGE_CENTER_TOPIC', default_value='/camera/color/image_raw')
        ),
        DeclareLaunchArgument(
            'image_right_topic',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_IMAGE_RIGHT_TOPIC', default_value='')
        ),
        DeclareLaunchArgument(
            'pose_interval',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_POSE_INTERVAL', default_value='1.0')
        ),
        DeclareLaunchArgument(
            'cmd_vel_interval',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_CMD_VEL_INTERVAL', default_value='0.2')
        ),
        DeclareLaunchArgument(
            'odom_interval',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_ODOM_INTERVAL', default_value='0.2')
        ),
        DeclareLaunchArgument(
            'diag_agg_interval',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_DIAG_AGG_INTERVAL', default_value='1.0')
        ),
        DeclareLaunchArgument(
            'battery_interval',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_BATTERY_INTERVAL', default_value='1.0')
        ),
        DeclareLaunchArgument(
            'image_interval',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_IMAGE_INTERVAL', default_value='5.0')
        ),
        DeclareLaunchArgument(
            'host',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_HOST', default_value='http://localhost:8086')
        ),
        DeclareLaunchArgument(
            'token',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_TOKEN', default_value='a54a87f7-73a0-4534-9741-ad7ff4e7d111')
        ),
        DeclareLaunchArgument(
            'org',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_ORG', default_value='cabot')
        ),
        DeclareLaunchArgument(
            'bucket',
            default_value=EnvironmentVariable('CABOT_INFLUXDB_BUCKET', default_value='cabot')
        ),
        DeclareLaunchArgument(
            'anchor_file',
            default_value='/home/developer/ros2_ws/src/anchor_file.yaml'
        ),

        Node(
            package='cabot_influxdb_client',
            executable='client_node',
            namespace='/cabot',
            name='client_node',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'battery_topic': LaunchConfiguration('battery_topic'),
                'image_left_topic': LaunchConfiguration('image_left_topic'),
                'image_center_topic': LaunchConfiguration('image_center_topic'),
                'image_right_topic': LaunchConfiguration('image_right_topic'),
                'pose_interval': LaunchConfiguration('pose_interval'),
                'cmd_vel_interval': LaunchConfiguration('cmd_vel_interval'),
                'odom_interval': LaunchConfiguration('odom_interval'),
                'diag_agg_interval': LaunchConfiguration('diag_agg_interval'),
                'battery_interval': LaunchConfiguration('battery_interval'),
                'image_interval': LaunchConfiguration('image_interval'),
                'host': LaunchConfiguration('host'),
                'token': LaunchConfiguration('token'),
                'org': LaunchConfiguration('org'),
                'bucket': LaunchConfiguration('bucket'),
                'anchor_file': LaunchConfiguration('anchor_file'),
            }]
        )
    ])
