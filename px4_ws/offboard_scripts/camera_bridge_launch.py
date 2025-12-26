from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            parameters=[{
                'bridges': [
                    {
                        'ros_topic_name': '/camera/image',
                        'gz_topic_name': '/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
                        'ros_type_name': 'sensor_msgs/msg/Image',
                        'gz_type_name': 'gz.msgs.Image',
                        'direction': 'BIDIRECTIONAL'
                    },
                    {
                        'ros_topic_name': '/camera/camera_info',
                        'gz_topic_name': '/world/forest/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info',
                        'ros_type_name': 'sensor_msgs/msg/CameraInfo',
                        'gz_type_name': 'gz.msgs.CameraInfo',
                        'direction': 'BIDIRECTIONAL'
                    }
                ]
            }],
            output='screen'
        )
    ])