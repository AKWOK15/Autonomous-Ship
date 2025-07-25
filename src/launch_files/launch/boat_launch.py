from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera image width'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera image height'
    )
    
    # Ultrasonic Sensor Node
    ultrasonic_sensor_node = Node(
        #Found in package.xml
        package='ultrasonic_sensor',
        #Found in CMakeLists.txt
        executable='ultrasonic_node',
        # arguments=['--ros-args', '--log-level', 'ERROR'],
        name='ultrasonic_node',
        parameters=[
            {
                'trigger_pin': 18,
                'echo_pin': 24
            }
        ],
    )
    # I have a custom camera node that also subscribes to /camera/image_raw, so I need to pick between the two
    # V4L2 Camera Node
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        # arguments=['--ros-args', '--log-level', 'ERROR'],
        name='v4l2_camera',
        parameters=[
            {
                'video_device': LaunchConfiguration('camera_device'),
                'image_width': LaunchConfiguration('camera_width'),
                'image_height': LaunchConfiguration('camera_height'),
                'pixel_format': 'YUYV',
                'camera_frame_id': 'camera_link',
                'fps': 30.0
            }
        ],
        #Remapping is used so that you can change topics, action without modifying source code
        #Color detection node listens on  /camera/image_raw
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ],
    )
    
    colors_detection_node = Node(
        package='camera',
        executable='colors_detection_node',
        # arguments=['--ros-args', '--log-level', 'ERROR'],
        name='colors_detection_node',
        parameters=[
            {
                'turn_speed': 0.5,
                'min_contour_area': 500
            }
        ],
    )

    sensor_fusion = Node(
        package='sensor_fusion',
        executable='fusion_node',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )
    
    return LaunchDescription([
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        ultrasonic_sensor_node,
        sensor_fusion,
        v4l2_camera_node,
        colors_detection_node
    ])
