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
    
    # V4L2 Camera Node
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
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
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ]
    )
    
    # Color Detection Node
    color_detection_node = Node(
        package='camera',
        executable='color_detection_node',
        name='color_detection_node',
        parameters=[
            {
                'hue_low': 100,      # Blue color detection (adjust for your target color)
                'hue_high': 130,
                'sat_low': 50,
                'sat_high': 255,
                'val_low': 50,
                'val_high': 255,
                'turn_speed': 0.5,
                'min_contour_area': 500
            }
        ],
        remappings=[
            ('/cmd_vel', '/ship/cmd_vel')  # Remap to your ship's command topic
        ]
    )
    
    return LaunchDescription([
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        v4l2_camera_node,
        color_detection_node
    ])
