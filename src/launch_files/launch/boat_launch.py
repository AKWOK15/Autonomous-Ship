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
        package='ultrasonic_sensor',
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
    
    # V4L2 Camera Node - Optimized for Black Object Detection
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
                'fps': 20.0,
                
                # === BALANCED PARAMETERS FOR BLACK OBJECT DETECTION ===
                
                # Auto White Balance - helps with consistent color detection
                'white_balance_temperature': 4600,  # Neutral daylight temperature
                
                
                # # Gain settings - MODERATE increase only
                'gain': 15,  # Much lower gain to avoid noise (start here)
                # # Brightness and Contrast - SUBTLE adjustments
                # 'brightness': 0,   # Keep neutral initially
                'contrast': 45,    # Slight contrast boost (default ~32)
          
            }
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ],
    )
    
    colors_detection_node = Node(
        package='camera',
        executable='colors_detection_node',
        arguments=['--ros-args', '--log-level', 'ERROR'],
        name='colors_detection_node',
        parameters=[
            {
                'turn_speed': 0.5,
                'min_contour_area': 500,
                
                # Add parameters for black detection
                'use_adaptive_thresholding': True,  # Better for low-light objects
                'black_detection_mode': True,      # Special mode for black objects
            }
        ],
    )

    movement_detection_node = Node(
        package='camera',
        executable='movement_detection_node',
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    obstacle_detection_node = Node(
        package='camera',
        executable='obstacle_detection_node',
        parameters=[
            {
                'resize_height':480,
                'resize_width':640
            }
        ]
        # arguments=['--ros-args', '--log-level', 'ERROR']
    )
    
    sensor_fusion = Node(
        package='sensor_fusion',
        executable='fusion_node',
        # arguments=['--ros-args', '--log-level', 'ERROR']
    )
    
    return LaunchDescription([
        camera_device_arg,
        camera_width_arg, 
        camera_height_arg,
        # ultrasonic_sensor_node,
        # sensor_fusion,
        v4l2_camera_node,
        # colors_detection_node,
        # movement_detection_node
        obstacle_detection_node
    ])