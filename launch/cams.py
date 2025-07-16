from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import glob
import cv2

def auto_detect_cameras(context, *args, **kwargs):
    """Auto-detect available cameras and create nodes"""
    nodes = []
    
    # Get launch configuration values
    image_width = LaunchConfiguration('image_width').perform(context)
    image_height = LaunchConfiguration('image_height').perform(context)
    framerate = LaunchConfiguration('framerate').perform(context)
    auto_detect = LaunchConfiguration('auto_detect').perform(context)
    
    if auto_detect.lower() == 'true':
        # Auto-detect available video devices
        video_devices = sorted(glob.glob('/dev/video*'))
        print(f"Auto-detected video devices: {video_devices}")
        
        for i, device in enumerate(video_devices):
            # Simple check if device is a camera (not just any video device)
            try:
                cap = cv2.VideoCapture(device)
                if cap.isOpened():
                    camera_node = Node(
                        package='v4l2_camera',
                        executable='v4l2_camera_node',
                        name=f'camera{i}_node',
                        namespace=f'camera{i}',
                        parameters=[
                            {'video_device': device},
                            {'image_size': [int(image_width), int(image_height)]},
                            {'framerate': float(framerate)},
                            {'camera_frame_id': f'camera{i}_link'},
                            {'qos_overrides': {
                                '/image_raw/compressed': {
                                    'publisher': {
                                        'reliability': 'best_effort',
                                        'durability': 'volatile',
                                        'history': 'keep_last',
                                        'depth': 1
                                    }
                                }
                            }}
                        ]
                    )
                    nodes.append(camera_node)
                    print(f"Auto-added camera {i} with device {device}")
                    cap.release()
                else:
                    print(f"Device {device} could not be opened")
            except Exception as e:
                print(f"Error checking device {device}: {e}")
    else:
        # Manual camera configuration
        camera_configs = [
            {'id': 0, 'device_arg': 'cam0_device', 'enable_arg': 'enable_cam0'},
            {'id': 1, 'device_arg': 'cam1_device', 'enable_arg': 'enable_cam1'},
            {'id': 2, 'device_arg': 'cam2_device', 'enable_arg': 'enable_cam2'},
        ]
        
        for config in camera_configs:
            try:
                enabled = LaunchConfiguration(config['enable_arg']).perform(context)
                if enabled.lower() == 'true':
                    device = LaunchConfiguration(config['device_arg']).perform(context)
                    if os.path.exists(device):
                        camera_node = Node(
                            package='v4l2_camera',
                            executable='v4l2_camera_node',
                            name=f'camera{config["id"]}_node',
                            namespace=f'camera{config["id"]}',
                            parameters=[
                                {'video_device': device},
                                {'image_size': [int(image_width), int(image_height)]},
                                {'framerate': float(framerate)},
                                {'camera_frame_id': f'camera{config["id"]}_link'},
                                {'qos_overrides': {
                                    '/image_raw/compressed': {
                                        'publisher': {
                                            'reliability': 'best_effort',
                                            'durability': 'volatile',
                                            'history': 'keep_last',
                                            'depth': 1
                                        }
                                    }
                                }}
                            ]
                        )
                        nodes.append(camera_node)
                        print(f"Added camera {config['id']} with device {device}")
            except Exception as e:
                print(f"Camera {config['id']} not configured: {e}")
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_detect',
            default_value='false',
            description='Auto-detect available cameras'
        ),
        DeclareLaunchArgument(
            'enable_cam0',
            default_value='true',
            description='Enable camera 0'
        ),
        DeclareLaunchArgument(
            'enable_cam1',
            default_value='false',
            description='Enable camera 1'
        ),
        DeclareLaunchArgument(
            'enable_cam2',
            default_value='false',
            description='Enable camera 2'
        ),
        DeclareLaunchArgument(
            'cam0_device',
            default_value='/dev/video0',
            description='Camera 0 device'
        ),
        DeclareLaunchArgument(
            'cam1_device',
            default_value='/dev/video1',
            description='Camera 1 device'
        ),
        DeclareLaunchArgument(
            'cam2_device',
            default_value='/dev/video2',
            description='Camera 2 device'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='Image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='Image height'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Camera framerate'
        ),
        
        # Auto-detect or manually configure cameras
        OpaqueFunction(function=auto_detect_cameras)
    ])