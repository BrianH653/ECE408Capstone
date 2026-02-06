from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Shared arguments
    device_webcam = DeclareLaunchArgument('device_webcam', default_value='/dev/video0')

    width  = DeclareLaunchArgument('width',  default_value='640')
    height = DeclareLaunchArgument('height', default_value='360')
    fps    = DeclareLaunchArgument('fps',    default_value='25')

    webcam_group = GroupAction([
        PushRosNamespace('webcam'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam_cam',
            parameters=[{
                'video_device': LaunchConfiguration('device_webcam'),
                'image_width': LaunchConfiguration('width'),
                'image_height': LaunchConfiguration('height'),
                'framerate': LaunchConfiguration('fps'),
            }],
            output='screen'
        ),
    ])

    # Thermal camera: publish from ht301_hacklib instead of v4l2_camera reading /dev/video2 directly
    thermal_group = GroupAction([
        PushRosNamespace('usbcam'),
        Node(
            package='ht301_thermal_ros',          # <-- your package name (see notes)
            executable='ht301_thermal_publisher', # <-- installed entrypoint
            name='ht301_thermal',
            parameters=[{
                'fps': LaunchConfiguration('fps'),
                'upscale': 2,
                'colormap_index': 3,
                'orientation': 0,
                'landscape': True,
                'frame_id': 'thermal_optical_frame',
            }],
            output='screen'
        ),
    ])

    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    return LaunchDescription([
        device_webcam,
        width,
        height,
        fps,
        webcam_group,
        thermal_group,
        web_video
    ])
