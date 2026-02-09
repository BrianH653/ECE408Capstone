from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # shared camera arguments
    # device_webcam = DeclareLaunchArgument('device_webcam', default_value='/dev/video0')
    device_webcam = DeclareLaunchArgument(
        'device_webcam',
        default_value='/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-720P_SN0001-video-index0'
    )

    width  = DeclareLaunchArgument('width',  default_value='640')
    height = DeclareLaunchArgument('height', default_value='360')
    fps    = DeclareLaunchArgument('fps',    default_value='25')

    # IR camera arguments only
    thermal_overlay_fps    = DeclareLaunchArgument('thermal_overlay_fps',    default_value='false') # only from camera to pi (not streaming over network)
    thermal_overlay_min    = DeclareLaunchArgument('thermal_overlay_min',    default_value='false')
    thermal_overlay_max    = DeclareLaunchArgument('thermal_overlay_max',    default_value='true')
    thermal_overlay_center = DeclareLaunchArgument('thermal_overlay_center', default_value='false')
    thermal_overlay_bar    = DeclareLaunchArgument('thermal_overlay_bar',    default_value='true')
    thermal_overlay_text   = DeclareLaunchArgument('thermal_overlay_text',   default_value='true')

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

    thermal_group = GroupAction([
        PushRosNamespace('usbcam'),
        Node(
            package='ht301_thermal_ros',
            executable='ht301_thermal_publisher',
            name='ht301_thermal',
            parameters=[{
                'fps': LaunchConfiguration('fps'),
                'out_width': LaunchConfiguration('width'),
                'out_height': LaunchConfiguration('height'),

                'colormap_index': 3,
                'orientation': 0,
                'landscape': True,
                'frame_id': 'thermal_optical_frame',

                'overlay_fps': LaunchConfiguration('thermal_overlay_fps'),
                'overlay_min': LaunchConfiguration('thermal_overlay_min'),
                'overlay_max': LaunchConfiguration('thermal_overlay_max'),
                'overlay_center': LaunchConfiguration('thermal_overlay_center'),
                'overlay_bar': LaunchConfiguration('thermal_overlay_bar'),
                'overlay_text': LaunchConfiguration('thermal_overlay_text'),
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

        thermal_overlay_fps,
        thermal_overlay_min,
        thermal_overlay_max,
        thermal_overlay_center,
        thermal_overlay_bar,
        thermal_overlay_text,

        webcam_group,
        thermal_group,
        web_video
    ])
