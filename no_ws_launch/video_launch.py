from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # v4l2 params
        DeclareLaunchArgument('device',         default_value='/dev/video2'),
        DeclareLaunchArgument('width',          default_value='640'), #640
        DeclareLaunchArgument('height',         default_value='360'), #480
        DeclareLaunchArgument('topic_base',     default_value='/camera'),
        DeclareLaunchArgument('fps',            default_value='20'),
        # DeclareLaunchArgument('time_per_frame',            default_value='[5, 15]'),
        
                
        # web video server params
        # DeclareLaunchArgument('port',           default_value='8080'), # already defaults to port 8080
        # DeclareLaunchArgument('stream',         default_value='mjpeg'), # already defaults to mjpeg

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='uvc_cam',
            parameters=[{
                'video_device': LaunchConfiguration('device'),
                'image_width': LaunchConfiguration('width'),
                'image_height': LaunchConfiguration('height'),
                'framerate': LaunchConfiguration('fps'),
                # 'time_per_frame': LaunchConfiguration('time_per_frame'),
            }],
            output='screen'
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            # parameters=[{
            #     'port': LaunchConfiguration('port'),
            #     'default_stream_type': LaunchConfiguration('stream'),
            # }],
            output='screen'
        ),
    ])
