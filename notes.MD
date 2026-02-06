Goal - 
1. Try to use comrpessed images when sending video to web server
2. build package from source and publish compressed images only. 


test video from usb camera:

v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=640,height=480,pixelformat=MJPG \
  --stream-mmap --stream-count=300 --stream-to=video.mjpg
  
  
  
Installed on pi 4 (ubuntu 24.04 LTS Desktop) with ros2 jazzy

*camera utils installation*
- sudo apt install v4l-utils


*Ros2 package installations*
- sudo apt install ros-${ROS_DISTRO}-v4l2-camera
- sudo apt install ros-${ROS_DISTRO}-image-transport-plugins
- sudo apt install ros-${ROS_DISTRO}-web-video-server





*Start RAW video streaming*

1. Run v4l2 camera node
    ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0

2. Run video web server
    ros2 run web_video_server web_video_server --ros-args \
  -p port:=8080 \
  -p default_stream_type:=mjpeg \
  -p qos_profile:=sensor_data

view available streams: http://localhost:8080


Ros packages for reference:
- https://gitlab.com/boldhearts/ros2_v4l2_camera
- https://github.com/RobotWebTools/web_video_server
