# Measure Wifi Bandiwdth and drone cpu usage
TODO: Know how to limit framerate. Currently getting wifi bandiwdth of 6.7 Mbps


1. Run the launch file for v4l2 node and web service node. This is for the omen laptop

`ros2 launch video_launch.py`

2. open the stream on a different device like an iPhone (get [laptop_ip] by doing hostname -I)

`[laptop_ip]:8080/stream?topic=/image_raw&type=mjpeg&width=480&height=800&quality=60`

3. Measure bandwidth between v4l2 node and web service (both running on laptop)

`ros2 topic bw /image_raw`     FOR CPU USAGE, NOT WIFI (since nodes exist on same device)

4. Find bandwidth between laptop and phone (service and stream)

`sudo iftop -i wlo1 -P -f 'tcp port 8080'`          (note, extra streams means a linear increase)

- Note, the rightmost three columns next to laptop -> iphone are the current rates
- the Mb/s is the main thing to look at for wifi

