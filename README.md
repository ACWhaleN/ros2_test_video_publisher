# ros2_test_video_publisher
## Introduce
The ros2_test_video_publisher is a ROS2 node designed to publish images from a video file. It is implemented in C++ and uses the ROS2 framework for communication, OpenCV for video processing, and cv_bridge for converting between ROS2 messages and OpenCV images.

Upon initialization, the node declares parameters for the video file path and a flag for controlling the video processing loop. It also creates a publisher for sensor_msgs::msg::Image messages.

The node starts a separate thread for video processing. In this thread, it continuously reads frames from the video file, converts each frame to a sensor_msgs::msg::Image message, and publishes the message. The loop continues until the end of the video file is reached or the node is shut down.

This node is a useful tool for testing image processing algorithms in ROS2. It allows developers to easily feed video data into their systems under controlled conditions, making it easier to reproduce and analyze results. It can be used with any video file, making it versatile for different testing scenarios.

Please note that the node assumes the video file is located at a specific path on the system. This path can be configured through the node’s parameters. The node also includes error checking to handle cases where the video file cannot be opened.

## To configure the ros2_test_video_publisher node:

Modify the video_path parameter in the config/default.yaml file to specify the directory of the video file.
Set the keep_processing parameter to true if you want the video to loop continuously.
## To run the node:

'''sh
colcon build --symlink-install
source install/setup.bash
ros2 launch image_pub image_pub.launch.py
'''

Or, you can use the provided script:

'''sh
bash run.sh
'''

This setup will allow you to test image processing algorithms by publishing video data as ROS2 messages. If you encounter any issues or have further questions, don’t hesitate to reach out for assistance. Good luck with your development!
