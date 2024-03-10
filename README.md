# ROS2 Test Video Publisher

## Overview
The `ros2_test_video_publisher` node is designed to facilitate the testing of image processing algorithms within a ROS2 environment. It publishes images from a video file as `sensor_msgs/msg/Image` messages, making it easier for developers to feed video data into their systems under controlled conditions.

## Features
- Publishes `sensor_msgs/msg/Image` messages from a video file.
- Configurable video file path and loop processing via parameters.
- Error checking for video file availability.

## Usage
After launching the node, the `/test/image_raw` topic is published by the `/test/image_publisher` node.

### Parameters
- `video_path`: Specify the directory of the video file.
- `keep_processing`: Set to `true` to enable continuous looping of the video.

### Running the Node
To run the `ros2_test_video_publisher` node, use the following commands:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch image_pub image_pub.launch.py
```

Alternatively, you can execute the provided script:
```
bash run.sh
```
Support
For further assistance or questions, feel free to reach out.
