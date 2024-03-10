// Copyright 2024 NongJingyu
// Licensed under the Apache Licence 2.0 License.

/**
 * @brief This file contains the implementation of the ImagePubNode class, which is a ROS2 node that publishes images from a video file.
 */

// ROS2
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


using namespace std::chrono_literals;

class ImagePubNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the ImagePubNode class.
   */
  ImagePubNode()
      : Node("ros2_test_video_publisher_node"),
        video_path_("/root/rm_perception/src/ros2_test_video_publisher/data/video/test.mp4"),
        keep_processing_(true) {  // 初始化标志位为真
    this->declare_parameter<std::string>("video_path",
                                         video_path_);  // 默认视频路径
    this->declare_parameter<bool>("keep_processing",
                                  keep_processing_);  // 默认标志位为真
    image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

    // 启动视频处理线程
    video_thread_ = std::make_unique<std::thread>(
        &ImagePubNode::process_video_control, this);
  }

  /**
   * @brief Destructor for the ImagePubNode class.
   */
  ~ImagePubNode() override {
    keep_processing_ = false;  // 确保process_video退出循环
    if (video_thread_ && video_thread_->joinable()) {
      video_thread_->join();
    }
  }

  /**
   * @brief Controls the video processing loop.
   */
  void process_video_control() {
    while (keep_processing_) {
      process_video();  // 循环调用process_video直到keep_processing_变为false
    }
  }

 private:
  /**
   * @brief Publishes a frame as a sensor_msgs::msg::Image message.
   * @param frame The frame to be published.
   */
  void publish_frame(const cv::Mat& frame) {
    if (!frame.empty()) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                     .toImageMsg();
      image_pub_->publish(*msg);
    }
  }

  /**
   * @brief Processes the video file and publishes frames.
   */
  void process_video() {
    // 确保能读取到参数
    RCLCPP_INFO(this->get_logger(), "Start to read parameters. ");
    this->get_parameter("video_path", video_path_);
    RCLCPP_INFO(this->get_logger(), "video_path: %s, keep_processing: %d",
                video_path_.c_str(), keep_processing_);

    cv::VideoCapture cap(video_path_);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s",
                   video_path_.c_str());
      return;
    }
    if (keep_processing_) {
      RCLCPP_INFO(this->get_logger(), "Start to play video file ");
      cv::Mat frame;
      while (rclcpp::ok()) {
        cap >> frame;  // 读取当前帧
        if (frame.empty()) {
          RCLCPP_INFO(this->get_logger(), "End of video file");
          break;  // Don't return, or it will stop the loop. Let it iterate.
        }

        publish_frame(frame);  // 调用publish_frame发布当前帧

        rclcpp::sleep_for(11ms);  // fps
      }
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Video processing is stopped");
    }
    this->get_parameter("keep_processing", keep_processing_);
    // 接着关闭并重新打开视频文件，为下一次播放做准备
    cap.release();
    rclcpp::sleep_for(1s);  // 等待一秒后重新开始播放视频
  }

  std::string video_path_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::unique_ptr<std::thread> video_thread_;
  bool keep_processing_;  // 控制视频处理线程是否继续运行的标志位
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePubNode>();

  // 使用rclcpp::spin使节点保持运行，以便它可以处理其他任务，比如响应服务请求等
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
