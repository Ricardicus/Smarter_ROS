#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher(const std::string& folder_path)
  : Node("image_publisher_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ImagePublisher::timerCallback, this));
    folder_path_ = folder_path;
  }

private:
  void timerCallback()
  {
    std::string folder = folder_path_ + "/" + std::to_string(count_) + ".jpg";
    cv::Mat img = cv::imread(folder);
    if (img.empty()) {
      std::cout << "End of images" << std::endl;
      rclcpp::shutdown();
      return;
    }

    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    pub_->publish(*msg_out);
    std::cout << "Published image nbr " << count_ << std::endl;

    count_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string folder_path_;
  int count_ = 1;
};

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << "Please provide folder path as input argument" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
