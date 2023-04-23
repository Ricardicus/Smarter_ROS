#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include <mutex>

class ImageSaver : public rclcpp::Node
{
public:
  ImageSaver(const std::string& folder_path)
  : Node("image_subscriber_node"), folder_path_(folder_path), count_(0)
  {
    sub1_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_gray", 10, std::bind(&ImageSaver::imageCallback, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_objects", 10, std::bind(&ImageSaver::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Got an image!");

    cv_bridge::CvImageConstPtr cvPtrImgRaw;
    try
    {
      cvPtrImgRaw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cvPtrImgRaw->image;

    // Lock the mutex to ensure thread-safe access to count_
    std::unique_lock<std::mutex> lock(count_mutex_);
    std::string file_path = folder_path_ + "/" + std::to_string(++count_) + ".jpg";
    lock.unlock(); // Unlock the mutex after updating count_

    cv::imwrite(file_path, img);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2_;
  std::string folder_path_;
  int count_;
  std::mutex count_mutex_; // Mutex to ensure thread-safe access to count_
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("image_saver_node"), "Please provide a folder path as an argument.");
    return 1;
  }
  rclcpp::spin(std::make_shared<ImageSaver>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
