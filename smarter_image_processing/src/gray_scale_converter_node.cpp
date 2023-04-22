#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class GrayScaleConverter : public rclcpp::Node
{
public:
  GrayScaleConverter()
  : Node("gray_scale_converter_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_gray", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/usb_cam/image_raw", 10, std::bind(&GrayScaleConverter::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Got an image!");

    cv_bridge::CvImageConstPtr cvPtrImgRaw;
    try
    {
      cvPtrImgRaw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cvPtrImgRaw->image;
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(msg->header, "mono8", img_gray).toImageMsg();
    pub_->publish(*msg_out);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GrayScaleConverter>());
  rclcpp::shutdown();
  return 0;
}


