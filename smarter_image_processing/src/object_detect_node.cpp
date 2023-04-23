#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"

class AnimalDetector : public rclcpp::Node
{
public:
  AnimalDetector(const std::string& cascade_path)
  : Node("object_detector_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_objects", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/usb_cam/image_raw", 10, std::bind(&AnimalDetector::imageCallback, this, std::placeholders::_1));

    // Load the object detection cascade
    if (!cascade_.load(cascade_path))
    {
      RCLCPP_ERROR(get_logger(), "Error loading cascade");
    }
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
    std::vector<cv::Rect> objects;

    // Detect objects in the image
    cascade_.detectMultiScale(img, objects);

    // Draw rectangles around detected objects
    for (const auto& animal : objects)
    {
      cv::rectangle(img, animal, cv::Scalar(0, 255, 0), 2);
    }

    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
    pub_->publish(*msg_out);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  cv::CascadeClassifier cascade_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <path_to_object_cascade.xml>" << std::endl;
    return 1;
  }

  rclcpp::spin(std::make_shared<AnimalDetector>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
