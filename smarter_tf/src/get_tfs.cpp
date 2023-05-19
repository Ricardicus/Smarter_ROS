#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class DebugMarker : public rclcpp::Node
{
public:
  DebugMarker() : Node("debug_marker"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DebugMarker::publishMarkers, this));
  }

private:
  visualization_msgs::msg::Marker TFToMarkerArrow(const tf2::Stamped<tf2::Transform> &transform, const std::string &ns, const std::string &frame_id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = ns;
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Convert tf2::Transform to geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();

    marker.pose = pose;

    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
  }

  void publishMarkers()
  {
    visualization_msgs::msg::Marker marker;

    // Query the /tf and find the pose between different frames and draw an arrow
    tf2::Stamped<tf2::Transform> transform;
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_.lookupTransform("/world", "/T2", tf2::TimePointZero);
      tf2::fromMsg(transform_stamped, transform);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    marker = TFToMarkerArrow(transform, "T2", "world");
    marker_pub_->publish(marker);

    try
    {
      transform_stamped = tf_buffer_.lookupTransform("/world", "/T3", tf2::TimePointZero);
      tf2::fromMsg(transform_stamped, transform);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    marker = TFToMarkerArrow(transform, "T3", "world");
    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugMarker>());
  rclcpp::shutdown();
  return 0;
}
