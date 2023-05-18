
#include <visualization_msgs/msg/marker.hpp>
#include <resource_retriever/retriever.h>
#include "rclcpp/rclcpp.hpp"

class DebugMarker : public rclcpp::Node
{
public:
  DebugMarker() : Node("debug_marker")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    std::string mesh_resource = "package://cititruck_description/meshes/sick_s300_laser.dae";
    resource_retriever::Retriever ret;
    resource_retriever::MemoryResource resource;
    try
    {
      resource = ret.get(mesh_resource);
    }
    catch (resource_retriever::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to retrieve file: %s", e.what());
      return;
    }
    std::cout << "mesh found : " << mesh_resource << std::endl;

   timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DebugMarker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    static double t = 0;

    // Publish pallet marker
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "/world";
      marker.header.stamp = this->now();
      marker.ns = "pallet";
      marker.id = 1;
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 2 + sin(t);
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      marker.mesh_resource = mesh_resource;

      marker_pub_->publish(marker);
    }

    // Publish sphere marker
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "/world";
      marker.header.stamp = this->now();
      marker.ns = "sphere";
      marker.id = 1;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.1 + sin(t);
      marker.color.a = 1.0;
      marker.color.r = 0.5 + 0.5 * sin(t);
      marker.color.g = 0.5 + 0.5 * sin(t - 1);
      marker.color.b = 0.5 + 0.5 * sin(t + 1);

      marker_pub_->publish(marker);
    }

    std::cout << "." << std::flush;
    t += 0.04;
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string mesh_resource;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugMarker>());
  rclcpp::shutdown();
  return 0;
}
