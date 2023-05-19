#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class SendTFsNode : public rclcpp::Node
{
public:
  SendTFsNode() : Node("send_tfs")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize the transforms
    T1_.setIdentity();
    T2_.setIdentity();
    T3_.setIdentity();

    T2_.translation() = Eigen::Vector3d(1, 0, 0);
    T3_.translation() = Eigen::Vector3d(1, 0, 0);

    T_incr_.setIdentity();
    // Add some rotation around z-axis
    T_incr_.linear() = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Create a timer to periodically update and send the transforms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SendTFsNode::updateTransforms, this));
  }

private:
  void updateTransforms()
  {
    // Update T1 and T2
    T1_ = T1_ * T_incr_;
    T2_ = T2_ * T_incr_;

    geometry_msgs::msg::TransformStamped transform;

    // Send the transforms
    sendTransform(T1_, "world", "T1", transform);
    sendTransform(T2_, "T1", "T2", transform);
    sendTransform(T3_, "T2", "T3", transform);
    sendTransform(T1_ * T2_, "world", "T2_global", transform);
    sendTransform(T1_ * T2_ * T3_, "world", "T3_global", transform);
  }


  void sendTransform(const Eigen::Affine3d &T, const std::string &parent_frame, const std::string &child_frame, geometry_msgs::msg::TransformStamped &transform)
  {
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;

    // Manually convert Eigen::Affine3d to geometry_msgs::msg::Transform
    Eigen::Matrix3d eigen_mat = T.rotation();
    tf2::Matrix3x3 mat(eigen_mat(0, 0), eigen_mat(0, 1), eigen_mat(0, 2),
                       eigen_mat(1, 0), eigen_mat(1, 1), eigen_mat(1, 2),
                       eigen_mat(2, 0), eigen_mat(2, 1), eigen_mat(2, 2));

    tf2::Quaternion quat;
    mat.getRotation(quat);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    transform.transform.translation.x = T.translation().x();
    transform.transform.translation.y = T.translation().y();
    transform.transform.translation.z = T.translation().z();

    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Eigen::Affine3d T1_, T2_, T3_, T_incr_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendTFsNode>());
  rclcpp::shutdown();
  return 0;
}
