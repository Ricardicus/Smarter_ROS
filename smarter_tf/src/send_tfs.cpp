#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv){
  ros::init(argc, argv, "send_tfs");

  tf::TransformBroadcaster br;

  Eigen::Affine3d T1, T2, T3;
  T1.setIdentity();
  T2.setIdentity();
  T3.setIdentity();

  T2.translation() = Eigen::Vector3d(1,0,0);
  T3.translation() = Eigen::Vector3d(1,0,0);

  Eigen::Affine3d T_incr;
  T_incr.setIdentity();
  // Add some rotation around z-axis
  T_incr.linear() = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  
  ros::Rate r(2); // This is slow to show the interpolation capabilities.
  while (ros::ok()) {

    // Update T1 and T2.
    T1 = T1*T_incr;
    T2 = T2*T_incr;
    
    tf::Transform transform;

    tf::transformEigenToTF(T1, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "T1"));
    tf::transformEigenToTF(T2, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "T1", "T2"));
    tf::transformEigenToTF(T3, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "T2", "T3"));

    // T1 global = T1
    // T2 global = T1*T2
    tf::transformEigenToTF(T1*T2, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "T2_global"));
    tf::transformEigenToTF(T1*T2*T3, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "T3_global"));

    r.sleep();
    ros::spinOnce();
  }
  return 0;
};

