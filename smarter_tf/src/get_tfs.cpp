#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>  //tf::poseEigenToMsg 
#include <tf_conversions/tf_eigen.h> // tf::transformTFToEigen
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker TFToMarkerArrow(const tf::Transform &transform, const std::string &ns, const std::string &frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  // marker.pose is of type geometry_msgs::Pose

  // This is ROS in a nutshell... tons of ways to represent a pose, here we take the detour of converting it from a tf::Transform to an Eigen::Affine3d which we then turn into a geometry_msgs::Pose!!! :-)
  Eigen::Affine3d T;
  tf::transformTFToEigen(transform, T);
  tf::poseEigenToMsg(T, marker.pose);
  
  marker.scale.x = 1.0;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  
  return marker;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_marker");
  ros::NodeHandle params ("~");
  ros::NodeHandle nh_;
  ros::Publisher marker_pub;

  marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

  tf::TransformListener listener;
  
  ros::Rate r(100); // Very fast compared to send_tfs...
  while (ros::ok()) {
    visualization_msgs::Marker marker;

    // Query the /tf and find the pose between different frames and draw an arrow
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world", "/T2",  
                               ros::Time::now()-ros::Duration(0.5),
			       transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    marker = TFToMarkerArrow(transform, "T2", "world");
    marker_pub.publish( marker );

    try{
      listener.lookupTransform("/world", "/T3",  
                               ros::Time::now()-ros::Duration(0.5),
			       transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    marker = TFToMarkerArrow(transform, "T3", "world");
    marker_pub.publish( marker );
  
    
    r.sleep();
    ros::spinOnce();
  }
}
