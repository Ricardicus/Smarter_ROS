#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

// This uses a global variable, you can also wrap everything in a class.
ros::Publisher pub;

// Whenever there is an image on topic /usb_cam/raw_image" this function will be called
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Got an image!"); // Output to the screen. There are different levels of output ROS_WARN, ROS_ERROR, ROS_DEBUG etc.

  // Here we do the conversion from color to gray scale

  // Convert from ROS sensor_msgs::Image type to an open cv image.
  cv_bridge::CvImageConstPtr cvPtrImgRaw;
  try
  {
    cvPtrImgRaw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGRA8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img = cvPtrImgRaw->image;

  // Change to gray scale
  cv::Mat img_gray;
  
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);

  // Publish the gray image, convert back from open cv image type to sensor_msgs::Image first.
  sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_gray).toImageMsg();

  pub.publish(msg_out);
}

int main(int argc, char **argv)
{
  // Must have this.
  ros::init(argc, argv, "gray_scale_converter_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1000, imageCallback);

 /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  pub = n.advertise<sensor_msgs::Image>("/image_gray", 1000);

  
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
