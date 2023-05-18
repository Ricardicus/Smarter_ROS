#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <boost/program_options.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace po = boost::program_options;
using namespace std;

template<class T> std::string toString (const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error ("::toString()");

    return o.str ();
}

void parseSickMessage(char *tok, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher){
  double ts;
  int scan_num;
  int sensor_id;
  double range_max;
  double start_ang;
  double ang_inc;
  int num_dist;
  std::vector<double> r;

  tok = strtok(NULL," ");
  if(tok!=NULL) ts = atof(tok);  ///TS
	
  tok = strtok(NULL," ");
  if(tok!=NULL) scan_num = atoi(tok); ///Scan number (rotating 0...255)
  
  (void)scan_num;
  tok = strtok(NULL," ");
  if(tok!=NULL) sensor_id = atoi(tok); ///Sensor ID
	
  tok = strtok(NULL," ");
  if(tok!=NULL) range_max = atof(tok); ///Range max value

  tok = strtok(NULL," ");
  if(tok!=NULL) num_dist = atoi(tok); ///Number of distances
	
  tok = strtok(NULL," ");
  if(tok!=NULL) start_ang = atof(tok); ///Start Ang (deg)
	
  tok = strtok(NULL," ");
  if(tok!=NULL) ang_inc = atof(tok); ///Angle incr
	
  tok = strtok(NULL," ");
  while(tok!=NULL){
    double range = atof(tok);
    if(tok != NULL) r.push_back(range);
    tok = strtok(NULL," ");
  }

  // Done reading. Create a ROS message.
  sensor_msgs::msg::LaserScan scan;
  rclcpp::Time rt(ts);
  scan.header.stamp = rt;
  scan.header.frame_id = "laserscan" + toString(sensor_id);
		
  scan.angle_min = start_ang*M_PI/180.0;
  scan.angle_max = start_ang*M_PI/180.0 + num_dist*ang_inc*M_PI/180.0;
  scan.angle_increment = ang_inc*M_PI/180.0;
	
  scan.range_min = 0.1;
  scan.range_max = range_max;
	
  for(int i=0;i<num_dist;i++) scan.ranges.push_back(r[i]);

  publisher->publish(scan);
}

void parseStateMessage(char *tok,
		       rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher){
  double ts;
  double x;
  double y;
  double a;
  int pos_known;
  int certainty;

  tok = strtok(NULL," ");
  if(tok!=NULL) ts = atof(tok);
  tok = strtok(NULL," ");
  if(tok!=NULL) x = atof(tok);
  tok = strtok(NULL," ");
  if(tok!=NULL) y = atof(tok);
  tok = strtok(NULL," ");
  if(tok!=NULL) a = atof(tok);
  tok = strtok(NULL," ");
  if(tok!=NULL) pos_known = atoi(tok);
  tok = strtok(NULL," ");
  if(tok!=NULL) certainty = atoi(tok);

  (void)pos_known;
  (void)certainty;
  // Create two types of ros message, the tf and odometry.
  // tf
  geometry_msgs::msg::TransformStamped state_trans;
  tf2::Quaternion state_quat;
  state_quat.setRPY(0, 0, a);
  state_trans.header.stamp = rclcpp::Time(ts);
  state_trans.header.frame_id = "/world";
  state_trans.child_frame_id = "/state_base_link";
	
  state_trans.transform.translation.x = x;
  state_trans.transform.translation.y = y;
  state_trans.transform.translation.z = 0.0;
  state_trans.transform.rotation.x = state_quat.x();
  state_trans.transform.rotation.y = state_quat.y();
  state_trans.transform.rotation.z = state_quat.z();
  state_trans.transform.rotation.w = state_quat.w();

  // odometry
  nav_msgs::msg::Odometry state;
  state.header.stamp = rclcpp::Time(ts);
  state.header.frame_id = "/world";
		
  state.pose.pose.position.x = x;
  state.pose.pose.position.y = y;
  state.pose.pose.position.z = 0.0;
  state.pose.pose.orientation.x = state_quat.x();
  state.pose.pose.orientation.y = state_quat.y();
  state.pose.pose.orientation.z = state_quat.z();
  state.pose.pose.orientation.w = state_quat.w();

  state.child_frame_id = "state_base_link";

  publisher->publish(state);
}


int main (int ac, char* av[])
{
  rclcpp::init(ac, av);

  std::string file_name;
  //int protocol_ver;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("file_name", po::value<string>(&file_name)->required(), "input file name")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);    

  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  string str;
  ifstream infile;
  std::cout << "Loading : " << file_name << std::endl;
  infile.open (file_name.c_str()); ///<Input 
  std::cout << "... done." << std::endl;

  auto node = rclcpp::Node::make_shared("ros2_converter");
  auto laser_scan_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>("/laserscan", 10);
  auto odometry_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/state", 10);

  while(!infile.eof()) // To get you all the lines.
    {
      getline(infile, str);
			 
      const char* delim = " ";
      char* msg = (char*)str.c_str();
      char *tok = strtok(msg,delim);
      std_msgs::msg::Header header;

      if(tok!=NULL) {
	if(strncmp("sick",tok,4) == 0){
	  parseSickMessage(tok, laser_scan_publisher);

	}
	else if(strncmp("state",tok,5) == 0){
	  parseStateMessage(tok, odometry_publisher);
	}
      }
    }
  
  infile.close();
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout << "... done." << std::endl;
}
