#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <rosbag/bag.h>
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

void parseSickMessage(char *tok, rosbag::Bag &obag){
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
  sensor_msgs::LaserScan scan;
  ros::Time rt(ts);
  scan.header.stamp = rt;
  scan.header.frame_id = "laserscan" + toString(sensor_id);
		
  scan.angle_min = start_ang*M_PI/180.0;
  scan.angle_max = start_ang*M_PI/180.0 + num_dist*ang_inc*M_PI/180.0;
  scan.angle_increment = ang_inc*M_PI/180.0;
	
  scan.range_min = 0.1;
  scan.range_max = range_max;
	
  for(int i=0;i<num_dist;i++) scan.ranges.push_back(r[i]);

  obag.write("/laserscan"+toString(sensor_id),scan.header.stamp,scan);
}

void parseStateMessage(char *tok,
		       rosbag::Bag &obag){
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
	
  // Create two types of ros message, the tf and odometry.
  // tf
  geometry_msgs::TransformStamped state_trans;
  geometry_msgs::Quaternion state_quat = tf::createQuaternionMsgFromYaw(a);
  state_trans.header.stamp = ros::Time(ts);
  state_trans.header.frame_id = "/world";
  state_trans.child_frame_id = "/state_base_link";
	
  state_trans.transform.translation.x = x;
  state_trans.transform.translation.y = y;
  state_trans.transform.translation.z = 0.0;
  state_trans.transform.rotation = state_quat;

  tf::tfMessage tfmsg;
  tfmsg.transforms.push_back(state_trans);
  obag.write("/tf",ros::Time(ts),tfmsg);
	
  // odometry
  nav_msgs::Odometry state;
  state.header.stamp = ros::Time(ts);
  state.header.frame_id = "/world";
		
  state.pose.pose.position.x = x;
  state.pose.pose.position.y = y;
  state.pose.pose.position.z = 0.0;
  state.pose.pose.orientation = state_quat;

  state.child_frame_id = "state_base_link";

  obag.write("/state",state.header.stamp,state);
}


int main (int ac, char* av[])
{
  std::string file_name;
  int protocol_ver;
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
  rosbag::Bag obag;

  std::cout << "Writing to : " << file_name+".bag" << std::endl;
  obag.open(file_name+".bag", rosbag::bagmode::Write); ///output

  while(!infile.eof()) // To get you all the lines.
    {
      getline(infile, str);
			 
      const char* delim = " ";
      char* msg = (char*)str.c_str();
      char *tok = strtok(msg,delim);
      std_msgs::Header header;

      if(tok!=NULL) {
	if(strncmp("sick",tok,4) == 0){
	  parseSickMessage(tok, obag);

	}
	else if(strncmp("state",tok,5) == 0){
	  parseStateMessage(tok, obag);
	}
      }
    }
  
  infile.close();
  obag.close();
  std::cout << "... done." << std::endl;
}
