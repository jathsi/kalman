#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <cmath>
#include <vector>


class Roshdl
{

public:
  // Constructor / destructor
   Roshdl(ros::NodeHandle&, ros::NodeHandle&);
  // Running

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_loc_;
  ros::Subscriber scan_sub_;
  ros::Publisher line_publ_;
  ros::Publisher marker_publ_;
  // Parameters
  std::vector<unsigned int> kbs;
  std::vector<double> bearings, cos_bearings, sin_bearings, ranges,xs,ys;
  std::vector<unsigned int> seg1;
  std::vector<unsigned int> indx;
  std::vector<unsigned int> seg2;
  std::vector<unsigned int> seg3;
  double Ranges[720];
  std::string frame_id_;
  std::string scan_topic_;
  bool pub_markers_;
  void ScanarrivalFn(const sensor_msgs::LaserScan::ConstPtr&);
  double pi_to_pi(double);
  double distToPoint(unsigned int, double , double );
  void split(unsigned int,unsigned int );

  // Line extraction
};
