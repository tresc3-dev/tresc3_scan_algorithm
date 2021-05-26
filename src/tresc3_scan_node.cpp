#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

class Tresc3Scanner {
 public:
  Tresc3Scanner();

  void InitROS(void);

  void laserMsgCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  geometry_msgs::Point32 CalPoint(double r, double theta);
  void SetColor(void);

 private:
  // ros
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{"~"};

  ros::Subscriber laser_sub;
  ros::Publisher pcl_pub;

  sensor_msgs::PointCloud pcl_msg;
  sensor_msgs::ChannelFloat32 channel;

  // launch variables
  std::string frame_id = std::string("pcl_data");
  std::string sub_topicname_lidar = std::string("scan");
  std::string pub_topicname_pcl = std::string("pcl_data");
  std::string direction = std::string("up");
  double velocity = 0.0;
  double height = 0.0;

  // local variables
  ros::Time last_scan_time;
  double z;
  int r = 255;
  int g = 0;
  int b = 0;
  unsigned int color = r * 65536 + g * 256 + b;
  float float_rgb = *reinterpret_cast<float*>(&color);
};

Tresc3Scanner::Tresc3Scanner() {
  // ros init
  InitROS();
}

void Tresc3Scanner::InitROS(void) {
  nh_priv.param("frame_id", frame_id, frame_id);
  nh_priv.param("sub_topicname_lidar", sub_topicname_lidar,
                sub_topicname_lidar);
  nh_priv.param("pub_topicname_pcl", pub_topicname_pcl, pub_topicname_pcl);
  nh_priv.param("direction", direction, direction);
  nh_priv.param("velocity", velocity, velocity);
  nh_priv.param("height", height, height);

  // set initial value
  if (direction == "up") {
    velocity *= -1;
  } else if (direction == "down") {
    velocity *= 1;
  } else {
    ros::shutdown();
  }

  z = height;

  laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &Tresc3Scanner::laserMsgCallback, this);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud>("/pcl_data", 10);
  pcl_msg.header.frame_id = frame_id;
  channel.name = "rgb";
}

void Tresc3Scanner::laserMsgCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  ros::Time scan_time = msg->header.stamp;
  if (!msg->header.seq) {
    last_scan_time = scan_time;
  }
  z += velocity * (scan_time - last_scan_time).toSec();
  int num_data = msg->ranges.size();
  if (num_data > 0) {
    pcl_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < num_data; i++) {
      geometry_msgs::Point32 p =
          CalPoint(msg->ranges[i], msg->angle_min + msg->angle_increment * i);
      pcl_msg.points.push_back(p);
      channel.values.push_back(float_rgb);
    }
    pcl_msg.channels.clear();
    pcl_msg.channels.push_back(channel);
    pcl_pub.publish(pcl_msg);
  }
  SetColor();
  last_scan_time = scan_time;
}

geometry_msgs::Point32 Tresc3Scanner::CalPoint(double r, double theta) {
  geometry_msgs::Point32 p;
  p.x = r * cos(theta);
  p.y = r * sin(theta);
  p.z = z;

  return p;
}

void Tresc3Scanner::SetColor(void) {
  if (r > 0 && b == 0) {
    r--;
    g++;
  } else if (g > 0 && r == 0) {
    g--;
    b++;
  } else if (b > 0 && g == 0) {
    r++;
    b--;
  }
  color = r * 65536 + g * 256 + b;
  float_rgb = *reinterpret_cast<float*>(&color);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Tresc3_scanner");

  Tresc3Scanner scanner;

  ros::spin();

  return 0;
}