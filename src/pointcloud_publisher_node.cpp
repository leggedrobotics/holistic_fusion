// ROS core
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "pcl_ros/publisher.h"

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

using namespace std;

class PCDGenerator {
 protected:
  string tf_frame_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

 public:
  // ROS messages
  sensor_msgs::PointCloud2 cloud_;

  string file_name_, cloud_topic_;
  double rate_;

  pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;

  ////////////////////////////////////////////////////////////////////////////////
  PCDGenerator() : tf_frame_("/base_link"), private_nh_("~") {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1

    cloud_topic_ = "cloud_pcd";
    pub_.advertise(nh_, cloud_topic_.c_str(), 1);
    private_nh_.param("frame_id", tf_frame_, std::string("/base_link"));
    ROS_INFO("Publishing data on topic %s with frame_id %s.", nh_.resolveName(cloud_topic_).c_str(), tf_frame_.c_str());
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Start
  int start() {
    if (file_name_ == "" || pcl::io::loadPCDFile(file_name_, cloud_) == -1) return (-1);
    cloud_.header.frame_id = tf_frame_;
    return (0);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Spin (!)
  bool spin() {
    std::cout << "Entered spinning method of class." << std::endl;
    int nr_points = cloud_.width * cloud_.height;
    string fields_list = pcl::getFieldsList(cloud_);
    double interval = rate_ * 1e+6;
    while (nh_.ok()) {
      ROS_DEBUG_ONCE("Publishing data with %d points (%s) on topic %s in frame %s.", nr_points, fields_list.c_str(),
                     nh_.resolveName(cloud_topic_).c_str(), cloud_.header.frame_id.c_str());
      cloud_.header.stamp = ros::Time::now();

      if (pub_.getNumSubscribers() > 0) {
        ROS_DEBUG("Publishing data to %d subscribers.", pub_.getNumSubscribers());
        pub_.publish(cloud_);
      } else {
        ros::Duration(0.001).sleep();
        continue;
      }
      std::cout << YELLOW_START << "MapPublisher" << COLOR_END << " Published leica map at time stamp " << ros::Time::now() << std::endl;

      usleep(interval);

      if (interval == 0)  // We only publish once if a 0 seconds interval is given
        break;
    }

    ros::Duration(3.0).sleep();
    return (true);
  }
};

/* ---[ */
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Syntax is: " << argv[0] << " <file.pcd> [publishing_interval (in seconds)]" << std::endl;
    return (-1);
  }

  ros::init(argc, argv, "pcd_to_pointcloud");

  PCDGenerator c;
  c.file_name_ = string(argv[1]);
  c.rate_ = atof(argv[2]);

  if (c.start() == -1) {
    ROS_ERROR("Could not load file %s. Exiting.", argv[1]);
    return (-1);
  }
  ROS_INFO("Loaded a point cloud with %d points (total size is %zu) and the following channels: %s.", c.cloud_.width * c.cloud_.height,
           c.cloud_.data.size(), pcl::getFieldsList(c.cloud_).c_str());
  c.spin();

  return (0);
}