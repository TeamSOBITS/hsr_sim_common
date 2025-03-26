#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>          // for transformToEigen
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <pcl_ros/transforms.h>           // for fromROSMsg, toROSMsg
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define SCAN_MAX 30.0f

class PCLScanCreater : public rclcpp::Node
{
public:
  PCLScanCreater()
  : Node("pcl_scan_creater"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare and retrieve ROS2 parameters (with defaults)
    this->declare_parameter<std::string>("pub_scan_topic_name", "/scan_mixed");
    this->declare_parameter<std::string>("sub_scan_topic_name", "/scan");
    this->declare_parameter<std::string>("sub_point_topic_name", "/camera/depth/points");
    this->declare_parameter<std::string>("laser_frame_name", "base_laser_link");
    this->declare_parameter<std::string>("camera_frame_name", "camera_depth_optical_frame");
    this->declare_parameter<double>("depth_z_min", 0.0);
    this->declare_parameter<double>("depth_z_max", 1.0);
    this->declare_parameter<double>("depth_x_min", 0.0);
    this->declare_parameter<double>("depth_x_max", 1.0);
    this->declare_parameter<double>("point_time_out", 1.0);

    this->get_parameter("pub_scan_topic_name", pub_scan_topic_name_);
    this->get_parameter("sub_scan_topic_name", sub_scan_topic_name_);
    this->get_parameter("sub_point_topic_name", sub_point_topic_name_);
    this->get_parameter("laser_frame_name", laser_frame_name_);
    this->get_parameter("camera_frame_name", camera_frame_name_);
    this->get_parameter("depth_z_min", depth_z_min_);
    this->get_parameter("depth_z_max", depth_z_max_);
    this->get_parameter("depth_x_min", depth_x_min_);
    this->get_parameter("depth_x_max", depth_x_max_);
    this->get_parameter("point_time_out", point_time_out_);

    // Create subscriptions
    sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      sub_scan_topic_name_, 
      rclcpp::QoS(1),
      std::bind(&PCLScanCreater::laserCallback, this, std::placeholders::_1));

    sub_point_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      sub_point_topic_name_,
      rclcpp::QoS(1),
      std::bind(&PCLScanCreater::pointCallback, this, std::placeholders::_1));

    // Create publisher
    pub_mixed_laser_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_scan_topic_name_, 1);

    initialized_flag_ = false;
    last_pcl_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "pcl_scan_creater node started.");
    RCLCPP_INFO(this->get_logger(), 
      "sub_scan_topic_name: %s, sub_point_topic_name: %s, pub_scan_topic_name: %s",
      sub_scan_topic_name_.c_str(), sub_point_topic_name_.c_str(), pub_scan_topic_name_.c_str());
  }

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_in)
  {
    // On the first LaserScan, store it as our baseline
    if(!initialized_flag_) {
      pcl_scan_ = *scan_in;
      initialized_flag_ = true;
    }

    // Start with a copy of the new LaserScan
    mixed_scan_ = *scan_in;

    // If the time since we last updated pcl_scan_ is less than point_time_out_, 
    // fuse the point-cloud-based ranges with the incoming laser data.
    rclcpp::Time now = this->get_clock()->now();
    if((last_pcl_time_ + rclcpp::Duration::from_seconds(point_time_out_)) > now)
    {
      // Combine the ranges by taking the min
      for(size_t i = 0; i < mixed_scan_.ranges.size(); i++) {
        mixed_scan_.ranges[i] = std::min(scan_in->ranges[i], pcl_scan_.ranges[i]);
      }
    }

    // Publish the merged LaserScan
    pub_mixed_laser_->publish(mixed_scan_);
  }

  void pointCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud)
  {
    if(!initialized_flag_) {
      // If we haven't even received the first LaserScan, there's nothing to merge into
      return;
    }

    // Check if a transform from camera_frame_name_ to laser_frame_name_ is possible
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // We do a synchronous lookup. If not available, it will throw
      transform_stamped = tf_buffer_.lookupTransform(
        laser_frame_name_, 
        camera_frame_name_, 
        tf2::TimePointZero
      );
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "PCL_Scan_Creater: canTransform failed: %s", ex.what());
      return;
    }

    // Convert ROS2 sensor_msgs::PointCloud2 -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *cloud_input);

    // Downsample with a voxel grid in XY (Z is set large to not overly reduce)
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_input);
    vg.setLeafSize(0.01f, 0.01f, 1.0f);
    vg.filter(*cloud_downsample);

    // Convert the geometry_msgs transform to an Eigen transform
    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform_stamped.transform);

    // Transform point cloud from camera_frame into laser_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_laser(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*cloud_downsample, *cloud_base_laser, eigen_transform);

    // Filter by X
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut_x(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::PassThrough<pcl::PointXYZ> pass_x;
      pass_x.setInputCloud(cloud_base_laser);
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(depth_x_min_, depth_x_max_);
      pass_x.filter(*cloud_cut_x);
    }

    // Filter by Z
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::PassThrough<pcl::PointXYZ> pass_z;
      pass_z.setInputCloud(cloud_cut_x);
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(depth_z_min_, depth_z_max_);
      pass_z.setFilterLimitsNegative(false);
      pass_z.filter(*cloud_cut);
    }

    // Re-initialize the "pcl_scan_" ranges to SCAN_MAX
    for(size_t i = 0; i < pcl_scan_.ranges.size(); i++) {
      pcl_scan_.ranges[i] = SCAN_MAX;
    }

    // Convert the valid points into angles and distances, store in pcl_scan_
    // The LaserScan is presumably symmetrical about its midpoint => 
    // index i ~ angle (i - ranges.size()/2)*angle_increment
    for(const auto &pt : cloud_cut->points) {
      // Skip points with x=0 to avoid division by zero
      if(std::fabs(pt.x) < 1e-7) {
        continue;
      }

      float point_angle = std::atan2(pt.y, pt.x);
      // Convert angle -> index in the scan array
      int scan_pt = static_cast<int>( (point_angle / pcl_scan_.angle_increment)
                         + (pcl_scan_.ranges.size() / 2.0f) );

      if(scan_pt < 0 || static_cast<size_t>(scan_pt) >= pcl_scan_.ranges.size()) {
        continue;
      }

      // Distance from origin
      float point_len = std::sqrt(pt.x * pt.x + pt.y * pt.y);
      if(point_len < pcl_scan_.ranges[scan_pt]) {
        pcl_scan_.ranges[scan_pt] = point_len;
      }
    }

    // Mark the time we last updated pcl_scan_
    last_pcl_time_ = this->get_clock()->now();
  }

  // --- Members ---
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Sub/Pub
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_mixed_laser_;

  // Internal data
  sensor_msgs::msg::LaserScan pcl_scan_;    // Holds the point-cloud–derived "scan"
  sensor_msgs::msg::LaserScan mixed_scan_;  // Holds the final merged scan

  std::string pub_scan_topic_name_;
  std::string sub_scan_topic_name_;
  std::string sub_point_topic_name_;
  std::string laser_frame_name_;
  std::string camera_frame_name_;

  double depth_x_min_;
  double depth_x_max_;
  double depth_z_min_;
  double depth_z_max_;
  double point_time_out_;

  bool initialized_flag_;

  rclcpp::Time last_pcl_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCLScanCreater>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
