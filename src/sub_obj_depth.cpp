#include <memory>
#include <string>
#include <iostream>
#include <cmath>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Messages
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

// PCL
// #include <pcl_ros/point_cloud.h>        // ROS2対応版のpcl_rosが必要
// #include <pcl_ros/transforms.hpp>       // 同上
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

// サービス (ROS2でのカスタムサービス)
#include "hsr_sim_common_interfaces/srv/obj_depth.hpp"

class ObjectDepthNode : public rclcpp::Node
{
public:
  explicit ObjectDepthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("object_depth", options)
  {
    // パラメータやトピック名などを適宜設定
    // ここでは固定文字列を直接利用
    topic_name_   = "/hsrb/head_rgbd_sensor/depth/points";
    frame_target_ = "hand_motor_dummy_link";       // TF先(変換先)
    frame_source_ = "head_rgbd_sensor_rgb_frame";  // TF元(変換元)

    // TF2 Buffer & Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher (PointCloud2)
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/grasped_object_depth", 1);

    // Subscriber (PointCloud2)
    // QoSはSensorDataQoS()など適切なものを選択
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name_,
      rclcpp::SensorDataQoS(),  // 必要に応じて
      std::bind(&ObjectDepthNode::depthCallback, this, std::placeholders::_1)
    );

    // Service
    depth_service_ = this->create_service<hsr_sim_common::srv::ObjDepth>(
      "get_object_depth",
      std::bind(&ObjectDepthNode::handleDepthService, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "ObjectDepthNode initialized.");
  }

private:
  // depthのPointCloud2を受け取るコールバック
  void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // TF変換を試みる
    try {
      // 30秒待つ (ROS2のtf2でlookupTransform)
      auto transform_stamped = tf_buffer_->lookupTransform(
        frame_target_,   // 変換先
        frame_source_,   // 変換元
        tf2::TimePointZero,
        rclcpp::Duration::from_seconds(30.0)
      );

      // 1) ROSのPointCloud2をPCLへ変換
      pcl::fromROSMsg(*msg, pcl_cloud_);
      // 2) Min/Maxをとる
      pcl::getMinMax3D(pcl_cloud_, min_pt_, max_pt_);

      // 3) PassThrough
      auto cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud_);
      pass_filter_.setInputCloud(cloud_ptr);
      pass_filter_.setFilterFieldName("z");
      pass_filter_.setFilterLimits(0.0, min_pt_.z + 0.5);
      pass_filter_.filter(pcl_cloud_); // フィルタ結果を同じpcl_cloud_に上書き

      // 4) tf2::doTransformで、frame_target_座標系へPointCloud2を変換
      //    - pcl_ros::transformPointCloudを使う手もあるが、
      //      ROS2対応状況によってはtf2::doTransformの方が確実
      pcl::toROSMsg(pcl_cloud_, cloud_msg_);  // PCL -> ROS Msg
      sensor_msgs::msg::PointCloud2 transformed_msg;
      tf2::doTransform(cloud_msg_, transformed_msg, transform_stamped);

      // 5) 再びpclへ変換し、Min/Max計算
      pcl::fromROSMsg(transformed_msg, pcl_cloud_);
      pcl::getMinMax3D(pcl_cloud_, min_pt_, max_pt_);

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF transform error: %s", ex.what());
      return;  // 変換失敗時は何もせず終了
    }

    // デバッグ用にフィルタ後の点群をパブリッシュするかどうか
    if (pub_filtered_flag_) {
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(pcl_cloud_, out_msg);
      out_msg.header.stamp = now();
      out_msg.header.frame_id = frame_target_;  // 変換先フレーム
      pub_->publish(out_msg);
    }
  }

  // サービスコール
  bool handleDepthService(
    const std::shared_ptr<hsr_sim_common::srv::ObjDepth::Request>  request,
    std::shared_ptr<hsr_sim_common::srv::ObjDepth::Response>       response)
  {
    RCLCPP_INFO(this->get_logger(), "GetObjectDepth service called.");
    // サービスで来たフラグに応じて、パブリッシュのオン/オフを切り替え
    pub_filtered_flag_ = request->request;

    // ここでは min_pt_, max_pt_ を元に結果を設定 (元コードとほぼ同じ計算)
    response->x = std::fabs(min_pt_.x);
    response->y = std::fabs(max_pt_.y) + std::fabs(min_pt_.y);
    response->z = std::fabs(max_pt_.z) + std::fabs(min_pt_.z);

    RCLCPP_INFO(this->get_logger(), "Computed obj depth => x=%.3f y=%.3f z=%.3f",
                response->x, response->y, response->z);
    return true;
  }

  // --- メンバ変数 ---
  // Subscriber/Publisher/Service
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
  rclcpp::Service<hsr_sim_common::srv::ObjDepth>::SharedPtr      depth_service_;

  // TF関連
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // パラメータ/設定
  std::string topic_name_;
  std::string frame_target_;  // hand_motor_dummy_link
  std::string frame_source_;  // head_rgbd_sensor_rgb_frame

  // PCL
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_;
  sensor_msgs::msg::PointCloud2  cloud_msg_;
  pcl::PassThrough<pcl::PointXYZ> pass_filter_;

  // Min/Max
  pcl::PointXYZ min_pt_, max_pt_;

  // フィルタリング結果をpublishするかどうか
  bool pub_filtered_flag_{false};
};

// main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
