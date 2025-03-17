#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Standard C++ includes
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <cstdlib>

class PoseSaver : public rclcpp::Node
{
public:
  PoseSaver()
  : Node("pose_saver"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // 1) Retrieve parameter for "save_location_folder_path" (like your ROS1 version)
    this->declare_parameter<std::string>("save_location_folder_path", "");
    this->get_parameter("save_location_folder_path", save_location_folder_path_);
    if (save_location_folder_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'save_location_folder_path' is empty.");
      // Sleep a bit and then quit.
      rclcpp::sleep_for(std::chrono::seconds(2));
      std::exit(EXIT_FAILURE);
    }

    // 2) Build file name using current date/time
    std::time_t now = std::time(nullptr);
    std::tm* pnow = std::localtime(&now);
    std::ostringstream oss;
    oss << save_location_folder_path_
        << "/map_location_"
        << (pnow->tm_mon + 1) << "_"
        << pnow->tm_mday << "_"
        << pnow->tm_hour << "_"
        << pnow->tm_min << ".yaml";
    file_name_ = oss.str();

    RCLCPP_INFO(this->get_logger(),
                "\n場所名を入力すると位置座標を保存します。\nPress 'q' to quit.\nSaving to file: %s",
                file_name_.c_str());

    first_flag_ = true;
    // 3) Block forever, prompting user for input
    while (true) {
      if (first_flag_) {
        std::cout << "\n場所名を入力してください (\"q\"で終了): ";
      } else {
        std::cout << "\n前回(" << point_name_ << ")と異なる場所名を入力してください (\"q\"で終了): ";
      }

      // Grab a line from stdin
      if (!std::getline(std::cin, point_name_)) {
        // If input failed or EOF, exit
        std::cerr << "Failed to read input or EOF encountered. Exiting...\n";
        std::exit(EXIT_FAILURE);
      }

      // Check for 'q'
      if (point_name_ == "q") {
        std::cout << "OK, I'll end...." << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(2));
        std::exit(EXIT_SUCCESS);
      }

      std::cout << "point_name: " << point_name_ << std::endl;
      // 4) Attempt to get pose from TF
      getPose();

      first_flag_ = false;
    }
  }

private:
  void getPose()
  {
    // We attempt a transform from "/map" to "/base_footprint"
    geometry_msgs::msg::TransformStamped transform;
    try {
      // There's no direct "waitForTransform" in ROS2. We do a synchronous lookup with a short timeout
      // or do repeated tries in a loop. For simplicity, try once:
      transform = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      std::cerr << "位置取得失敗: " << ex.what() << std::endl;
      std::cerr << "tfは出ていますか？ /map と /base_footprint は繋がっていますか？\n";
      // Sleep and return
      rclcpp::sleep_for(std::chrono::seconds(2));
      return;
    }

    // Print to console
    std::cout << "\ntransform.getOrigin().x(): " << transform.transform.translation.x << std::endl;
    std::cout << "transform.getOrigin().y(): " << transform.transform.translation.y << std::endl;
    std::cout << "transform.getOrigin().z(): " << transform.transform.translation.z << std::endl;
    std::cout << "transform.getRotation().x(): " << transform.transform.rotation.x << std::endl;
    std::cout << "transform.getRotation().y(): " << transform.transform.rotation.y << std::endl;
    std::cout << "transform.getRotation().z(): " << transform.transform.rotation.z << std::endl;
    std::cout << "transform.getRotation().w(): " << transform.transform.rotation.w << std::endl;

    // 5) Append to the YAML file
    std::ofstream ofs(file_name_, std::ios::app);
    if (!ofs.is_open()) {
      std::cerr << file_name_ << " は作成できませんでした。パスを確認してください。\n";
      return;
    }

    ofs << "\n# " << point_name_ << " #\n\n"
        << "/" << point_name_ << "_translation_x: " << transform.transform.translation.x << "\n"
        << "/" << point_name_ << "_translation_y: " << transform.transform.translation.y << "\n"
        << "/" << point_name_ << "_translation_z: " << transform.transform.translation.z << "\n\n"
        << "/" << point_name_ << "_rotation_x: " << transform.transform.rotation.x << "\n"
        << "/" << point_name_ << "_rotation_y: " << transform.transform.rotation.y << "\n"
        << "/" << point_name_ << "_rotation_z: " << transform.transform.rotation.z << "\n"
        << "/" << point_name_ << "_rotation_w: " << transform.transform.rotation.w << "\n"
        << "#======================================#\n";
    ofs.close();

    if (first_flag_) {
      std::cout << "「 " << file_name_ << " 」として保存完了。\n";
    } else {
      std::cout << "「 " << file_name_ << " 」に追記完了。\n";
    }
  }

  // Node members
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool first_flag_;
  std::string point_name_;
  std::string file_name_;
  std::string save_location_folder_path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Because the constructor blocks, we never actually get to spin.
  // This is just to replicate your original logic. Typically you'd separate user input from node spinning.
  auto node = std::make_shared<PoseSaver>();
  // In a typical ROS2 node, we'd call rclcpp::spin(node) here,
  // but the code in the constructor never returns.

  rclcpp::shutdown();
  return 0;
}
