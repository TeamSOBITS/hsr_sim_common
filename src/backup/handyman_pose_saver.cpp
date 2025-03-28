#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Standard C++ headers
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <cmath>

class PoseKeep : public rclcpp::Node
{
public:
  PoseKeep()
  : Node("handyman_pose_saver"),
    buffer_(this->get_clock()),    // Initialize TF2 buffer with the node’s clock
    tf_listener_(buffer_)
  {
    // Create subscription to /tf (tf2_msgs::msg::TFMessage)
    sub_posi_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf",
      1,
      std::bind(&PoseKeep::callback, this, std::placeholders::_1)
    );

    // Declare/Fetch a parameter if desired (for file path)
    this->declare_parameter<std::string>("save_location_folder_path", ".");
    this->get_parameter("save_location_folder_path", save_location_folder_path_);

    RCLCPP_INFO(this->get_logger(), "handyman_pose_saver node started.");

    // Pre-fill the room candidates
    room_candidate_[0] = "initial_furniture_position";
    room_candidate_[1] = "kitchen";
    room_candidate_[2] = "bedroom";
    room_candidate_[3] = "bedroom_2";
    room_candidate_[4] = "lobby";
    room_candidate_[5] = "lobby_1";
    room_candidate_[6] = "lobby_2";
    room_candidate_[7] = "living_room";
    room_candidate_[8] = "living_room_1";
    room_candidate_[9] = "living_room_2";

    // Pre-fill the furniture candidates
    furniture_candidate_[0] =  "initial_room_position";
    furniture_candidate_[1] =  "armchair";
    furniture_candidate_[2] =  "bed";
    furniture_candidate_[3] =  "cardboard_box";
    furniture_candidate_[4] =  "corner_sofa";
    furniture_candidate_[5] =  "iron_bed";
    furniture_candidate_[6] =  "low_table";
    furniture_candidate_[7] =  "round_low_table";
    // (8 is missing in your code snippet, so be mindful if that’s intentional)
    furniture_candidate_[9] =  "trash_box_for_burnable";
    furniture_candidate_[10] = "trash_box_for_recycle";
    furniture_candidate_[11] = "wagon";
    furniture_candidate_[12] = "white_table";
    furniture_candidate_[13] = "white_rack";
    furniture_candidate_[14] = "wooden_table";
    furniture_candidate_[15] = "wooden_side_table";

    // Example furniture heights
    // (keys must match the strings in furniture_candidate_, watch for any missing index 8, etc.)
    furniture_hight_candidate_["initial_room_position"] = 100;
    furniture_hight_candidate_["armchair"] = 20;
    furniture_hight_candidate_["bed"] = 300;
    furniture_hight_candidate_["cardboard_box"] = 40;
    furniture_hight_candidate_["corner_sofa"] = 500;
    furniture_hight_candidate_["iron_bed"] = 60;
    furniture_hight_candidate_["low_table"] = 7;
    furniture_hight_candidate_["round_low_table"] = 800;
    furniture_hight_candidate_["trash_box_for_burnable"] = 100;
    furniture_hight_candidate_["trash_box_for_recycle"] = 110;
    furniture_hight_candidate_["wagon"] = 120;
    furniture_hight_candidate_["white_table"] = 13;
    furniture_hight_candidate_["white_rack"] = 1400;
    furniture_hight_candidate_["wooden_table"] = 150;
    furniture_hight_candidate_["wooden_side_table"] = 160;

    count_ = 0;
    flag_ = false;

    // Prepare the output file name (with current time)
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    // Format the date/time into a string
    std::ostringstream oss;
    oss << save_location_folder_path_ << "/saved_location_"
        << (tm.tm_mon + 1) << "_" << tm.tm_mday << "_"
        << tm.tm_hour << "_" << tm.tm_min << ".yaml";

    file_name_ = oss.str();

    RCLCPP_INFO(this->get_logger(), "Will save to: %s", file_name_.c_str());
  }

private:
  void callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr /*msg*/)
  {
    // We only use the /tf message as a trigger. Then we do a TF lookup from "map" to "base_footprint".
    geometry_msgs::msg::TransformStamped transform;
    try {
      // In ROS2, no direct waitForTransform. We can do a try/catch with lookupTransform
      transform = buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    // Store the data
    translation_x_.push_back(static_cast<float>(transform.transform.translation.x));
    translation_y_.push_back(static_cast<float>(transform.transform.translation.y));
    translation_z_.push_back(static_cast<float>(transform.transform.translation.z));

    rotation_x_.push_back(static_cast<float>(transform.transform.rotation.x));
    rotation_y_.push_back(static_cast<float>(transform.transform.rotation.y));
    rotation_z_.push_back(static_cast<float>(transform.transform.rotation.z));
    rotation_w_.push_back(static_cast<float>(transform.transform.rotation.w));

    // For demonstration, replicate the original console-based flow:
    while (true) {
      // (1) Ask if we want to add "@" mark
      std::cout << "① 「@」をつけますか？" << std::endl;
      std::cout << "はい -> 「1」を押す/ いいえ -> 「2」を押す" << std::endl;

      int at_num;
      while (true) {
        std::cout << "==▶ ";
        std::cin >> at_num;
        if(at_num == 1) {
          std::cout << "「@」をつけます。\n" << std::endl;
          at_mark_vec_.push_back("@");
          break;
        } else if(at_num == 2) {
          std::cout << "「@」をつけません。\n" << std::endl;
          at_mark_vec_.push_back("");
          break;
        } else {
          std::cout << "押すキーが違います。再度キーを入力してください。\n" << std::endl;
        }
      }

      // (2) Choose the room
      std::cout << "② 部屋の名前番号を選択して下さい。" << std::endl;
      int max_room_num = 0;
      for (auto & kv : room_candidate_) {
        std::cout << "番号 = " << kv.first << ", 部屋の名前 = " << kv.second << std::endl;
        if(kv.first > max_room_num) {
          max_room_num = kv.first;
        }
      }

      int room_num;
      while (true) {
        std::cout << "==▶ ";
        std::cin >> room_num;
        std::cout << "部屋の候補総数(最大番号): " << max_room_num << std::endl;
        if(room_candidate_.count(room_num) == 0) {
          std::cout << "押されたキーの番号は部屋の候補にありません。再度入力してください。\n" << std::endl;
        } else {
          std::cout << "「 " << room_num << " 」 の 「 " << room_candidate_[room_num] << " 」を選択完了。\n" << std::endl;
          room_num_vec_.push_back(room_num);
          break;
        }
      }

      // (3) Choose the furniture
      std::cout << "③ 家具の名前番号を選択して下さい。" << std::endl;
      int max_furniture_num = 0;
      for(auto & kv : furniture_candidate_) {
        std::cout << "番号 = " << kv.first << ", 家具の名前 = " << kv.second << std::endl;
        if(kv.first > max_furniture_num) {
          max_furniture_num = kv.first;
        }
      }

      int furniture_num;
      while (true) {
        std::cout << "==▶ ";
        std::cin >> furniture_num;
        std::cout << "家具の候補総数(最大番号): " << max_furniture_num << std::endl;
        if(furniture_candidate_.count(furniture_num) == 0) {
          std::cout << "押されたキーの番号は家具の候補にありません。再度入力してください。\n" << std::endl;
        } else {
          std::cout << "「 " << furniture_num << " 」 の 「 " << furniture_candidate_[furniture_num] << " 」を選択完了。\n" << std::endl;
          furniture_num_vec_.push_back(furniture_num);
          break;
        }
      }

      // (4) Confirm selection
      std::cout << "④ 選択した内容は以下でよろしいですか?\n";
      std::cout << at_mark_vec_[count_] << room_candidate_[room_num_vec_[count_]] << "#"
                << furniture_candidate_[furniture_num_vec_[count_]] << std::endl;
      std::cout << "はい -> 「1」を押す/ いいえ -> 「2」を押す" << std::endl;

      int check_num;
      while(true) {
        std::cout << "==▶ ";
        std::cin >> check_num;
        if(check_num == 1) {
          std::cout << "選択した内容を保存します。\n" << std::endl;
          count_++;
          flag_ = true;
          break;
        } else if(check_num == 2) {
          std::cout << "内容を再度選択し直してください。\n" << std::endl;
          // remove last selections
          at_mark_vec_.pop_back();
          room_num_vec_.pop_back();
          furniture_num_vec_.pop_back();
          // Do not increment count_
          break;
        } else {
          std::cout << "押すキーが違います。再度キーを入力し直して下さい。\n" << std::endl;
        }
      }

      // If we actually saved, move on. Otherwise loop again
      if(flag_) {
        flag_ = false;
      } else {
        // means we re-selected => go back to (1)
        continue;
      }

      // (5) Decide if we continue or stop
      std::cout << "⑤ 記録を続けるか選択して下さい。\n";
      std::cout << "続ける -> 「1」を押す/ 終了する -> 「2」を押す\n";
      std::cout << "==▶ ";
      int contenue_num;
      std::cin >> contenue_num;
      while(true) {
        if(contenue_num == 1) {
          std::cout << "--記録を続けます--\n" << std::endl;
          break; // break this while => go back to outer while => do next entry
        } else if(contenue_num == 2) {
          std::cout << "\n⑥ 家具名と部屋名と座標をファイルに記録します。" << std::endl;
          {
            std::ofstream ofs(file_name_);
            if(ofs) {
              for(size_t k=0; k<at_mark_vec_.size(); k++) {
                ofs << "\n# " << at_mark_vec_[k]
                    << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << " #\n\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_translation_x: "
                    << translation_x_[k] << "\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_translation_y: "
                    << translation_y_[k] << "\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_translation_z: "
                    << translation_z_[k] << "\n\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_rotation_x: "
                    << rotation_x_[k] << "\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_rotation_y: "
                    << rotation_y_[k] << "\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_rotation_z: "
                    << rotation_z_[k] << "\n";

                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]] << "_rotation_w: "
                    << rotation_w_[k] << "\n\n";

                // Furniture height
                const std::string &furniture_str = furniture_candidate_[furniture_num_vec_[k]];
                float height_val = furniture_hight_candidate_[furniture_str];
                ofs << at_mark_vec_[k] << room_candidate_[room_num_vec_[k]] << "#"
                    << furniture_candidate_[furniture_num_vec_[k]]
                    << "_furniture_hight: "
                    << height_val << "\n";

                ofs << "#======================================#" << std::endl;
              }
              ofs.close();
              std::cout << "「 " << file_name_ << " 」として保存完了。\n\n";
              std::cout << "以上で終了です。ノードを終了してください。\n" << std::endl;
            } else {
              std::cout << "危険!!: ファイルを開けません: " << file_name_ << std::endl;
              std::cout << "パス名を確認してください。" << std::endl;
            }
          }
          // Once done writing, just block or end
          // We'll just break the while loops
          return; // End the callback => stop
        } else {
          std::cout << "押されたキーが違います。再度キーを入力し直して下さい。\n" << std::endl;
        }
        // Wait for next input, but since we do it once outside the while, break
        break;
      } // while (true) for contenue_num
    } // end outer while (true)
  }

  // Node variables
  std::string save_location_folder_path_;
  std::string file_name_;

  // Subscription
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_posi_;

  // TF2
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Internal data
  bool flag_;
  int count_;

  // Data structures from original code
  std::map<int, std::string> furniture_candidate_;
  std::map<int, std::string> room_candidate_;
  std::map<std::string, float> furniture_hight_candidate_;

  std::vector<std::string> at_mark_vec_;
  std::vector<int> room_num_vec_;
  std::vector<int> furniture_num_vec_;

  std::vector<float> translation_x_;
  std::vector<float> translation_y_;
  std::vector<float> translation_z_;

  std::vector<float> rotation_x_;
  std::vector<float> rotation_y_;
  std::vector<float> rotation_z_;
  std::vector<float> rotation_w_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::cout << "\nHandyman 部屋と家具の名前・座標の登録開始\n" << std::endl;

  auto node = std::make_shared<PoseKeep>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
