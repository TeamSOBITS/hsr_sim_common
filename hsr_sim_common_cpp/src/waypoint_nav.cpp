#include <chrono>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>
#include <iostream>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// TF2
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Messages (ROS2)
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Nav2 Action
#include "nav2_msgs/action/navigate_to_pose.hpp"

// Custom (ROS2対応が必要)
#include "hsr_sim_common_interfaces/msg/location_stock.hpp"
#include "hsr_sim_common_interfaces/srv/waypoint_nav.hpp"

using namespace std::chrono_literals;

static const char* FAIL_STR = "FAILURE";
static const char* OK_STR   = "SUCCESS";
static const char* STR_ERR  = "STRING_ERROR";

// 名前付き座標を保存するための構造体
struct NameAndPose
{
  std::string location_name;
  geometry_msgs::msg::Pose location;
};

class WaypointNavNode : public rclcpp::Node
{
public:
  // Nav2のNavigateToPoseアクションをエイリアス化
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit WaypointNavNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_nav", options)
  {
    // パラメータ宣言＆取得
    this->declare_parameter<std::string>("map_frame_name", "map");
    this->get_parameter("map_frame_name", map_frame_name_);

    this->declare_parameter<double>("server_wake_wait_time", 100.0);
    this->get_parameter("server_wake_wait_time", server_wake_wait_time_);

    this->declare_parameter<double>("server_action_wait_time", 300.0);
    this->get_parameter("server_action_wait_time", server_action_wait_time_);

    this->declare_parameter<std::string>("initialpose_topic_name", "/initialpose");
    std::string initialpose_topic;
    this->get_parameter("initialpose_topic_name", initialpose_topic);

    this->declare_parameter<std::string>("pub_arrive_flag_topic_name", "/point_arrive_judg");
    std::string arrive_flag_topic;
    this->get_parameter("pub_arrive_flag_topic_name", arrive_flag_topic);

    this->declare_parameter<std::string>("pub_location_marker_topic_name", "/location_marker");
    std::string location_marker_topic;
    this->get_parameter("pub_location_marker_topic_name", location_marker_topic);

    // ここ以降のsubscribeトピック名も同様に取得
    this->declare_parameter<std::string>("sub_initial_ctrl_position_topic_name", "/initial_ctrl_position");
    std::string sub_init_posi_topic;
    this->get_parameter("sub_initial_ctrl_position_topic_name", sub_init_posi_topic);

    this->declare_parameter<std::string>("sub_initial_ctrl_topic_name", "/initial_ctrl");
    std::string sub_init_name_topic;
    this->get_parameter("sub_initial_ctrl_topic_name", sub_init_name_topic);

    this->declare_parameter<std::string>("sub_move_ctrl_position_topic_name", "/move_ctrl_position");
    std::string sub_move_posi_topic;
    this->get_parameter("sub_move_ctrl_position_topic_name", sub_move_posi_topic);

    this->declare_parameter<std::string>("sub_move_ctrl_topic_name", "/move_ctrl");
    std::string sub_move_name_topic;
    this->get_parameter("sub_move_ctrl_topic_name", sub_move_name_topic);

    this->declare_parameter<std::string>("sub_location_stock_topic_name", "/location_stock");
    std::string sub_location_stock_topic;
    this->get_parameter("sub_location_stock_topic_name", sub_location_stock_topic);

    // サービス名
    this->declare_parameter<std::string>("move_service_name", "move_service");
    std::string move_service_name;
    this->get_parameter("move_service_name", move_service_name);

    this->declare_parameter<std::string>("initial_service_name", "initial_service");
    std::string initial_service_name;
    this->get_parameter("initial_service_name", initial_service_name);

    this->declare_parameter<std::string>("stock_service_name", "stock_service");
    std::string stock_service_name;
    this->get_parameter("stock_service_name", stock_service_name);

    // location.yaml
    this->declare_parameter<std::string>("location_yaml_path", "");
    this->get_parameter("location_yaml_path", location_yaml_path_);

    // TF送信用周期
    this->declare_parameter<double>("tf_pub_cycle_time", 0.1);
    double tf_pub_cycle_time;
    this->get_parameter("tf_pub_cycle_time", tf_pub_cycle_time);

    // --- Publisher 作成 ---
    pub_initial_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic, 1);
    pub_arrive_flag_  = this->create_publisher<std_msgs::msg::Bool>(arrive_flag_topic, 1);
    pub_location_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(location_marker_topic, 1);

    // --- Subscriber 作成 ---
    sub_initial_posi_position_ = this->create_subscription<geometry_msgs::msg::Pose>(
      sub_init_posi_topic, 1,
      std::bind(&WaypointNavNode::initialPosiPositionCB, this, std::placeholders::_1)
    );
    sub_initial_posi_name_ = this->create_subscription<std_msgs::msg::String>(
      sub_init_name_topic, 1,
      std::bind(&WaypointNavNode::initialPosiNameCB, this, std::placeholders::_1)
    );
    sub_target_posi_position_ = this->create_subscription<geometry_msgs::msg::Pose>(
      sub_move_posi_topic, 1,
      std::bind(&WaypointNavNode::movePositionCB, this, std::placeholders::_1)
    );
    sub_target_posi_name_ = this->create_subscription<std_msgs::msg::String>(
      sub_move_name_topic, 1,
      std::bind(&WaypointNavNode::moveNameCB, this, std::placeholders::_1)
    );
    sub_target_posi_position_stock_ = this->create_subscription<hsr_sim_common_interfaces::msg::LocationStock>(
      sub_location_stock_topic, 1,
      std::bind(&WaypointNavNode::locationStockCB, this, std::placeholders::_1)
    );

    // --- Service 作成 ---
    waypoint_move_service_ = this->create_service<hsr_sim_common_interfaces::srv::WaypointNav>(
      move_service_name,
      std::bind(&WaypointNavNode::waypointMoveServer, this, std::placeholders::_1, std::placeholders::_2)
    );
    waypoint_initial_service_ = this->create_service<hsr_sim_common_interfaces::srv::WaypointNav>(
      initial_service_name,
      std::bind(&WaypointNavNode::waypointInitialServer, this, std::placeholders::_1, std::placeholders::_2)
    );
    waypoint_stock_service_ = this->create_service<hsr_sim_common_interfaces::srv::WaypointNav>(
      stock_service_name,
      std::bind(&WaypointNavNode::waypointStockServer, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Nav2 アクションクライアント（"navigate_to_pose"はNav2のデフォルトアクション名）
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // location.yamlの読み込み
    while (true) {
      if (!location_yaml_path_.empty()) {
        if (readLocationYaml(location_yaml_path_)) {
          RCLCPP_INFO(this->get_logger(), "Loaded location YAML successfully. Waiting for topics...");
          break;
        }
      }
      RCLCPP_WARN(this->get_logger(),
                  "No valid yaml. Please set param location_yaml_path: <...>. Retrying in 5 sec...");
      rclcpp::sleep_for(5000ms);
    }

    // 初期poseを少なくとも(0,0,0, 0,0,0,1)で設定してPublish
    initial_pose_.header.frame_id = map_frame_name_;
    initial_pose_.pose.pose.position.x = 0.0;
    initial_pose_.pose.pose.position.y = 0.0;
    initial_pose_.pose.pose.position.z = 0.0;
    initial_pose_.pose.pose.orientation.w = 1.0;

    // 連続で3回程度送っておく（初期化時に確実にセットさせるため）
    pub_initial_pose_->publish(initial_pose_);
    rclcpp::sleep_for(1s);
    pub_initial_pose_->publish(initial_pose_);
    rclcpp::sleep_for(1s);
    pub_initial_pose_->publish(initial_pose_);

    // 初期座標を名前付きで保存（例: "initialpose"）
    stockNamedLocation("initialpose", initial_pose_.pose.pose);

    // TF送信用のタイマー
    tf_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(tf_pub_cycle_time),
      std::bind(&WaypointNavNode::tfPosiBroadcaster, this)
    );
  }

private:
  // ---- Serviceコールバック群 ----
  bool waypointMoveServer(
    const std::shared_ptr<hsr_sim_common_interfaces::srv::WaypointNav::Request> request,
    std::shared_ptr<hsr_sim_common_interfaces::srv::WaypointNav::Response>      response)
  {
    // 要求された location_name が空なら、Poseを使って移動
    if (request->location_name.empty()) {
      // クォータニオンが有効かチェック
      if (!validQuaternion(request->location_pose.orientation)) {
        response->result_text = FAIL_STR;
        RCLCPP_WARN(this->get_logger(), "Invalid quaternion in waypointMoveServer");
        return true;
      }
      geometry_msgs::msg::Pose target_pose = request->location_pose;

      // 新しい座標として保存(名前は自動生成)
      stockUnnamedLocation(target_pose);
      tfPosiBroadcaster();  // TF更新

      // Nav2での移動
      bool ok = moveActionNav2(target_pose);
      if (ok) {
        RCLCPP_INFO(this->get_logger(), "Arrived successfully in waypointMoveServer.");
        response->result_text = OK_STR;
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to reach the goal in waypointMoveServer.");
        response->result_text = FAIL_STR;
      }
      return true;
    } else {
      // 名前がある場合、既存のlocationを探して移動
      geometry_msgs::msg::Pose found_pose;
      bool found = findLocationPose(request->location_name, found_pose);
      if (!found) {
        // リストに無い場合 => 新規登録して移動するロジック(元コード準拠)
        if (!validQuaternion(request->location_pose.orientation)) {
          response->result_text = FAIL_STR;
          RCLCPP_WARN(this->get_logger(), "Invalid quaternion in waypointMoveServer");
          return true;
        }
        stockUnnamedLocation(request->location_pose);
        bool ok = moveActionNav2(request->location_pose);
        if (ok) {
          response->result_text = OK_STR;
          RCLCPP_INFO(this->get_logger(), "Arrived after new stock");
        } else {
          response->result_text = FAIL_STR;
          RCLCPP_WARN(this->get_logger(), "Could not reach after new stock");
        }
      } else {
        // 既存のPoseに移動
        bool ok = moveActionNav2(found_pose);
        if (ok) {
          response->result_text = OK_STR;
          RCLCPP_INFO(this->get_logger(), "Arrived at named location");
        } else {
          response->result_text = FAIL_STR;
          RCLCPP_WARN(this->get_logger(), "Could not reach named location");
        }
      }
      return true;
    }
    return true;
  }

  bool waypointInitialServer(
    const std::shared_ptr<hsr_sim_common_interfaces::srv::WaypointNav::Request> request,
    std::shared_ptr<hsr_sim_common_interfaces::srv::WaypointNav::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "waypointInitialServer called");
    if (request->location_name.empty()) {
      // 名前が無い => そのままPoseを初期値とする
      if (!validQuaternion(request->location_pose.orientation)) {
        response->result_text = FAIL_STR;
        return true;
      }
      initial_pose_.header.frame_id = map_frame_name_;
      initial_pose_.pose.pose = request->location_pose;
      stockUnnamedLocation(initial_pose_.pose.pose);

      pub_initial_pose_->publish(initial_pose_);
      RCLCPP_INFO(this->get_logger(), "Initial pose set directly by pose");
      response->result_text = OK_STR;
      return true;
    } else {
      // 名前あり => リストから検索
      geometry_msgs::msg::Pose found_pose;
      if (!findLocationPose(request->location_name, found_pose)) {
        // リストになし => 登録だけしておいてFAIL返す(元コードと同様の動作)
        if (!validQuaternion(request->location_pose.orientation)) {
          response->result_text = FAIL_STR;
          return true;
        }
        stockUnnamedLocation(request->location_pose);
        RCLCPP_WARN(this->get_logger(), "Name not found. But saved new position. Return fail.");
        response->result_text = FAIL_STR;
        return true;
      }
      // リストにあった場合
      initial_pose_.header.frame_id = map_frame_name_;
      initial_pose_.pose.pose = found_pose;

      pub_initial_pose_->publish(initial_pose_);
      RCLCPP_INFO(this->get_logger(), "Initial pose set by name");
      response->result_text = OK_STR;
      return true;
    }
  }

  bool waypointStockServer(
    const std::shared_ptr<hsr_sim_common_interfaces::srv::WaypointNav::Request> request,
    std::shared_ptr<hsr_sim_common_interfaces::srv::WaypointNav::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "waypointStockServer called");
    if (request->location_name.empty()) {
      RCLCPP_WARN(this->get_logger(), "No location_name in stock request");
      response->result_text = STR_ERR;
      return true;
    }
    // 同じ名前が既にあるかチェック
    for (auto & loc : location_vec_) {
      if (loc.location_name == request->location_name) {
        RCLCPP_WARN(this->get_logger(), "Same location_name already exists");
        response->result_text = STR_ERR;
        return true;
      }
    }
    // クォータニオン有効性チェック
    if (!validQuaternion(request->location_pose.orientation)) {
      response->result_text = FAIL_STR;
      return true;
    }
    // 新規登録
    stockNamedLocation(request->location_name, request->location_pose);
    RCLCPP_INFO(this->get_logger(), "Location %s stocked", request->location_name.c_str());
    response->result_text = OK_STR;
    return true;
  }

  // ---- Subscriberコールバック群 ----
  // 指定されたPoseを初期位置に設定
  void initialPosiPositionCB(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!validQuaternion(msg->orientation)) {
      pubArriveFlag(false);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Set initial pose by direct Pose (%.2f, %.2f)",
                msg->position.x, msg->position.y);
    // 名前なしで保存
    stockUnnamedLocation(*msg);

    initial_pose_.header.frame_id = map_frame_name_;
    initial_pose_.pose.pose = *msg;
    pub_initial_pose_->publish(initial_pose_);
  }

  // 名前で初期位置を設定
  void initialPosiNameCB(const std_msgs::msg::String::SharedPtr msg)
  {
    geometry_msgs::msg::Pose found_pose;
    bool ok = findLocationPose(msg->data, found_pose);
    if (!ok) {
      pubArriveFlag(false);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Set initial pose by name: %s", msg->data.c_str());
    initial_pose_.header.frame_id = map_frame_name_;
    initial_pose_.pose.pose = found_pose;
    pub_initial_pose_->publish(initial_pose_);
  }

  // 目標地点を「座標」で受け取る => Nav2で移動
  void movePositionCB(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!validQuaternion(msg->orientation)) {
      pubArriveFlag(false);
      return;
    }
    stockUnnamedLocation(*msg);
    RCLCPP_INFO(this->get_logger(), "Move to (%.2f, %.2f)", msg->position.x, msg->position.y);

    // スレッドを立てずに、ここで moveActionNav2 を直接呼んでもOK。
    // ただし長時間ブロックするので、非同期にしたいなら別スレッドにする。
    bool ok = moveActionNav2(*msg);
    pubArriveFlag(ok);
  }

  // 目標地点を「名前」で受け取る => Nav2で移動
  void moveNameCB(const std_msgs::msg::String::SharedPtr msg)
  {
    geometry_msgs::msg::Pose found_pose;
    if (!findLocationPose(msg->data, found_pose)) {
      pubArriveFlag(false);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Move to location_name: %s", msg->data.c_str());
    bool ok = moveActionNav2(found_pose);
    pubArriveFlag(ok);
  }

  // 名前＋Poseを保存するだけ
  void locationStockCB(const hsr_sim_common_interfaces::msg::LocationStock::SharedPtr msg)
  {
    if (msg->location_name.empty()) {
      pubArriveFlag(false);
      return;
    }
    // 同名があればNG
    for (auto & loc : location_vec_) {
      if (loc.location_name == msg->location_name) {
        pubArriveFlag(false);
        return;
      }
    }
    // クォータニオンチェック
    if (!validQuaternion(msg->pose.orientation)) {
      pubArriveFlag(false);
      return;
    }
    stockNamedLocation(msg->location_name, msg->pose);
    RCLCPP_INFO(this->get_logger(), "Stored location: %s", msg->location_name.c_str());
  }

  // ---- Nav2アクションを使ってロボットを移動する関数 ----
  bool moveActionNav2(const geometry_msgs::msg::Pose & pose)
  {
    // Nav2のアクションクライアントがまだなら待機
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::duration<double>(server_wake_wait_time_))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available after waiting.");
      return false;
    }
    // Goalを設定
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = map_frame_name_;
    goal_msg.pose.pose = pose;

    // Goal送信
    auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      return false;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return false;
    }

    // 結果待ち
    auto result_future = nav_to_pose_client_->async_get_result(goal_handle);
    auto ret = rclcpp::spin_until_future_complete(
                  this->get_node_base_interface(), 
                  result_future,
                  std::chrono::duration<double>(server_action_wait_time_));
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Navigation did not finish before the timeout");
      // キャンセルしたければ↓など
      nav_to_pose_client_->async_cancel_goal(goal_handle);
      return false;
    }

    // 結果を確認
    auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      return true;
    } else if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(this->get_logger(), "Goal was aborted");
      return false;
    } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Goal was canceled");
      return false;
    }
    RCLCPP_WARN(this->get_logger(), "Unknown result code");
    return false;
  }

  // ---- 補助関数 ----
  // location.yamlを読む。成功true,失敗false
  bool readLocationYaml(const std::string & file_name)
  {
    RCLCPP_INFO(this->get_logger(), "Reading location_yaml: %s", file_name.c_str());
    std::ifstream ifs(file_name);
    if (ifs.fail()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open location_yaml.");
      return false;
    }

    int list_count = 0;
    NameAndPose temp_data;
    std::string line;
    // YAMLをざっくりパース(元コードの簡易版)
    std::string find_str_list[7] = {
      "_translation_x: ", "_translation_y: ", "_translation_z: ",
      "_rotation_x: ", "_rotation_y: ", "_rotation_z: ", "_rotation_w: "
    };
    while (std::getline(ifs, line)) {
      // # name # のような所を探す
      {
        int ws = line.find("# ");
        int we = line.find(" #");
        if (ws > -1 && we > -1) {
          temp_data.location_name = line.substr(ws+2, we-(ws+2));
        }
      }
      // 各要素を探す
      int start = line.find(find_str_list[list_count]);
      if (start > -1) {
        std::stringstream sstr;
        sstr << line.substr(start + find_str_list[list_count].size(), 20);
        switch (list_count) {
          case 0: sstr >> temp_data.location.position.x; break;
          case 1: sstr >> temp_data.location.position.y; break;
          case 2: sstr >> temp_data.location.position.z; break;
          case 3: sstr >> temp_data.location.orientation.x; break;
          case 4: sstr >> temp_data.location.orientation.y; break;
          case 5: sstr >> temp_data.location.orientation.z; break;
          case 6: sstr >> temp_data.location.orientation.w; break;
        }
        list_count++;
      }
      if (list_count == 7) {
        // 1座標分読み終わった
        location_vec_.push_back(temp_data);
        list_count = 0;
      }
    }

    // デバッグ表示したい場合
    // for (auto & l : location_vec_) {
    //   RCLCPP_INFO(this->get_logger(), "loaded %s => (%.2f, %.2f, %.2f)", 
    //               l.location_name.c_str(), 
    //               l.location.position.x, l.location.position.y, l.location.position.z);
    // }
    return true;
  }

  // TF＆Marker送信用
  void tfPosiBroadcaster()
  {
    // MarkerArray作成
    visualization_msgs::msg::MarkerArray marker_array;
    // すべてのLocationに対してTFとMarkerを放送
    for (size_t i = 0; i < location_vec_.size(); i++) {
      const auto & loc = location_vec_[i];

      // TF2 transform
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->now();
      transform_stamped.header.frame_id = map_frame_name_;
      transform_stamped.child_frame_id = loc.location_name;

      transform_stamped.transform.translation.x = loc.location.position.x;
      transform_stamped.transform.translation.y = loc.location.position.y;
      transform_stamped.transform.translation.z = loc.location.position.z;
      transform_stamped.transform.rotation = loc.location.orientation;

      tf_broadcaster_->sendTransform(transform_stamped);

      // ARROW marker
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = map_frame_name_;
      marker.header.stamp = this->now();
      marker.ns = loc.location_name;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.color.a = 1.0f;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.scale.x = 0.3;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.pose = loc.location;
      marker_array.markers.push_back(marker);

      // TEXT marker
      marker.ns = loc.location_name + "_txt";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.text = loc.location_name;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.scale.z = 0.3;
      auto text_pose = loc.location;
      text_pose.position.z += 0.1;
      marker.pose = text_pose;
      marker_array.markers.push_back(marker);
    }
    pub_location_marker_->publish(marker_array);
  }

  // 名前からPoseを探して見つかったらtrue
  bool findLocationPose(const std::string & name, geometry_msgs::msg::Pose & out_pose)
  {
    for (auto & loc : location_vec_) {
      if (loc.location_name == name) {
        out_pose = loc.location;
        return true;
      }
    }
    return false;
  }

  // 成功/失敗をBoolとしてPublish
  void pubArriveFlag(bool flag)
  {
    std_msgs::msg::Bool msg;
    msg.data = flag;
    pub_arrive_flag_->publish(msg);
  }

  // クォータニオンが有効かざっくりチェック
  bool validQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    double sum = std::fabs(q.x) + std::fabs(q.y) + std::fabs(q.z) + std::fabs(q.w);
    return (sum > 0.0);
  }

  // 名前付きでlocation_vec_に登録
  void stockNamedLocation(const std::string & name_str, const geometry_msgs::msg::Pose & pose)
  {
    NameAndPose nap;
    nap.location_name = name_str;
    nap.location = pose;
    location_vec_.push_back(nap);
    RCLCPP_INFO(this->get_logger(), "Stocked location name=%s (size=%zu)", name_str.c_str(), location_vec_.size());
  }

  // 名前なし => 自動生成した名前で登録
  void stockUnnamedLocation(const geometry_msgs::msg::Pose & pose)
  {
    std::stringstream ss;
    ss << "new_position_" << new_position_count_++;
    NameAndPose nap;
    nap.location_name = ss.str();
    nap.location = pose;
    location_vec_.push_back(nap);
    RCLCPP_INFO(this->get_logger(), "Stocked location name=%s (size=%zu)",
                nap.location_name.c_str(), location_vec_.size());
  }

private:
  // -- ROS2通信関連 --
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                          pub_arrive_flag_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr         pub_location_marker_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr  sub_initial_posi_position_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_initial_posi_name_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr  sub_target_posi_position_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_target_posi_name_;
  rclcpp::Subscription<hsr_sim_common_interfaces::msg::LocationStock>::SharedPtr sub_target_posi_position_stock_;

  rclcpp::Service<hsr_sim_common_interfaces::srv::WaypointNav>::SharedPtr waypoint_move_service_;
  rclcpp::Service<hsr_sim_common_interfaces::srv::WaypointNav>::SharedPtr waypoint_initial_service_;
  rclcpp::Service<hsr_sim_common_interfaces::srv::WaypointNav>::SharedPtr waypoint_stock_service_;

  // -- Nav2アクション --
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  // -- TFブロードキャスト --
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{
    std::make_unique<tf2_ros::TransformBroadcaster>(this)
  };

  // -- タイマー --
  rclcpp::TimerBase::SharedPtr tf_timer_;

  // -- パラメータ類 --
  std::string map_frame_name_;
  double server_wake_wait_time_;
  double server_action_wait_time_;
  std::string location_yaml_path_;

  // -- 内部状態 --
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
  int new_position_count_{0};
  std::vector<NameAndPose> location_vec_;
};

// ---- メイン関数 ----
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointNavNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
