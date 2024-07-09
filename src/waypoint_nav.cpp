#include <stdio.h>
#include <iostream>
#include <fstream>//for yaml
#include <string>
#include <sstream>
#include <vector>
#include <time.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <boost/thread/thread.hpp>

#include <hsr_sim_common/location_stock.h>
#include <hsr_sim_common/waypoint_nav.h>

#include <typeinfo>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;


struct name_and_posi_set {//tfに座標を渡すための変数
    std::string location_name;
    geometry_msgs::Pose location;
};



using namespace std;
#define nsec2sec 1e-9

#define fail_str "FAILURE"
#define ok_str   "SUCCESS"
#define str_err  "STRING_ERROR"




class waypoint_nav_class{

public:

    waypoint_nav_class()//クラスの初期設定作業
    {
    //クラス変数の初期化
    new_position_count = 0;

    this->map_frame_name = "map";
    ros::param::get( "map_frame_name", map_frame_name );

    server_wake_wait_time = 100.0;//[sec]
    ros::param::get( "server_wake_wait_time", server_wake_wait_time );

    server_action_wait_time = 300.0;//[sec]
    ros::param::get( "server_action_wait_time", server_action_wait_time );


    // Initial pose
    std::string initialpose_topic_name = "/initialpose";
    ros::param::get( "initialpose_topic_name", initialpose_topic_name );
    initial_pose.header.frame_id = map_frame_name;
    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;
    pub_initial_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( initialpose_topic_name, 1 );
    pub_initial_pose.publish(initial_pose);//初期位置座標の送信
  	ros::Duration(1.0).sleep();
  	pub_initial_pose.publish(initial_pose);
  	ros::Duration(1.0).sleep();
  	pub_initial_pose.publish(initial_pose);

    waypoint_nav_class::stock_named_new_location( "initialpose", initial_pose.pose.pose );




    //PUB
    std::string pub_arrive_flag_topic_name             = "/point_arrive_judg";
    std::string pub_location_marker_topic_name         = "/location_marker";
    ros::param::get( "pub_arrive_flag_topic_name",     pub_arrive_flag_topic_name );
    ros::param::get( "pub_location_marker_topic_name", pub_location_marker_topic_name );

    pub_arrive_flag     = nh.advertise<std_msgs::Bool>(                  pub_arrive_flag_topic_name, 1 );
    pub_location_marker = nh.advertise<visualization_msgs::MarkerArray>( pub_location_marker_topic_name, 1 );


    //SUB
    std::string sub_initial_ctrl_position_topic_name  = "/initial_ctrl_position";
    std::string sub_initial_ctrl_topic_name           = "/initial_ctrl";
    std::string sub_move_ctrl_position_topic_name     = "/move_ctrl_position";
    std::string sub_move_ctrl_topic_name              = "/move_ctrl";
    std::string sub_location_stock_topic_name         = "/location_stock";



    ros::param::get( "sub_initial_ctrl_position_topic_name", sub_initial_ctrl_position_topic_name );
    ros::param::get( "sub_initial_ctrl_topic_name",          sub_initial_ctrl_topic_name );
    ros::param::get( "sub_move_ctrl_position_topic_name",    sub_move_ctrl_position_topic_name );
    ros::param::get( "sub_move_ctrl_topic_name",             sub_move_ctrl_topic_name );
    ros::param::get( "sub_location_stock_topic_name",        sub_location_stock_topic_name  );


    sub_initial_posi_position      = nh.subscribe<geometry_msgs::Pose>(          sub_initial_ctrl_position_topic_name, 1, &waypoint_nav_class::initial_posi_position_CB, this );//直接、初期位置の座標をもらう場合
    sub_initial_posi_name          = nh.subscribe<std_msgs::String>(             sub_initial_ctrl_topic_name,          1, &waypoint_nav_class::initial_posi_name_CB, this );//初期位置の座標の名前をもらう場合
    sub_target_posi_position       = nh.subscribe<geometry_msgs::Pose>(          sub_move_ctrl_position_topic_name,    1, &waypoint_nav_class::move_position_CB, this );//直接、目標地点の座標をもらう場合
    sub_target_posi_name           = nh.subscribe<std_msgs::String>(             sub_move_ctrl_topic_name,             1, &waypoint_nav_class::move_name_CB, this );//目標地点の名前をもらう場合
    sub_target_posi_position_stock = nh.subscribe<hsr_sim_common::location_stock>( sub_location_stock_topic_name,        1, &waypoint_nav_class::location_stock_CB, this );//地点の名前と座標をもらう。移動はしない


    //サービス
    std::string move_service_name    = "move_service";
    std::string initial_service_name = "initial_service";
    std::string stock_service_name   = "stock_service";
    ros::param::get( "move_service_name",    move_service_name  );
    ros::param::get( "initial_service_name", initial_service_name  );
    ros::param::get( "stock_service_name",   stock_service_name  );
    waypoint_move_service    = nh.advertiseService(move_service_name,    &waypoint_nav_class::waypoint_move_server, this);
    waypoint_initial_service = nh.advertiseService(initial_service_name, &waypoint_nav_class::waypoint_initial_server, this);
    waypoint_stock_service   = nh.advertiseService(stock_service_name,   &waypoint_nav_class::waypoint_stock_server, this);


    //location_yamlの読み込み
    std::string location_yaml_path;
    while (true)
    {
      if( ros::param::get( "location_yaml_path", location_yaml_path ) )
      {
        if( waypoint_nav_class::read_location_yaml(location_yaml_path) )
        {
          ROS_INFO_STREAM( "load_location yaml OK. and waiting topic" );
          break;//読み込みOK
        }//if
      }//if
      ROS_WARN_STREAM( "do rosparam set " << "/waypoint_nav/" << "location_yaml_path  ****.yaml . " );
      ros::Duration(5).sleep();
    }//while

    double tf_pub_cycle_time = 0.1;//[sec]
    ros::param::get( "tf_pub_cycle_time", tf_pub_cycle_time );
    timer = nh.createTimer(ros::Duration( tf_pub_cycle_time ), &waypoint_nav_class::tf_posi_broadcaster_time_CB, this );




  }//初期化＿終了
  ~waypoint_nav_class(){}


// #####################  移動の際のサービス

  bool waypoint_move_server( hsr_sim_common::waypoint_nav::Request  &input, hsr_sim_common::waypoint_nav::Response &output )
  {
    //ROS_INFO_STREAM( "\n\n### waypoint_move_server" );
    if(input.location_name == "")//名前が無い　→　座標をそのまま使って移動
    {
      //クォータニオンを調べる
      if( !waypoint_nav_class::valid_quaternion_judge( input.location_pose.orientation ) )
      {
        output.result_text = fail_str;
        ROS_WARN_STREAM( "\n" << output.result_text );
        return true;
      }//if
      //ROS_INFO_STREAM( "\nmove to (x,y)= (\t" << input.location_pose.position.x << ",\t" << input.location_pose.position.y << ")" );
      //move_base_msgs::MoveBaseGoal goal;
      //goal.target_pose.header.frame_id = map_frame_name;
      //goal.target_pose.pose = input.location_pose;
      geometry_msgs::Pose pose;//ハコをつくっておく
      pose = input.location_pose;
      waypoint_nav_class::stock_unnamed_new_location( pose );
      waypoint_nav_class::tf_posi_broadcaster();
      if( waypoint_nav_class::move_actionlib( pose ) )//移動
      {
       ROS_INFO_STREAM( "arrival OK" );
       output.result_text = ok_str ;
       ROS_INFO_STREAM(output.result_text );
       return true;
      }//if

      ROS_WARN_STREAM( "Could not reach there because there are obstacles" );
      output.result_text = fail_str;
      ROS_WARN_STREAM(output.result_text );
      return true;
    }//if

    else//名前がある　→　クラスのリストから該当する名前を探して、その座標を使って移動
    {
      geometry_msgs::Pose pose;//ハコをつくっておく
      if( !waypoint_nav_class::find_location_pose(input.location_name, pose) )//リストに名前があるか検索する　→　ない場合　→　新たに登録して移動
      {
        //クォータニオンを調べる
        if( !waypoint_nav_class::valid_quaternion_judge( input.location_pose.orientation ) )
        {
          output.result_text = fail_str;
          ROS_WARN_STREAM( "\n" << output.result_text );
          return true;
        }//if

        pose = input.location_pose;
        waypoint_nav_class::stock_unnamed_new_location( pose );



        if( waypoint_nav_class::move_actionlib( pose ) )//移動
        {
          ROS_INFO_STREAM( "\narrival OK" );
          output.result_text = ok_str ;
          ROS_INFO_STREAM( "\n" << output.result_text );
          return true;
        }//if
        else
        {
          ROS_WARN_STREAM( "\nCould not reach there because there are obstacles" );
          output.result_text = fail_str ;
          ROS_WARN_STREAM( "\n" << output.result_text );
          return true;
        }//else
      }//if
      else//リストを検索して名前があった場合　→　移動
      {
        ROS_INFO_STREAM( "\nmove to " << input.location_name );
        //boost::thread move_actionlib_thread( &waypoint_nav_class::move_actionlib, this, goal );
        if( waypoint_nav_class::move_actionlib( pose ) )//移動
        {
          output.result_text = ok_str ;
          ROS_INFO_STREAM( "\n" << output.result_text );
          return true;
        }//if
        else
        {
          ROS_WARN_STREAM( "\nCould not reach there because there are obstacles" );
          output.result_text = fail_str ;
          ROS_WARN_STREAM( "\n" << output.result_text );
          return true;
        }//else
      }//else
    }//else

    return true;
  }//waypoint_move_server



// #####################  初期位置の設定の際のサービス

  bool waypoint_initial_server( hsr_sim_common::waypoint_nav::Request  &input, hsr_sim_common::waypoint_nav::Response &output )
  {
    ROS_INFO_STREAM( "\n\n### waypoint_initial_server" );
    if( input.location_name == "" )//名前が無い　→　座標をそのまま送信
    {
      //クォータニオンを調べる
      if( !waypoint_nav_class::valid_quaternion_judge( input.location_pose.orientation ) )
      {
        output.result_text = fail_str ;
        ROS_WARN_STREAM( "\n" << output.result_text );
        return true;
      }//if

      //ROS_INFO_STREAM( "\nset initial pose : (x,y) = (\t" << input.location_pose.position.x << ",\t" << input.location_pose.position.y << ")" );
      initial_pose.header.frame_id = map_frame_name;
      initial_pose.pose.pose = input.location_pose;
      waypoint_nav_class::stock_unnamed_new_location( initial_pose.pose.pose );
      waypoint_nav_class::tf_posi_broadcaster();
      pub_initial_pose .publish(initial_pose);//初期位置の座標の送信
      ROS_INFO_STREAM( "\ninitial_pose set OK" );
      output.result_text = ok_str ;
      //ROS_INFO_STREAM( "\n" << output.result_text );
      return true;
    }//if
    else//名前がある　→　クラスのリストから該当する名前を探して、その座標を送信
    {
      initial_pose.header.frame_id = map_frame_name;
      if( !waypoint_nav_class::find_location_pose( input.location_name, initial_pose.pose.pose) )//リストを検索　→　リストにない　→　登録？
      {
        //クォータニオンを調べる
        if( !waypoint_nav_class::valid_quaternion_judge( input.location_pose.orientation ) )
        {
          output.result_text = fail_str ;
          ROS_WARN_STREAM( "\n" << output.result_text );
          return true;
        }//if


        waypoint_nav_class::stock_unnamed_new_location( input.location_pose );//登録


        ROS_WARN_STREAM( "\nNO location_name" );
        ROS_WARN_STREAM( "\nNO location_name . But maybe I can save the position. " );
        output.result_text = fail_str ;
        ROS_WARN_STREAM( "\n" << output.result_text );
        return true;
      }//if
      ROS_INFO_STREAM( "\nset initial pose : " <<  input.location_name );
      ROS_INFO_STREAM( "\ninitial.pose.pose : " << initial_pose.pose.pose );
      pub_initial_pose .publish(initial_pose);//初期位置の座標の送信

      ROS_INFO_STREAM( "\ninitial_pose set OK" );
      output.result_text = ok_str ;
      ROS_INFO_STREAM( "\n" << output.result_text );
      return true;
    }//else

    return true;
  }//waypoint_initial_server


// #####################  地点登録の際のサービス

  bool waypoint_stock_server( hsr_sim_common::waypoint_nav::Request  &input, hsr_sim_common::waypoint_nav::Response &output )
  {
    ROS_INFO_STREAM( "\n\n### waypoint_stock_server" );
    if( input.location_name == "" )
    {
      ROS_WARN_STREAM( "\nNO location_name " );
      output.result_text = str_err;
      ROS_WARN_STREAM( "\n" << output.result_text );
      return true;
    }//if
    else
    {
      bool already_same_location_name_exist_flag = false;
      for( int i = 0 ; i < location_vec.size() ; i++ )//既に同じ名前があると困るので、かぶっている名前が無いか確認する。
      {
        if( location_vec[i].location_name == input.location_name )
        {
          already_same_location_name_exist_flag = true;
          break;//for
        }//if
      }//for
      if( already_same_location_name_exist_flag == true )
      {
        ROS_WARN_STREAM( "\nalready same location_name is exist" );
        output.result_text = str_err ;
        ROS_WARN_STREAM( "\n" << output.result_text );
        return true;
      }//if

      //クォータニオンを調べる
      if( !waypoint_nav_class::valid_quaternion_judge( input.location_pose.orientation ) )
      {
        output.result_text = fail_str ;
        ROS_WARN_STREAM( "\n" << output.result_text );
        return true;
      }//if

      //ROS_INFO_STREAM( "\nsave position : \nlocation_name=" << input.location_name << "\n(x,y)= ( " <<  input.location_pose.position.x << ", " << input.location_pose.position.y << ")" );
      waypoint_nav_class::stock_named_new_location( input.location_name, input.location_pose );
      output.result_text = ok_str ;
      ROS_INFO_STREAM( "\n" << output.result_text );
      return true;
    }//else
  }//waypoint_stock_server




// #####################  サービスやコールバックに使う関数群

  // yaml読み込み
  bool read_location_yaml( std::string file_name )//location_yamlの読み込み
  {
    ROS_INFO_STREAM( "\nread_location_yaml = " << file_name );
    std::ifstream ifs( file_name.c_str() );
    std::string str;
    if(ifs.fail())
    {
      ROS_INFO_STREAM( "\nread_location_yaml : read error" );
      return false;
    }//if

    int list_num_count = 0;
    //読み込む要素の設定
    std::string find_str_list[7] =
    { "_translation_x: ",
      "_translation_y: ",
      "_translation_z: ",
      "_rotation_x: ",
      "_rotation_y: ",
      "_rotation_z: ",
      "_rotation_w: "  };
    name_and_posi_set temp_name_and_posi;

    //読み込んだ内容の整理
    while (getline(ifs, str))
    {
      int word_start_num = str.find("# ");
      int word_end_num = str.find(" #");
      if( word_start_num > -1 && word_end_num > -1 )//場所の名前の検索
      {
        std::string temp_str;
        temp_str = str.substr( word_start_num+2, word_end_num-2 );
        temp_name_and_posi.location_name = temp_str;
      }//if

      word_start_num = str.find( find_str_list[list_num_count] );
      word_end_num = 20;//とりあえず20桁あれば大丈夫でしょ
      if(word_start_num > -1 )//具体的な座標の値の検索
      {
        std::stringstream temp_str ;
        temp_str  << str.substr( word_start_num + find_str_list[list_num_count].size(), word_end_num );
        switch ( list_num_count ) {
          case 0:
            temp_str >> temp_name_and_posi.location.position.x ;
          case 1:
            temp_str >> temp_name_and_posi.location.position.y ;
          case 2:
            temp_str >> temp_name_and_posi.location.position.z ;
          case 3:
            temp_str >> temp_name_and_posi.location.orientation.x ;
          case 4:
            temp_str >> temp_name_and_posi.location.orientation.y ;
          case 5:
            temp_str >> temp_name_and_posi.location.orientation.z ;
          case 6:
            temp_str >> temp_name_and_posi.location.orientation.w ;
        }//switch
        list_num_count++;
      }//if
      if( list_num_count == 7 )//座標の格納　＆　次の座標の読み込みに向けて初期化
      {
        location_vec.push_back( temp_name_and_posi );
        list_num_count = 0;
      }//if
    }//while

    //読み込んだ内容の表示
    bool show_location_list = false;
    if( show_location_list )
    {
      for ( int i = 0; i < location_vec.size() ; i ++ )
      {
        ROS_INFO_STREAM( "\n" << location_vec[i].location_name << " : \n" << location_vec[i].location );
      }//for
    }
    return true;
  }//read_location_yaml


  //tfの送信用のコールバック
  void tf_posi_broadcaster_time_CB( const ros::TimerEvent& event )//const ros::TimerEvent& event
  {
    waypoint_nav_class::tf_posi_broadcaster();
    return;
  }


  //ｔｆの送信
  void tf_posi_broadcaster()
  {
    visualization_msgs::MarkerArray marker_array;
    for ( int i = 0 ; i < location_vec.size() ; i++ )
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(location_vec[i].location.position.x, location_vec[i].location.position.y, location_vec[i].location.position.z) );
      tf::Quaternion q( location_vec[i].location.orientation.x, location_vec[i].location.orientation.y, location_vec[i].location.orientation.z, location_vec[i].location.orientation.w );
      transform.setRotation( q );
      br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), map_frame_name, location_vec[i].location_name ) );

      visualization_msgs::Marker marker;
      marker.header.frame_id = map_frame_name;
      marker.header.stamp = ros::Time::now();
      marker.ns = location_vec[i].location_name;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = i;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.scale.x = 0.3;//[m]
      marker.scale.y = 0.1;//[m]
      marker.scale.z = 0.1;//[m]
      marker.pose = location_vec[i].location;
      marker_array.markers.push_back( marker );

      marker.header.frame_id = map_frame_name;
      marker.header.stamp = ros::Time::now();
      marker.ns = location_vec[i].location_name + "_txt";
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = i;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.text = location_vec[i].location_name;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.scale.z = 0.3;//[m]
      marker.pose = location_vec[i].location;
      marker.pose.position.z += 0.1;
      marker_array.markers.push_back( marker );
    }//for
    pub_location_marker.publish( marker_array );
    return ;
  }//void tf_posi_broadcaster()


  //リストから該当する名前を探しだし、座標を返す 無いならfalseを返す
  bool find_location_pose( std::string location_name, geometry_msgs::Pose &pose_ptr )
  {
    bool found_target_location_name_flag = false;
    for ( int i = 0 ; i < location_vec.size() ; i++ )
    {
      if( location_vec[i].location_name == location_name )//名前が一致する座標があったのでそれを格納する。
      {
        pose_ptr = location_vec[i].location;
        ROS_INFO_STREAM( "\n pose = " << location_vec[i].location );
        found_target_location_name_flag = true;
        return true;
      }//if
    }//for
    if( found_target_location_name_flag == false )//名前が一致するもの何もなかったので断念
    {
      ROS_INFO_STREAM( "\ntarget_location_name : " << location_name << " is not exist." );
      return false;
    }//if
    return true;
  }//find_location_pose


  //トピックから指示をもらった場合に、この関数を使ってその結果を返す
  void pub_arrive_flag_func( bool true_or_false )
  {
    std_msgs::Bool arrive_flag;
    arrive_flag.data = true_or_false;
    pub_arrive_flag.publish(arrive_flag);
    return;
  }//pub_arrive_flag_func


  //有効なクォータニオンかどうかを判断する
  bool valid_quaternion_judge( geometry_msgs::Quaternion quat )
  {
    double temp_total_q = fabs( quat.x ) + fabs( quat.y ) + fabs( quat.z ) + fabs( quat.w );
    if( temp_total_q == 0 )
    {
      ROS_WARN_STREAM( "\nVALID QUATERNION ? : " << "NO "  << temp_total_q );
      return false;
    }//if
    ROS_INFO_STREAM( "\nVALID QUATERNION ? : " << "ok "  << temp_total_q );
    return true;
  }//valid_quaternion_judge


  //リストに名前と座標を格納する関数
  void stock_named_new_location( std::string name_str, geometry_msgs::Pose pose )
  {
    //tfで送信するための座標登録
    name_and_posi_set temp_name_and_posi;
    temp_name_and_posi.location_name = name_str;
    temp_name_and_posi.location = pose;
    ROS_INFO_STREAM( "\nsaved : " << temp_name_and_posi.location_name << "\n" << temp_name_and_posi.location );
    location_vec.push_back(temp_name_and_posi);
    ROS_INFO_STREAM( "\nlocation_vec.size() : " << location_vec.size() );
    return;
  }//stock_named_new_location


  //リストにテキトーな名前をつけて座標を保存する
  void stock_unnamed_new_location( geometry_msgs::Pose pose )
  {
    std::stringstream temp_str;
    temp_str << "new_position_" << new_position_count;
    new_position_count++;
    //tfで送信するための座標登録
    name_and_posi_set temp_name_and_posi;
    temp_name_and_posi.location_name = temp_str.str();
    temp_name_and_posi.location = pose;
    ROS_INFO_STREAM( "\nsaved : " << temp_name_and_posi.location_name << "\n" << temp_name_and_posi.location );
    location_vec.push_back(temp_name_and_posi);
    ROS_INFO_STREAM( "\nlocation_vec.size() : " << location_vec.size() );
    return;
  }//stock_unnamed_new_location





// #####################  座標を保存するコールバック(トピックから)

  void location_stock_CB( const hsr_sim_common::location_stock::ConstPtr& msg )//目標地点の座標を受け取りクラスの内に保存する。移動はしない。
  {
    if( msg->location_name == "" )
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }
    bool already_same_location_name_exist_flag = false;
    for( int i = 0 ; i < location_vec.size() ; i++ )//既に同じ名前があると困るので、かぶっている名前が無いか確認する。
    {
      if( location_vec[i].location_name == msg->location_name )
      {
        already_same_location_name_exist_flag = true;
        break;
      }//if
    }//for
    if( already_same_location_name_exist_flag == true )
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }

    //クォータニオンを調べる
    if( !waypoint_nav_class::valid_quaternion_judge( msg->pose.orientation ) )
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }//if

    waypoint_nav_class::stock_named_new_location( msg->location_name, msg->pose );

    ROS_INFO_STREAM( "\nlocation_vec.size() : " << location_vec.size() );
    return;
  }//location_stock_CB




// #####################  初期位置を設定するコールバック(トピックから)　名前をもらう
  void initial_posi_name_CB( const std_msgs::String msg )//初期位置の設定
  {

    initial_pose.header.frame_id = map_frame_name;

    if( !waypoint_nav_class::find_location_pose(msg.data, initial_pose.pose.pose) )
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }
    ROS_INFO_STREAM( "set initial pose : " <<  msg.data );
    ROS_INFO_STREAM( "init_pose : " << initial_pose );
    pub_initial_pose .publish(initial_pose);//初期位置の座標の送信
    return;
  }//initial_posi_name_CB


// #####################  座標を保存するコールバック(トピックから)　座標をもらう
  void initial_posi_position_CB( const geometry_msgs::Pose::ConstPtr &msg )//初期位置の設定 直接座標を受け取る場合
  {
    //クォータニオンを調べる
    if( !waypoint_nav_class::valid_quaternion_judge( msg->orientation ) )
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }//if
    ROS_INFO_STREAM( "set initial pose : (x,y) = (\t" << msg->position.x << ",\t" << msg->position.y << ")" );
    geometry_msgs::Pose pose;
    pose = *msg;
    waypoint_nav_class::stock_unnamed_new_location( pose );
    initial_pose.header.frame_id = map_frame_name;
    initial_pose.pose.pose = *msg;
    pub_initial_pose .publish(initial_pose);//初期位置の座標の送信
    return;
  }//initial_posi_position_CB




// #####################  移動るコールバック(トピックから)　座標をもらう
  void move_position_CB( const geometry_msgs::Pose::ConstPtr& msg )//目標地点の座標を受け取り、　移動の関数へ渡す。
  {
    //クォータニオンを調べる
    if( !waypoint_nav_class::valid_quaternion_judge( msg->orientation ) )
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }//if
    geometry_msgs::Pose pose;
    pose = *msg;
    waypoint_nav_class::stock_unnamed_new_location( pose );
    //move_base_msgs::MoveBaseGoal goal;
    //goal.target_pose.header.frame_id = map_frame_name;
    //goal.target_pose.pose = *msg;
    ROS_INFO_STREAM( "move to (x,y)= (\t" << pose.position.x << ",\t" << pose.position.y << ")" );
    boost::thread move_actionlib_thread( &waypoint_nav_class::move_actionlib, this, pose );
    return;
  }//move_position_CB


// #####################  移動るコールバック(トピックから)　名前をもらう
  void move_name_CB( const std_msgs::String msg )//目標地点の名前を受け取り、名前を座標に変換し、座標を移動の関数に渡す。
  {
    geometry_msgs::Pose pose;
    //move_base_msgs::MoveBaseGoal goal;
    //goal.target_pose.header.frame_id = map_frame_name;
    if( !waypoint_nav_class::find_location_pose(msg.data, pose ) )//goal.target_pose.pose
    {
      waypoint_nav_class::pub_arrive_flag_func( false );
      return;
    }
    ROS_INFO_STREAM( "move to " << msg.data );
    boost::thread move_actionlib_thread( &waypoint_nav_class::move_actionlib, this, pose );
    return;
  }//move_name_CB






// #####################  実際に移動する関数

  bool move_actionlib( geometry_msgs::Pose pose )//移動する関数
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = map_frame_name;
    goal.target_pose.pose = pose;

      //ROS_INFO_STREAM( "move_target_pose " << pose );

    move_base_client client( "/move_base", true );
    //ROS_INFO_STREAM( "wait for server. " << server_wake_wait_time << "[sec]" );
    bool server_wake_before_timeout = client.waitForServer( ros::Duration( server_wake_wait_time ) );//ros::Duration(5.0)
    if( !server_wake_before_timeout )
    {
      //ROS_WARN_STREAM( "Action server did not wake before the time out." );
      waypoint_nav_class::pub_arrive_flag_func( false );
      client.cancelGoal();//移動のキャンセル
      return false;
    }//if
    //ROS_INFO_STREAM( " server OK. -> send posi." );
    this->goal = goal;
    client.sendGoal( goal );
    bool finished_before_timeout = client.waitForResult(ros::Duration( server_action_wait_time ));
    if( finished_before_timeout )
    {
      actionlib::SimpleClientGoalState state = client.getState();
      //ROS_INFO_STREAM( "Action finished: " << state.toString().c_str() );
      if(  state.toString() == "SUCCEEDED" )//成功
      {
        //ROS_INFO_STREAM( "The action ended and arrived." );
        waypoint_nav_class::pub_arrive_flag_func( true );
        return true;
      }//if

      else if( state.toString() == "ABORTED" )//中断された場合
      {
        ROS_WARN_STREAM( "For some reason, I canceled the action." );
        waypoint_nav_class::pub_arrive_flag_func( false );
        client.cancelGoal();//移動のキャンセル
        return false;
      }//else if


      return true;
    }//if
    else
    {
      //ROS_INFO_STREAM( "Action did not finish before the time out." );
      waypoint_nav_class::pub_arrive_flag_func( false );
      client.cancelGoal();//移動のキャンセル
      return false;
    }//else
    return false;
  }//move_actionlib



private://クラスの中で共有する変数
 	ros::NodeHandle nh;
  ros::Subscriber sub_initial_posi_position;
  ros::Subscriber sub_initial_posi_name;
  ros::Subscriber sub_target_posi_position;
  ros::Subscriber sub_target_posi_name;
  ros::Subscriber sub_target_posi_position_stock;
  tf::TransformBroadcaster br;

  ros::ServiceServer waypoint_move_service;
  ros::ServiceServer waypoint_initial_service;
  ros::ServiceServer waypoint_stock_service;


  ros::Timer timer;

  ros::Publisher pub_initial_pose;
  ros::Publisher pub_arrive_flag;
  ros::Publisher pub_location_marker;

  std::string param_head_name;//param_getで取得するための名前

  move_base_msgs::MoveBaseGoal goal;//目標座標を保持しておく



  geometry_msgs::PoseWithCovarianceStamped initial_pose;//初期位置の保持
  int new_position_count;//新しい座標を登録する際のナンバリングに使う
  double server_wake_wait_time;//move_base_のサーバが立ち上がるまで待つ時間
  double server_action_wait_time;//移動が終わるまで待てる時間
  std::string map_frame_name;//map上の地点の基準フレーム名

  std::vector<name_and_posi_set> location_vec;//tfに送信するための座標の格納

};//waypoint_nav_class




int main( int argc, char **argv )//メイン関数
{
  ros::init( argc, argv, "waypoint_nav" );
  ROS_INFO_STREAM( "waypoint_nav start! " );
  waypoint_nav_class wpn;//上の関数を実行

  ros::spin();
  return 0;
}//main
