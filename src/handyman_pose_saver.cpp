#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>


#include <time.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

class pose_keep{

public:

    pose_keep(){

    // tfのsub
    sub_posi = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &pose_keep::Callback, this);

    // 部屋の候補
    room_candidate[0] = "initial_furniture_position";
    room_candidate[1] = "kitchen";
    room_candidate[2] = "bedroom";
    room_candidate[3] = "bedroom_2";
    room_candidate[4] = "lobby";
    room_candidate[5] = "lobby_1";
    room_candidate[6] = "lobby_2";
    room_candidate[7] = "living_room";
    room_candidate[8] = "living_room_1";
    room_candidate[9] = "living_room_2";

    // 家具の候補
    furniture_candidate[0] = "initial_room_position";
    furniture_candidate[1] = "armchair";
    furniture_candidate[2] = "bed";
    furniture_candidate[3] = "cardboard_box";
    furniture_candidate[4] = "corner_sofa";
    furniture_candidate[5] = "iron_bed";
    furniture_candidate[6] = "low_table";
    furniture_candidate[7] = "round_low_table";
    furniture_candidate[9] = "trash_box_for_burnable";
    furniture_candidate[10] = "trash_box_for_recycle";
    furniture_candidate[11] = "wagon";
    furniture_candidate[12] = "white_table";
    furniture_candidate[13] = "white_rack";
    furniture_candidate[14] = "wooden_table";
    furniture_candidate[15] = "wooden_side_table";

    // 家具の高さ(m) ・・配列番号は家具の名前の配列番号と対応
    furniture_hight_candidate[furniture_candidate[0]] = 100;
    furniture_hight_candidate[furniture_candidate[1]] = 20;
    furniture_hight_candidate[furniture_candidate[2]] = 300;
    furniture_hight_candidate[furniture_candidate[3]] = 40;
    furniture_hight_candidate[furniture_candidate[4]] = 500;
    furniture_hight_candidate[furniture_candidate[5]] = 60;
    furniture_hight_candidate[furniture_candidate[6]] = 7;
    furniture_hight_candidate[furniture_candidate[7]] = 800;
    furniture_hight_candidate[furniture_candidate[8]] = 90;
    furniture_hight_candidate[furniture_candidate[9]] = 100;
    furniture_hight_candidate[furniture_candidate[10]] = 110;
    furniture_hight_candidate[furniture_candidate[11]] = 120;
    furniture_hight_candidate[furniture_candidate[12]] = 13;
    furniture_hight_candidate[furniture_candidate[13]] = 1400;
    furniture_hight_candidate[furniture_candidate[14]] = 150;
    furniture_hight_candidate[furniture_candidate[15]] = 160;

    // 登録個数
    count = 0;

    // 選択内容を保存するか判定するフラグ
    flag = false;

    // 座標登録のファイル保存のパス
    std::string save_location_folder_path;// 保存席　hsr_sim_common/map
    ros::param::get( "save_location_folder_path", save_location_folder_path );

    // 現在時間
		time_t now = time(NULL);
    struct tm *pnow = localtime(&now);

    // 保存するファイル名
	  file_name << save_location_folder_path << "/saved_location_"<< pnow->tm_mon + 1 <<"_"<< pnow->tm_mday <<"_"<< pnow->tm_hour <<"_"<< pnow->tm_min <<".yaml";

	}
	~pose_keep(){}



  // 座標取得の関数
	void Callback(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
	{
		//tfで変換
		tf::StampedTransform transform;

		listener.waitForTransform("/map","base_footprint",ros::Time(0),ros::Duration(3.0));
		listener.lookupTransform("/map","base_footprint",ros::Time(0),transform);

    // 座標保存
		translation_x. push_back(transform.getOrigin().x() );
		translation_y. push_back(transform.getOrigin().y() );
		translation_z. push_back(transform.getOrigin().z() );

		rotation_x. push_back(transform.getRotation().x() );
		rotation_y. push_back(transform.getRotation().y() );
		rotation_z. push_back(transform.getRotation().z() );
		rotation_w. push_back(transform.getRotation().w() );

    // 出力文
		//std::cout<<"transform.getOrigine().x(): "<<transform.getOrigin().x()<<std::endl;
		//std::cout<<"transform.getOrigine().y(): "<<transform.getOrigin().y()<<std::endl;
		//std::cout<<"transform.getOrigine().z(): "<<transform.getOrigin().z()<<std::endl;

		//std::cout<<"transform.getRotation().x(): "<<transform.getRotation().x()<<std::endl;
		//std::cout<<"transform.getRotation().y(): "<<transform.getRotation().y()<<std::endl;
		//std::cout<<"transform.getRotation().z(): "<<transform.getRotation().z()<<std::endl;
		//std::cout<<"transform.getRotation().w(): "<<transform.getRotation().w()<<std::endl;


    // 座標記録開始
    while(true)
    {
      // 座標記録の流れ -- 全5工程 --
      while(true){
        /* ① @を付けるか判定 */
        std::cout << "① 「@」をつけますか？"<< std::endl;
        std::cout << "はい -> 「1」を押す/ いいえ -> 「2」を押す"<< std::endl;
        while(true){
          std::cout << "==▶ ";
          cin >> at_num;
          if(at_num==1){
            std::cout << "「@」をつけます。\n"<< std::endl;
            at_mark_vec.push_back("@");
            break;
          }//if
          else if(at_num==2){
            std::cout << "「@」をつけません。\n"<< std::endl;
            at_mark_vec.push_back("");
            break;
          }//else if
          else{
            std::cout << "押すキーが違います。　再度キーを入力し直して下さい。\n"<< std::endl;
          }//else
        }


        /* ② 部屋の名前選択 */
        std::cout << "② 部屋の名前番号を選択して下さい。" << std::endl;
        for(auto room_itr = room_candidate.begin(); room_itr != room_candidate.end(); ++room_itr) {
          std::cout << "番号 = " << room_itr->first << ", 部屋の名前 = " << room_itr->second << "\n";    // 値を表示
          max_room_num = room_itr->first;
        }
        while(true){
          std::cout << "==▶ ";
          cin >> room_num;
          std::cout << "部屋の候補総数: " << max_room_num << std::endl;
          if(room_num>max_room_num){
            std::cout << "押されたキーの番号は部屋の候補にありません。　再度キーを入力し直して下さい。\n"<< std::endl;
          }
          else{
            std::cout << "「 " << room_num << " 」 の 「 " << room_candidate[room_num] << " 」を選択完了。\n" << std::endl;
            room_num_vec.push_back(room_num);
            break;
          }
        }

        /* ③ 家具の名前選択 */
        std::cout << "③ 家具の名前番号を選択して下さい。" << std::endl;
        for(auto furniture_itr = furniture_candidate.begin(); furniture_itr != furniture_candidate.end(); ++furniture_itr) {
          std::cout << "番号 = " << furniture_itr->first << ", 家具の名前 = " << furniture_itr->second << "\n";    // 値を表示
          max_furniture_num = furniture_itr->first;
        }
        while(true){
          std::cout << "==▶ ";
          cin >> furniture_num;
          std::cout << "家具の候補総数: " << max_furniture_num << std::endl;
          if(furniture_num>max_furniture_num){
            std::cout << "押されたキーの番号は家具の候補にありません。　再度キーを入力し直して下さい。\n"<< std::endl;
          }
          else{
            std::cout << "「 " << furniture_num << " 」 の 「 " << furniture_candidate[furniture_num] << " 」を選択完了。\n" << std::endl;
            furniture_num_vec.push_back(furniture_num);
            break;
          }
        }

        /* ④ 選択した内容の確認 */
        std::cout << "④ 選択した内容は以下でよろしいですか?" << std::endl;
        std::cout << at_mark_vec[count] << room_candidate[room_num_vec[count]] << "#" << furniture_candidate[furniture_num_vec[count]] << std::endl;
        std::cout << "はい -> 「1」を押す/ いいえ -> 「2」を押す"<< std::endl;
        while(true){
          std::cout << "==▶ ";
          cin >> check_num;
          if(check_num==1){
            std::cout << "選択した内容を保存します。\n"<< std::endl;
            count++;
            flag = true;
            break;
          }//if
          else if(check_num==2){
            std::cout << "内容を再度選択し直してください。\n"<< std::endl;
            // 選択した内容の削除
            at_mark_vec.pop_back();
            room_num_vec.pop_back();
            furniture_num_vec.pop_back();
            count--;
            break;
          }//else if
          else{
            std::cout << "押すキーが違います。　再度キーを入力し直して下さい。\n"<< std::endl;
          }//else
        }//while

        // 選択した内容を保存する場合 -> ⑤へ移行
        if(flag==true){
          flag = false;
          break;
        }
      }//while


      /* ⑤ 記録を続けるか判定 */
      std::cout << "⑤ 記録を続けるか選択して下さい。" << std::endl;
      std::cout << "続ける -> 「1」を押す/ 終了する -> 「2」を押す"<< std::endl;
      std::cout << "==▶ ";
      cin >> contenue_num;
      while(true){
        if(contenue_num==1){
          std::cout << "--記録を続けます⏬--\n"<< std::endl;
          break;
        }//if
        else if(contenue_num==2){

          /* ⑥ 家具名と部屋名と座標をファイルに記録する */
          std::cout<< "\n⑥ 家具名と部屋名と座標をファイルに記録します。" <<std::endl;
          ofstream ofs(file_name.str().c_str());
          if(ofs)
          {
          for(int k=0;k<at_mark_vec.size();k++)
            {
              ofs << std::endl;
              ofs << "# " << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << " #" << std::endl;
              ofs << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_translation_x: " << translation_x[k] << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_translation_y: " << translation_y[k] << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_translation_z: " << translation_z[k] << std::endl;
              ofs << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_rotation_x: " << rotation_x[k] << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_rotation_y: " << rotation_y[k] << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_rotation_z: " << rotation_z[k] << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_rotation_w: " << rotation_w[k] << std::endl;
              ofs << std::endl;
              ofs << at_mark_vec[k] << room_candidate[room_num_vec[k]] << "#" << furniture_candidate[furniture_num_vec[k]] << "_furniture_hight: " <<  furniture_hight_candidate[furniture_candidate[furniture_num_vec[k]]] << std::endl;
              ofs << "#======================================#" << std::endl;
            }//for
            ofs.close();
            std::cout<< "「　" << file_name.str() << "　」として保存完了。" <<std::endl;
            std::cout<<"\n以上で終了です。xボタンで閉じてください。\n"<<std::endl;
            break;/* while文抜ける */
            while(1){};
          }//if
          else{
            // 保存するファイルが開けなかった場合
            std::cout<< "危険!!: " <<std::endl;
            std::cout<< "このままでは " << file_name.str() << "　を作成できません。" <<std::endl;
            std::cout<< "ファイルを保存するパス名を確認してください。" <<std::endl;
          }//else
        }//else if
        else{
          std::cout << "押されたキーが違います。　再度キーを入力し直して下さい。\n"<< std::endl;
        }//else
      }//while

    };//while

  }//callback



private:
 	ros::NodeHandle nh;
  ros::Subscriber sub_posi;

	stringstream file_name;

  int at_num;
  int room_num;
  int furniture_num;
  int contenue_num;
  int max_furniture_num;
  int max_room_num;
  int check_num;
  int count;

  bool flag;

  map<int, string> furniture_candidate;
  map<int, string> room_candidate;

  map<string, float> furniture_hight_candidate;

  vector<string> at_mark_vec;
  vector<int> room_num_vec;
	vector<int> furniture_num_vec;

	vector<float> translation_x;
	vector<float> translation_y;
	vector<float> translation_z;

	vector<float> rotation_x;
	vector<float> rotation_y;
	vector<float> rotation_z;
	vector<float> rotation_w;

	tf::TransformListener listener;

};//pose_keep




int main(int argc, char **argv)
{
	ros::init(argc, argv, "handyman_pose_saver");

	std::cout<<"\nHandyman 部屋と家具の名前・座標の登録開始"<<std::endl;

	pose_keep pk;//上の関数を実行

	ros::spin();

	return 0;
}
