#include "ros/ros.h"
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <stdlib.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>

using namespace std;

class pose_keep{

public:

	pose_keep(){
		first_flag = true;
		point_name="";
		
		std::string save_location_folder_path;
		ros::param::get( "save_location_folder_path", save_location_folder_path );

		if( save_location_folder_path == "" )
		{
			std::cout << "save_location_folder_path is empty." << std::endl;
			while(true){ros::Duration(10).sleep();}
		}

		time_t now = time(NULL);
		struct tm *pnow = localtime(&now);
		this->file_name << save_location_folder_path << "/map_location_"<< pnow->tm_mon + 1 <<"_"<< pnow->tm_mday <<"_"<< pnow->tm_hour <<"_"<< pnow->tm_min <<".yaml";

		while(true)//無限ループ
		{
			if(first_flag == true)
				std::cout <<  "\n場所名を入力してください。「q」で終了。" << std::endl;
			else
				std::cout <<  "\n前回("<< point_name <<")とは違う場所名を入力してください。「q」で終了。" << std::endl;
			std::getline(std::cin, point_name);
			if(point_name =="q")
			{
				std::cout <<  "OK,I'll end...."  << std::endl;					
				ros::Duration(2).sleep();
				exit(EXIT_SUCCESS);
			}//if
			else
			{
				std::cout << "point_name : " << point_name << std::endl;
				this->get_pose();
			}//else

		
		}//while




	}//pose_keep()//初期化＿終了
	~pose_keep(){}

	void get_pose()
	{
		//tfで変換
		tf::StampedTransform transform;
		if(!listener.waitForTransform("/map","base_footprint",ros::Time(0),ros::Duration(3.0)) )
		{
			std::cout << "位置取得失敗。	tfは出てますか？　/mapと/base_footprintのフレームは繋がっていますか？" << std::endl;
			while(1){ros::Duration(2).sleep();}
		}
		listener.lookupTransform("/map","base_footprint",ros::Time(0),transform);			


		//現在位置をファイルに出力する．


		std::cout << std::endl;
		std::cout << "transform.getOrigine().x(): "<<transform.getOrigin().x() << std::endl;
		std::cout << "transform.getOrigine().y(): "<<transform.getOrigin().y() << std::endl;
		std::cout << "transform.getOrigine().z(): "<<transform.getOrigin().z() << std::endl;

		std::cout << "transform.getRotation().x(): "<<transform.getRotation().x() << std::endl;
		std::cout << "transform.getRotation().y(): "<<transform.getRotation().y() << std::endl;
		std::cout << "transform.getRotation().z(): "<<transform.getRotation().z() << std::endl;
		std::cout << "transform.getRotation().w(): "<<transform.getRotation().w() << std::endl;

		ofstream ofs(this->file_name.str().c_str(), ios::app);
		if(ofs)
		{
			ofs << std::endl;
			ofs << "# "<< point_name << " #" << std::endl;
			ofs << std::endl;
			ofs << "/" << point_name << "_translation_x: " << transform.getOrigin().x() << std::endl;
			ofs << "/" << point_name << "_translation_y: " << transform.getOrigin().y() << std::endl;
			ofs << "/" << point_name << "_translation_z: " << transform.getOrigin().z() << std::endl;
			ofs << std::endl;
			ofs << "/" << point_name << "_rotation_x: " << transform.getRotation().x() << std::endl;
			ofs << "/" << point_name << "_rotation_y: " << transform.getRotation().y() << std::endl;
			ofs << "/" << point_name << "_rotation_z: " << transform.getRotation().z() << std::endl;
			ofs << "/" << point_name << "_rotation_w: " << transform.getRotation().w() << std::endl;
			ofs << "#======================================#" << std::endl;
			ofs.close();
			if(first_flag == true)
				std::cout <<  "「　" << this->file_name.str() << "　」として保存完了。"  << std::endl;
			else
				std::cout <<  "「　" << this->file_name.str() << "　」に追記完了。"  << std::endl;
		}//if
		else
		{
			ofs.close();
			std::cout <<  this->file_name.str() << "　は作成できませんでした。ｍ(_ _;)ｍ"  << std::endl;
			std::cout << "ファイルのパスを確認して下さい。" << std::endl;
		
		}//else	

		this->first_flag = false;
		return;
	}//get_pose()



private:
	ros::NodeHandle nh;
	ros::Subscriber sub_posi;

	string point_name;
	bool first_flag;

	stringstream file_name;

	tf::TransformListener listener;
};//pose_keep




int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_saver");
	std::cout << "\n場所名を入力すると位置座標を保存" << std::endl;
	pose_keep pk;//上の関数を実行
	ros::spin();
	return 0;
}
