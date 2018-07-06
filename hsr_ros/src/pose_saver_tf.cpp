#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <unistd.h>

#include <time.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

class pose_keep{

public:

    pose_keep(){
		sub_posi = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1, &pose_keep::Callback, this);


    std::cout<<"tf: "<<std::endl;
		point_name="table0";
		save_flag=false;

    std::string save_location_folder_path;
    ros::param::get( "save_location_folder_path", save_location_folder_path );

		time_t now = time(NULL);
    struct tm *pnow = localtime(&now);

	  file_name << save_location_folder_path << "/saved_location_"<< pnow->tm_mon + 1 <<"_"<< pnow->tm_mday <<"_"<< pnow->tm_hour <<"_"<< pnow->tm_min <<".yaml";

	}
	~pose_keep(){}

	void Callback(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
	{
		//tfで変換
		tf::StampedTransform transform;

		listener.waitForTransform("/map","base_footprint",ros::Time(0),ros::Duration(3.0));
		listener.lookupTransform("/map","base_footprint",ros::Time(0),transform);

		/*
		std::cout<<"transform.getOrigine().x(): "<<transform.getOrigin().x()<<std::endl;
		std::cout<<"transform.getOrigine().y(): "<<transform.getOrigin().y()<<std::endl;
		std::cout<<"transform.getOrigine().z(): "<<transform.getOrigin().z()<<std::endl;
		std::cout<<"transform.getRotation().x(): "<<transform.getRotation().x()<<std::endl;
		std::cout<<"transform.getRotation().y(): "<<transform.getRotation().y()<<std::endl;
		std::cout<<"transform.getRotation().z(): "<<transform.getRotation().z()<<std::endl;
		std::cout<<"transform.getRotation().w(): "<<transform.getRotation().w()<<std::endl;
		*/
		//std::cout << "Callback " << std::endl;
		//std::cout << "received: " << pose_msg -> translation << " " << pose_msg -> rotation << std::endl;
		//std::cout << "received: " << pose_msg -> transforms[0].transform << std::endl;
		//std::cout << "received: " << pose_msg -> transforms[0] << std::endl;
		//std::cout << "received_num: " << pose_msg -> transforms.size() << std::endl;

		if (save_flag==true)
        {
			save_flag=false;
            //保存する．
			point_name_vec.push_back(point_name);
			translation_x. push_back(transform.getOrigin().x() );
			translation_y. push_back(transform.getOrigin().y() );
			translation_z. push_back(transform.getOrigin().z() );

			rotation_x. push_back(transform.getRotation().x() );
			rotation_y. push_back(transform.getRotation().y() );
			rotation_z. push_back(transform.getRotation().z() );
			rotation_w. push_back(transform.getRotation().w() );

			std::cout<<"transform.getOrigine().x(): "<<transform.getOrigin().x()<<std::endl;
			std::cout<<"transform.getOrigine().y(): "<<transform.getOrigin().y()<<std::endl;
			std::cout<<"transform.getOrigine().z(): "<<transform.getOrigin().z()<<std::endl;

			std::cout<<"transform.getRotation().x(): "<<transform.getRotation().x()<<std::endl;
			std::cout<<"transform.getRotation().y(): "<<transform.getRotation().y()<<std::endl;
			std::cout<<"transform.getRotation().z(): "<<transform.getRotation().z()<<std::endl;
			std::cout<<"transform.getRotation().w(): "<<transform.getRotation().w()<<std::endl;

			std::cout << "I'v kept it.\n"<< std::endl;
       	}//if



		if(save_flag==false)
		{
			std::cout << "前回("<< point_name <<")とは違うキーを入力してください。「q」で終了。"<< std::endl;
			while(true)
			{

				cin >> point_name;
				if (point_name=="\n" )
				{
					std::cout<< point_name <<std::endl;
				}
				else
				{
					if(point_name =="q")
					{
						std::cout<< "OK,I'll end...." <<std::endl;


						ofstream ofs(file_name.str().c_str());
						if(ofs)
						{
						for(int i=0;i<point_name_vec.size();i++)
							{
								ofs << std::endl;
								ofs << "# "<< point_name_vec[i] << " #" << std::endl;
								ofs << std::endl;
								ofs << "/" << point_name_vec[i] << "_translation_x: " << translation_x[i] << std::endl;
								ofs << "/" << point_name_vec[i] << "_translation_y: " << translation_y[i] << std::endl;
								ofs << "/" << point_name_vec[i] << "_translation_z: " << translation_z[i] << std::endl;
								ofs << std::endl;
								ofs << "/" << point_name_vec[i] << "_rotation_x: " << rotation_x[i] << std::endl;
								ofs << "/" << point_name_vec[i] << "_rotation_y: " << rotation_y[i] << std::endl;
								ofs << "/" << point_name_vec[i] << "_rotation_z: " << rotation_z[i] << std::endl;
								ofs << "/" << point_name_vec[i] << "_rotation_w: " << rotation_w[i] << std::endl;
								ofs << "#======================================#" << std::endl;
							}//for
							ofs.close();
							std::cout<< "「　" << file_name.str() << "　」として保存完了。" <<std::endl;
						}//if
						else
						{
							ofs.close();
							std::cout<< file_name.str() << "　は作成できませんでした。ｍ（_ _;）ｍ" <<std::endl;
							std::cout<< "ターミナルが今居る階層に注意してください。" <<std::endl;
						}//else


						std::cout<<"\n ctr + z で終了"<<std::endl;
						std::cout<<"それでもダメならxボタンで閉じて"<<std::endl;
						while(1){};
						exit(EXIT_SUCCESS);
					}
					save_flag=true;
					break;
				}
			};
		}

	}

private:
 	ros::NodeHandle nh;
    ros::Subscriber sub_posi;


	string point_name;
	bool save_flag;

	stringstream file_name;

	vector<string> point_name_vec;

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
	ros::init(argc, argv, "pose_saver_edu");
	//ros::NodeHandle n;
	//ros::Subscriber pose = n.subscribe("tf", 1000, Callback);
	std::cout<<"\nキーを入力すると座標を保存"<<std::endl;

	pose_keep pk;//上の関数を実行


	ros::spin();

	return 0;
}
