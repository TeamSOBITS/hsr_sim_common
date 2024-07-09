#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define SCAN_MAX 30

class PCL_Scan_Creater
{

public:

    PCL_Scan_Creater(){

        this->initialized_flag = false;
        this->point_time_out = 1.0;
        ros::param::get("/pcl_scan_creater/pub_scan_topic_name", pub_scan_topic_name);
        ros::param::get("/pcl_scan_creater/sub_scan_topic_name", sub_scan_topic_name);
        ros::param::get("/pcl_scan_creater/sub_point_topic_name", sub_point_topic_name);
        ros::param::get("/pcl_scan_creater/laser_frame_name", laser_frame_name);
        ros::param::get("/pcl_scan_creater/camera_frame_name", camera_frame_name);
        ros::param::get("/pcl_scan_creater/depth_z_min", depth_z_min);
        ros::param::get("/pcl_scan_creater/depth_z_max", depth_z_max);
        ros::param::get("/pcl_scan_creater/depth_x_min", depth_x_min);
        ros::param::get("/pcl_scan_creater/depth_x_max", depth_x_max);
        ros::param::get("/pcl_scan_creater/point_time_out", point_time_out);

        this->sub_laser = nh.subscribe(sub_scan_topic_name, 1, &PCL_Scan_Creater::laser_cb, this);
        this->sub_point = nh.subscribe(sub_point_topic_name, 1, &PCL_Scan_Creater::point_cb, this);
        this->pub_mixed_laser = nh.advertise<sensor_msgs::LaserScan> (pub_scan_topic_name, 1);
        //this->pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("/detect_plane_cloud", 1);

    }//PCL_Scan_Creater

    void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan_in){

        if(initialized_flag == false){
            this->pcl_scan = *scan_in;
            initialized_flag = true;
        }
        this->mixed_scan = *scan_in;

        if( this->last_pcl_time + ros::Duration(this->point_time_out) > ros::Time::now() ){//最後のpclから10秒以内であれば、点群を合成
            for(int i = 0; i < this->mixed_scan.ranges.size(); i++){
                this->mixed_scan.ranges[i] = std::min(scan_in->ranges[i],this->pcl_scan.ranges[i]);
            }//for
        }//if

        this->pub_mixed_laser.publish(this->mixed_scan);
    }//laser_cb

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& input){

        if(initialized_flag == false){ return; }

        //座標変換可能か確認
        bool key = listerner.canTransform (laser_frame_name, camera_frame_name, ros::Time(0));
        if(key == false){
            ROS_ERROR("PCL_Scan_Creater : pcl canTransform failue");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*input, *cloud_input);

        //Down Sample ボクセルで軽量化
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud_input);
        vg.setLeafSize (0.01, 0.01, 1.0);
        vg.filter (*cloud_downsample);

		//base_laser基準のpointcloudに変換
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base_laser(new pcl::PointCloud<pcl::PointXYZ>);
		pcl_ros::transformPointCloud(laser_frame_name, ros::Time(0), *cloud_downsample, camera_frame_name, *cloud_base_laser, listerner);

        // filtering X limit
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud (cloud_base_laser);
        pass_x.setFilterFieldName ("x");
        pass_x.setFilterLimits (depth_x_min, depth_x_max);
        pass_x.filter (*cloud_cut_x);

		// filtering Z limit
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass_z;
		pass_z.setInputCloud (cloud_cut_x);
		pass_z.setFilterFieldName ("z");
		pass_z.setFilterLimits (depth_z_min, depth_z_max);
        pass_z.setFilterLimitsNegative (false);
		pass_z.filter (*cloud_cut);
        /*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud_cut, *cloud_color);
        for (int i = 0; i < cloud_color->points.size(); i++){
                cloud_color->points[i].g = 255;
        }//for
        sensor_msgs::PointCloud2 sensor_cloud_plane;
        pcl::toROSMsg(*cloud_color, sensor_cloud_plane);
        sensor_cloud_plane.header.frame_id = "base_laser_link";
        pub_cloud.publish (sensor_cloud_plane);
        */
        for(int i = 0; i < this->pcl_scan.ranges.size(); i++){//初期化
            this->pcl_scan.ranges[i] = SCAN_MAX;
        }

        for(int i = 0; i < cloud_cut->points.size(); i++){
            if(cloud_cut->points[i].x == 0){ continue; }

            float point_angle = atanf(cloud_cut->points[i].y / cloud_cut->points[i].x);
            int scan_pt = (point_angle / this->pcl_scan.angle_increment) + (this->pcl_scan.ranges.size() / 2);
            if(scan_pt < 0 || this->pcl_scan.ranges.size() <= scan_pt){ continue; }

            float point_len = sqrt(pow(cloud_cut->points[i].x, 2) + pow(cloud_cut->points[i].y, 2));
            if(point_len < this->pcl_scan.ranges[scan_pt]){
                this->pcl_scan.ranges[scan_pt] = point_len;
            }//if
        }//for_point

        this->last_pcl_time = ros::Time::now();//pcl情報の更新時間を更新
    }//point_cb


private:

    ros::NodeHandle nh;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_point;
    ros::Publisher pub_mixed_laser;
    //ros::Publisher pub_cloud;
    ros::Time last_pcl_time;
    tf::TransformListener listerner;

    sensor_msgs::LaserScan pcl_scan;
    sensor_msgs::LaserScan mixed_scan;
    std::string sub_point_topic_name;
    std::string sub_scan_topic_name;
    std::string pub_scan_topic_name;
    std::string laser_frame_name;
    std::string camera_frame_name;
    float depth_x_min;
    float depth_x_max;
    float depth_z_min;
    float depth_z_max;
    double point_time_out;
    bool initialized_flag;
};


int main(int argc, char** argv){

    ros::init(argc, argv, "pcl_scan_creater");
    ROS_INFO("Start pcl_scan_creater.");
    PCL_Scan_Creater psc;
    ros::spin();

    return 0;
}//main
