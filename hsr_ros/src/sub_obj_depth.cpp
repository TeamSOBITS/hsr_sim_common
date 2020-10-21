#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <hsr_ros/obj_depth.h>
#include <iostream>


class object_depth_class{
  private:
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/grasped_object_depth",1);
    ros::Subscriber sub;
    ros::ServiceServer depth_service;
    tf::TransformListener listener;
    std::string topic_name = "/hsrb/head_rgbd_sensor/depth/points";
    std::string tf_frame  = "hand_motor_dummy_link";
    std::string tf_frame_ = "head_rgbd_sensor_rgb_frame";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2 pc2_transformed;
    pcl::PassThrough<pcl::PointXYZ> pass;
    
  public:
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass;
    pcl::PointXYZ min_pt, max_pt;
    tf::StampedTransform transform;
    bool pub_filterd_flug;

    //コンストラクタ
    object_depth_class()
    {
      //pass_through用のクラウドを宣言
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>());
      //サービスおよびサブスクライバの起動
      this->depth_service = nh.advertiseService("get_object_depth", &object_depth_class::server, this);
      this->sub = nh.subscribe(this->topic_name,1, &object_depth_class::depth_cb, this);
    }
    //call back関数
    void depth_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
      try{
          //tf_frame1からtf_frameまでのTFを取得している
          this->listener.lookupTransform(tf_frame,tf_frame_,ros::Time(0),this->transform);
          //ROSのmsg型からPCLで使えるように
          pcl::fromROSMsg(*msg, cloud);
          //ポイントクラウドのポインタを生成
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(cloud));
          pcl::getMinMax3D(cloud,this->min_pt, this->max_pt);
          //std::cout << "min_x= " << this->min_pt.x << std::endl;
          //std::cout << "min_y= " << this->min_pt.y << std::endl;
          //std::cout << "min_z= " << this->min_pt.z << std::endl;
          //passthroughフィルター 
          this->pass.setInputCloud(cloud_ptr);
          this->pass.setFilterFieldName("z");
          this->pass.setFilterLimits(0, this->min_pt.z + 0.5);
          this->pass.filter(cloud);
          //PCL→ROSmsgへ…
          pcl::toROSMsg(cloud, cloud_msg);
          //hsrのhand基準にtf変換
          pcl_ros::transformPointCloud(tf_frame,this->transform,cloud_msg,pc2_transformed);
          pcl::fromROSMsg(pc2_transformed, this->cloud);
          pcl::getMinMax3D(cloud,this->min_pt, this->max_pt);
          //std::cout << "min_x= " << this->min_pt.x << std::endl;
          //std::cout << "min_y= " << this->min_pt.y << std::endl;
          //std::cout << "min_z= " << this->min_pt.z << std::endl;
          //std::cout << "max_x= " << this->max_pt.x << std::endl;
          //std::cout << "max_y= " << this->max_pt.y << std::endl;
          //std::cout << "max_z= " << this->max_pt.z << std::endl;
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
      //デバッグ用（フィルタリングした点群をpub）
      if (pub_filterd_flug == true) pub.publish(cloud);
      //sub.shutdown();
    }
    //service
    bool server(hsr_ros::obj_depth::Request &req, hsr_ros::obj_depth::Response &res)
    {
      ROS_INFO("service called");
      this->pub_filterd_flug = req.request;
      //this->sub = nh.subscribe(this->topic_name,1, &object_depth_class::depth_cb, this);
      res.x = fabs(this->max_pt.x) + fabs(this->min_pt.x);
      if (max_pt.x > 0) res.x = fabs(this->min_pt.x);
      res.y = fabs(this->max_pt.y) + fabs(this->min_pt.y);
      res.z = fabs(this->max_pt.z) + fabs(this->min_pt.z);

      ROS_INFO("Get obj depth");
      std::cout << res << std::endl;
      return true;
    }
};

int main(int argc, char** argv)
{
  //ROS initialize
  ros::init (argc, argv, "object_depth");

  object_depth_class obj_depth;
  while(ros::ok() == true)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  return 0;
}
