/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Anchuanxu.
*  All rights reserved.
* Author: Chuanxu An
*********************************************************************/
#include <detect_recovery/detect_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <std_msgs/Int16MultiArray.h>
#include <tf/transform_listener.h>
#include <cmath>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(detect_recovery::DetectRecovery, nav_core::RecoveryBehavior)

namespace detect_recovery
{
DetectRecovery::DetectRecovery()
{
  ;
}

//void DetectRecovery::initialize(std::string scan_topic, std::string odom_topic)
void DetectRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  nh_ = new ros::NodeHandle("~/");
  laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectRecovery::scanCallback,this,_1));
  odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectRecovery::odomCallback,this,_1));
  vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel",1);
  base_frame_ = "base_link";
  laser_frame_ = "laser";
  nh_->setParam("detect_recovery/base_frame", base_frame_);
  nh_->setParam("detect_recovery/laser_frame", laser_frame_);
  laser_sub_.shutdown();
  odom_sub_.shutdown();
  this->odom_data_.header.stamp = ros::Time::now();
  this->laser_data_.header.stamp = ros::Time::now();

  getLaserTobaselinkTF(laser_frame_, base_frame_);
  move_base_cancel_ = false;
  pi = 3.14;
  frequency_ = 10;
  initialized_ = true;
}

DetectRecovery::~DetectRecovery()
{
  if(nh_)
  {
    delete nh_;
    nh_ = nullptr;
  }
}

void DetectRecovery::runBehavior()
{
    ROS_INFO("---Detect Recovery started---");


    laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectRecovery::scanCallback,this,_1));
    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectRecovery::odomCallback,this,_1));

    ros::NodeHandle movebaseCancel_nh;
    ros::Subscriber movebasecancel_sub;
    movebasecancel_sub = movebaseCancel_nh.subscribe<actionlib_msgs::GoalID>("/move_base/cancel",1,
                                        boost::bind(&DetectRecovery::movebaseCancelCallback,this,_1));
    //如果有中断请求，则停止恢复
    if(move_base_cancel_ == true)
    {
      publishZeroVelocity();
      ROS_INFO("---move_base canceled---");
      move_base_cancel_ = false;

      laser_sub_.shutdown();
      odom_sub_.shutdown();
      return;
    }

    //从里程计获取现在的位置和姿态
    nav_msgs::Odometry odom_data;
    //ros::Duration(0.5).sleep();
    this->getOdomData(odom_data);
    double robot_start_x = odom_data.pose.pose.position.x;
    double robot_start_y = odom_data.pose.pose.position.y;
    double robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
    uint32_t count_step = 0;
    while(isnan(robot_start_t))
    {
       ROS_INFO("can't get right robot post");
       if(count_step++ > 10)
       {
         laser_sub_.shutdown();
         odom_sub_.shutdown();
         return;
       }
       ros::Duration(1).sleep();
       this->getOdomData(odom_data);
       robot_start_x = odom_data.pose.pose.position.x;
       robot_start_y = odom_data.pose.pose.position.y;
       robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
    }
    //定义现在的位置和姿态
    double robot_current_x = robot_start_x;
    double robot_current_y = robot_start_y;
    double robot_current_t = robot_start_t;


    //获取激光数据
    std::vector< std::pair<double,double> > laser_point;
    this->getLaserPoint(laser_point);//使用回调函数赋值
    count_step=0;
    while(laser_point.size() == 0) //如果激光没有数据，等待10s，如果还没有，退出恢复
    {
      ROS_INFO("can't get scan data");
      if(count_step++ > 10)
      {
        laser_sub_.shutdown();
        odom_sub_.shutdown();
        return;
      }
      ros::Duration(1).sleep();
      this->getLaserPoint(laser_point);//使用回调函数赋值
    }
    size_t laser_data_size = laser_point.size();
    bool no_point = false;
    bool turn_half = false;
    bool turn_done = false;
    bool goStright = false;
    geometry_msgs::Twist cmd_vel;

    while(ros::ok() && !goStright){
    while(ros::ok() && turn_done == false)//如果还没有旋转一周，就可以开始恢复了。
    {
      this->getLaserPoint(laser_point);//获取最新激光数据
      size_t index = 0;
      for(index=0; index<laser_data_size; index++)
      {
        if(laser_point[index].first < 0.6 && fabs(laser_point[index].second) < 0.35) //截取前方60cm，左右35cm的数据
          break;//发现障碍物，直接跳出遍历循环。
      }
      if(index == laser_data_size)//如果顺利扫描一圈，则说明没有障碍物
      {
        no_point = true;
      }
      cmd_vel.angular.z = no_point == true?0:0.25;//没有障碍物，停下机器人，有就以0.25的速度继续旋转
      this->vel_pub_.publish(cmd_vel);
      if(no_point == true)
        break;
      this->getOdomData(odom_data);
      robot_current_t = tf::getYaw(odom_data.pose.pose.orientation);
      if(fabs(robot_current_t - robot_start_t) > 2)
        turn_half = true;//旋转了一半
      if(turn_half == true && fabs(robot_current_t-robot_start_t) < 0.1)
      {
        //确实已经旋转了一半 而且角度差又很小，说明回到了原点。
        turn_done = true;
        cmd_vel.angular.z = 0;
        this->vel_pub_.publish(cmd_vel);//旋转一圈，停止旋转。
      }
      ros::Duration(0.05).sleep();
      ros::spinOnce();
    }
    //前进逻辑
    if(no_point == true)
    {
      this->getOdomData(odom_data);
      robot_current_x = odom_data.pose.pose.position.x;
      robot_current_y = odom_data.pose.pose.position.y;

      robot_start_x = robot_current_x;
      robot_start_y = robot_current_y;
      double diff_x = robot_current_x - robot_start_x;
      double diff_y = robot_current_y - robot_start_y;

      double distance = sqrt(diff_x*diff_x+diff_y*diff_y);
      while(ros::ok() && distance < 0.2)
      {
        //std::cout << "distance: " << distance << std::endl; //debug
        this->getLaserPoint(laser_point);
        size_t index = 0;
        for(index=0;index<laser_point.size();index++)
        {
          if(laser_point[index].first < 0.4 && fabs(laser_point[index].second) < 0.35)
          {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            this->vel_pub_.publish(cmd_vel);//前方又有了障碍物，先停止
            no_point = false;
            break;//跳出遍历循环
//            turn_done = false;
//            turn_half = false;
          }
        }
        if(!no_point)
        {
          break;
        }
        //再跳出直行循环；

        cmd_vel.linear.x = 0.08;
        cmd_vel.angular.z = 0;
        this->vel_pub_.publish(cmd_vel);

        //更新位置
        this->getOdomData(odom_data);
        robot_current_x = odom_data.pose.pose.position.x;
        robot_current_y = odom_data.pose.pose.position.y;
        diff_x = robot_current_x - robot_start_x;
        diff_y = robot_current_y - robot_start_y;

        distance = sqrt(diff_x*diff_x+diff_y*diff_y);
        //std::cout << "+++distance: " << distance << std::endl; //debug
        if(distance >= 0.2)
        {
          goStright = true;
        }
        ros::Duration(0.05).sleep();
        ros::spinOnce();
      }
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      this->vel_pub_.publish(cmd_vel);
    }
    }
    ROS_INFO("detect recovery end");
    laser_sub_.shutdown();
    odom_sub_.shutdown();
    return;
}
void DetectRecovery::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  ROS_INFO_ONCE("odom data recevied");
  boost::mutex::scoped_lock lock(this->odom_mutex_);
  this->odom_data_ = *msg;
}
void DetectRecovery::movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr &msg)
{
  move_base_cancel_ = true;
  return;
}
  void DetectRecovery::getOdomData(nav_msgs::Odometry &data)
  {
    ros::Time now = ros::Time::now();
    if(now.toSec() - this->odom_data_.header.stamp.toSec() > 0.5){
      return;
    }
    data = this->odom_data_;
  }
  void DetectRecovery::getLaserData(sensor_msgs::LaserScan &data)
  {
    ros::Time now = ros::Time::now();
    if(now.toSec() - this->laser_data_.header.stamp.toSec() > 0.5){
      return;
    }
    data = this->laser_data_;
  }
void DetectRecovery::getLaserPoint(std::vector<std::pair<double, double> > &data)
{
  data = this->point_vec_;
}
void DetectRecovery::publishZeroVelocity()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;
  this->vel_pub_.publish(cmd_vel);
}
void DetectRecovery::getLaserTobaselinkTF(std::string laser_frame, std::string base_frame)
{
  tf::TransformListener* tf_ = new tf::TransformListener(ros::Duration(10));
  while(ros::ok())
  {
    if(tf_->waitForTransform(base_frame, ros::Time::now(), laser_frame, ros::Time::now(), laser_frame, ros::Duration(1)))
    {
      tf_->lookupTransform(base_frame,laser_frame,ros::Time::now(),transform);
      break;
    }
    ROS_WARN("frame %s to %s unavailable",base_frame.c_str(),laser_frame.c_str());
    ros::Duration(0.5).sleep();
  }
  //debug
//  std::cout << "laserTbase:" << std::endl;
//  std::cout << "Tx:" << transform.getOrigin().getX() << std::endl;
//  std::cout << "Ty:" << transform.getOrigin().getY() << std::endl;
//  std::cout << "Tz:" << transform.getOrigin().getZ() << std::endl;
//  std::cout << "Rx:" << transform.getRotation().getX() << std::endl;
//  std::cout << "Ry:" << transform.getRotation().getY() << std::endl;
//  std::cout << "Rz:" << transform.getRotation().getZ() << std::endl;
//  std::cout << "Rw:" << transform.getRotation().getW() << std::endl;

  delete tf_;
  tf_ = nullptr;
}
void DetectRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO_ONCE("scan data recevied");
  boost::mutex::scoped_lock lock(this->laser_mutex_);
  laser_data_ = *msg;
  this->point_vec_.clear();
  double min_angle = laser_data_.angle_min;
  double max_angle = laser_data_.angle_max;
  double laser_angle_increment = laser_data_.angle_increment; //雷达数据的角度间隔
  size_t laser_point_size = laser_data_.ranges.size();//雷达数据一帧的点数
  double theta = 0; //转角置为０；

  std::pair<double,double> tem_pair;
  for (size_t i = 0; i < laser_point_size; i++) {
    //做一些必要的判断，筛除无用的点
    if(isfinite(laser_data_.ranges[i]) == false)
      continue;
    if(isinf(laser_data_.ranges[i]) == true)
      continue;
    if(isnan(laser_data_.ranges[i]) == true)
      continue;
    tf::Quaternion q = transform.getRotation();
    theta = min_angle + i * laser_angle_increment + tf::getYaw(q);
    theta = normalizeAngle(theta,-M_PI,M_PI);
    if(theta >= -M_PI/2 && theta <= M_PI/2)
    {
     if(laser_data_.ranges[i] > 0)
     {
       tem_pair.first = laser_data_.ranges.at(i) * cos(theta) + transform.getOrigin().x();
       tem_pair.second = laser_data_.ranges.at(i) * sin(theta) + transform.getOrigin().y();
       this->point_vec_.push_back(tem_pair);
     }
    }
  }
}
};  // namespace detect_recovery

