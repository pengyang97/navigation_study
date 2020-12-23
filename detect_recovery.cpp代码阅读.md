#  detect_recovery.cpp代码阅读

​		自己通过在对firefly机器人进行导航时，观察到终端的日志输出出现了有关detect_recovery的信息，这种情况机器人一般是前方遇到了障碍物，伴随着局部路径规划失败的日志信息同时出现的。于是自己通过查看detect_recovery功能包中的源码了解到它也是在原先的`nav_core::RecoveryBehavior`类下派生出的一个类，它来代替原先move_base中的旋转恢复行为rotate_recovery。以下是自己对代码的简单梳理。

* detect_recovery的头文件首先对detect_recovery进行了一些函数的声明和成员属性的定义

```C++
namespace detect_recovery
{
/**
 * @class RoRecovery
 * @brief A recovery behavior that ros the robot in-place to attempt to clear out space
 */
class DetectRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  DetectRecovery();

  /**
   * @brief  Initialization function for the DetectRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
 void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Run the DetectRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the detect recovery behavior
   */
  ~DetectRecovery();

private:
  ros::NodeHandle *nh_;
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  bool move_base_cancel_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
  base_local_planner::CostmapModel* world_model_;
  double pi;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void publishZeroVelocity();

// new add
  void getLaserData(sensor_msgs::LaserScan& data);
  void getLaserPoint(std::vector< std::pair<double,double> >& data);
  void getOdomData(nav_msgs::Odometry& data);
  ros::Subscriber laser_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  vel_pub_;
  std::vector<std::pair<double,double>> point_vec_;
  std::string base_frame_, laser_frame_;
  sensor_msgs::LaserScan laser_data_;
  nav_msgs::Odometry odom_data_;
  boost::mutex laser_mutex_;
  boost::mutex odom_mutex_;
  tf::StampedTransform transform;
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);
  void getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame);
  double inline normalizeAngle(double val, double min, double max)
  {
    double norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max-min));
    else
      norm = max - fmod((min - val), (max-min));
    return norm;
  }
};
};  // namespace detect_recovery
#endif  // DETECT_RECOVERY_DETECT_RECOVERY_H
```

接下来对声明的函数进行分析

* ` DetectRecovery::initialize();`首先是传入一些参数变量进行初始化操作，通过新开辟一个句柄来对雷达，里程计和向底盘发布速度的节点进行管理和参数的设置。

```C++
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
  laser_sub_.shutdown();//开始雷达不订阅
  odom_sub_.shutdown();//里程计不订阅信息
  this->odom_data_.header.stamp = ros::Time::now();//里程计和雷达数据的时间设为当前时刻
  this->laser_data_.header.stamp = ros::Time::now();
  //设置一些状态
  getLaserTobaselinkTF(laser_frame_, base_frame_);
  move_base_cancel_ = false;
  pi = 3.14;
  frequency_ = 10;
  initialized_ = true;
}
```

* `DetectRecovery::runBehavior()`函数是实现恢复行为的主要函数，其中他又调用了一些回调函数

```C++
void DetectRecovery::runBehavior()
{
    ROS_INFO("---Detect Recovery started---");//提示开始进行恢复行为了

//雷达和里程计开始订阅接受信息
    laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectRecovery::scanCallback,this,_1));//调用回调函数scanCallback，主要是获取一些激光数据的信息，并对接收到的激光点进行帅选，剔除一些无用点
    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectRecovery::odomCallback,this,_1));//调用回调函数odomCallback，获取里程计的数据信息

    ros::NodeHandle movebaseCancel_nh;//创建一个取消movebase的节点句柄
    ros::Subscriber movebasecancel_sub;//创建一个名为movebasecancel_sub订阅者
    //订阅的话题的消息类型为actionlib_msgs::GoalID，话题名为：/move_base/cancel，回调函数movebaseCancelCallback
    movebasecancel_sub = movebaseCancel_nh.subscribe<actionlib_msgs::GoalID>("/move_base/cancel",1,
                                        boost::bind(&DetectRecovery::movebaseCancelCallback,this,_1));
    //如果有中断请求，则停止恢复
    if(move_base_cancel_ == true)
    {
      publishZeroVelocity();//发布速度为零，使机器人停止
      ROS_INFO("---move_base canceled---");
      move_base_cancel_ = false;

      laser_sub_.shutdown();
      odom_sub_.shutdown();
      return;
    }

    //从里程计获取现在的位置和姿态
    nav_msgs::Odometry odom_data;
    //ros::Duration(0.5).sleep();
    this->getOdomData(odom_data);//通过getOdomData()函数获取里程计的当前时刻的位姿信息
    double robot_start_x = odom_data.pose.pose.position.x;
    double robot_start_y = odom_data.pose.pose.position.y;
    double robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
    //接下来进入一个循环的判断，对于得到的里程计的偏航角如果转换不成功，那么进入这个while循环，持续的接受里程计的信息，如果超过了设定的次数（10），则表示机器人当前位姿没有办法得到，这时也不再订阅雷达和里程计的消息，恢复行为也就失败了，退出恢复
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


    //获取激光数据（这里的逻辑和上面的里程计一样），放到laser_point容器中
    std::vector< std::pair<double,double> > laser_point;
    this->getLaserPoint(laser_point);//使用回调函数赋值，向容器中存放激光数据
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
    //这里设置一些标志位
    bool no_point = false;
    bool turn_half = false;
    bool turn_done = false;
    bool goStright = false;
    geometry_msgs::Twist cmd_vel;
	//这里是不是让机器人向前移动一点点的距离帮助机器人恢复？
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
        break;//
        //获取当前时刻机器人的位姿
      this->getOdomData(odom_data);
      robot_current_t = tf::getYaw(odom_data.pose.pose.orientation);
        //判断恢复行为，机器人是不是旋转了一半
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
    //前进逻辑，这里机器人往前面走了一点点的距离（但是distance那里没有看懂）
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
          //因为判断出了往前直行时，前方0.4长0.35宽的范围内出现了障碍物，就不能再直行了
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
          goStright = true;//说明往前直行了一点
        }
        ros::Duration(0.05).sleep();
        ros::spinOnce();
      }
        //不再让机器人动
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      this->vel_pub_.publish(cmd_vel);
    }
    }
    ROS_INFO("detect recovery end");//恢复行为结束
    laser_sub_.shutdown();
    odom_sub_.shutdown();
    return;
}
```

##  detect_recovery.h

```C++
#ifndef DETECT_RECOVERY_DETECT_RECOVERY_H
#define DETECT_RECOVERY_DETECT_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>		//相当于#include <tf/transform_listener.h>
#include <base_local_planner/costmap_model.h>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <nav_core/recovery_behavior.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <boost/thread/mutex.hpp>
#include <boost/bind/bind.hpp>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

namespace detect_recovery
{
/**
 * @class RoRecovery
 * @brief A recovery behavior that ros the robot in-place to attempt to clear out space
 */
class DetectRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  DetectRecovery();

  /**
   * @brief  Initialization function for the DetectRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
 void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Run the DetectRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the detect recovery behavior
   */
  ~DetectRecovery();

private:
  ros::NodeHandle *nh_;
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  bool move_base_cancel_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
  base_local_planner::CostmapModel* world_model_;
  double pi;
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void publishZeroVelocity();

// new add
  void getLaserData(sensor_msgs::LaserScan& data);
  void getLaserPoint(std::vector< std::pair<double,double> >& data);
  void getOdomData(nav_msgs::Odometry& data);
  ros::Subscriber laser_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  vel_pub_;
  std::vector<std::pair<double,double>> point_vec_;
  std::string base_frame_, laser_frame_;
  sensor_msgs::LaserScan laser_data_;
  nav_msgs::Odometry odom_data_;
  boost::mutex laser_mutex_;
  boost::mutex odom_mutex_;
  tf::StampedTransform transform;
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void movebaseCancelCallback(const actionlib_msgs::GoalID::ConstPtr& msg);
  void getLaserTobaselinkTF(std::string sensor_frame, std::string base_frame);
  double inline normalizeAngle(double val, double min, double max)
  {
    double norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max-min));
    else
      norm = max - fmod((min - val), (max-min));
    return norm;
  }
};
};  // namespace detect_recovery
#endif  // DETECT_RECOVERY_DETECT_RECOVERY_H
```

##  detect_recovery.cpp

```C++
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


// register this planner as a RecoveryBehavior plugin	将这个规划器注册登记一下，后面在move_base node中作为插件调用
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
  nh_ = new ros::NodeHandle("~/");	//相当于nh_的命名空间为<node_namespace>/node_name
  //这里使用了bind绑定函数，相当于this->DetectRecovery::scan(或odom)Callback(参数为时间戳)，其中_1为占位符
  laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectRecovery::scanCallback,this,_1));//创建雷达消息订阅者
  odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectRecovery::odomCallback,this,_1));//创建里程计消息订阅者
  vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel",1);//创建速度指令发布者
  base_frame_ = "base_link";//机器人坐标系
  laser_frame_ = "laser";//雷达坐标系
  nh_->setParam("detect_recovery/base_frame", base_frame_);//分别设置机器人和雷达坐标系的参数
  nh_->setParam("detect_recovery/laser_frame", laser_frame_);
  laser_sub_.shutdown();//恢复行为开始先不订阅雷达和里程计的信息
  odom_sub_.shutdown();
  this->odom_data_.header.stamp = ros::Time::now();		//传入scanCallback回调函数
  this->laser_data_.header.stamp = ros::Time::now();	//传入odomCallback回调函数

  getLaserTobaselinkTF(laser_frame_, base_frame_);		//通过TF将laser_frame转换到base_frame中
  move_base_cancel_ = false;//设置是否进行move_base的标志位
  pi = 3.14;
  frequency_ = 10;  //速度发布频率
  initialized_ = true;//初始化完成后将initialized_置为true
}

DetectRecovery::~DetectRecovery()
{
  //执行到最后释放新开辟的内存空间
  if(nh_)
  {
    delete nh_;
    nh_ = nullptr;
  }
}

//执行恢复行为的主体函数
void DetectRecovery::runBehavior()
{
    ROS_INFO("---Detect Recovery started---");

	//开始订阅雷达和里程计的数据消息
    laser_sub_ = nh_->subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectRecovery::scanCallback,this,_1));
    odom_sub_ = nh_->subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&DetectRecovery::odomCallback,this,_1));

    ros::NodeHandle movebaseCancel_nh;
    ros::Subscriber movebasecancel_sub;//创建一个movebasecancel_sub
    movebasecancel_sub = movebaseCancel_nh.subscribe<actionlib_msgs::GoalID>("/move_base/cancel",1,
                                        boost::bind(&DetectRecovery::movebaseCancelCallback,this,_1));
    //如果有中断请求，则停止恢复（比如机器人收到了一个新的目标点，之前的目标点就失效，需要停止当前的恢复行为），否则正常进行恢复
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
    //接下来进入一个循环的判断，对于从里程计得到的偏航角如果是无效的（nan），那么进入这个while循环，持续的接受里程计的信息，如果超过了设定的次数（10），则表示机器人当前位姿没有办法获到，这时也不再订阅雷达和里程计的消息，恢复行为也就失败了，退出恢复
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
       ros::Duration(1).sleep();//休眠一秒后再从里程计获取一下当前的位姿信息
       this->getOdomData(odom_data);
       robot_start_x = odom_data.pose.pose.position.x;
       robot_start_y = odom_data.pose.pose.position.y;
       robot_start_t = tf::getYaw(odom_data.pose.pose.orientation);
    }
    //定义机器人现在（当前）的位置和姿态
    double robot_current_x = robot_start_x;
    double robot_current_y = robot_start_y;
    double robot_current_t = robot_start_t;


    //获取激光数据
    std::vector< std::pair<double,double> > laser_point;
    this->getLaserPoint(laser_point);//使用回调函数赋值，
    count_step=0;
    while(laser_point.size() == 0) //如果激光没有数据，等待10s，如果还没有，退出恢复（就说明等待了十秒之后，依然在雷达可扫描到的范围内没有障碍物出现，就是前方完全是一片开阔地，没有必要进行恢复了）
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
    size_t laser_data_size = laser_point.size();//size_t类型的数据其实是保存了一个整数，可以转化为int并赋值给int类型的变量。(实际上就是记录存放到vector容器中获得到的激光数据)
    //定义一些标志位信息
    bool no_point = false;
    bool turn_half = false;
    bool turn_done = false;
    bool goStright = false;
    geometry_msgs::Twist cmd_vel;//定义用来发布速度消息的一个对象

    while(ros::ok() && !goStright){
    while(ros::ok() && turn_done == false)//如果还没有完成旋转一周，就可以开始恢复了。
    {
      this->getLaserPoint(laser_point);//获取最新激光数据
      size_t index = 0;
      for(index=0; index<laser_data_size; index++)
      {
        if(laser_point[index].first < 0.6 && fabs(laser_point[index].second) < 0.35) //判断前方60cm，左右35cm是否存在激光数据
          break;//发现障碍物，直接跳出对激光点的遍历循环。（相当于是激光扫描到有障碍物，其存在于正前方一个长为70cm，宽为60cm的矩形框中）
      }
      if(index == laser_data_size)//如果顺利扫描完获取的一组激光数据了，则说明没有障碍物（!是不是应该是index == laser_data_size-1？!）#####################
      {
        no_point = true;
      }
      cmd_vel.angular.z = no_point == true?0:0.25;//没有障碍物，机器人停止旋转，有障碍物就以0.25的速度继续旋转
      this->vel_pub_.publish(cmd_vel);
      if(no_point == true)
        break;//这时机器人遍历完当前获得激光点数据，且在规定的矩形框范围内没有出现障碍物，这时机器人就可以进行下一步的往前直行部分的代码了（代码的优点也在于此处：不需要非得旋转一周，只要是找到了某个区域，其满足行走的条件就可以试着按这个方向前进一段试一下）
      //下面的几行是判断机器人是否旋转了一周（通过里程计数据，计算机器人当前的位置与开始位置的间的距离）
      this->getOdomData(odom_data);
      robot_current_t = tf::getYaw(odom_data.pose.pose.orientation);
      if(fabs(robot_current_t - robot_start_t) > 2)//2？一半弧度是1.57，故意给的大一些？
        turn_half = true;//旋转了一半
      if(turn_half == true && fabs(robot_current_t-robot_start_t) < 0.1)
      {
        //确实已经旋转了一半 而且与初始点的角度差又很小，说明回到了原点。
        turn_done = true;
        cmd_vel.angular.z = 0;
        this->vel_pub_.publish(cmd_vel);//旋转一圈，停止旋转。
      }
      ros::Duration(0.05).sleep();
      ros::spinOnce();
    }//到这里机器人旋转行为就结束了，下面就是前进的逻辑
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
      while(ros::ok() && distance < 0.2)//机器人满足在他前进距离范围内时，进行下面的循环
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

//上述代码中部分函数的定义如下：
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
    //waitForTransform()第五个参数是指不随时间的坐标系laser_frame(不随时间动？)
    if(tf_->waitForTransform(base_frame, ros::Time::now(), laser_frame, ros::Time::now(), laser_frame, ros::Duration(1)))
    {
      //lookupTransfrom()可以获得两个坐标系之间的转换矩阵，参数第一个是target_frame，第二个source_frame，第三个是评估source_frame的时间，第四个参数存放转换矩阵
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
```



