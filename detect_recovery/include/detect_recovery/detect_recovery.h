/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Anchuanxu.
*  All rights reserved.
* Author: Chuanxu An
*********************************************************************/
#ifndef DETECT_RECOVERY_DETECT_RECOVERY_H
#define DETECT_RECOVERY_DETECT_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
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
