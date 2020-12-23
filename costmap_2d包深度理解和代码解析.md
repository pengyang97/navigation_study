# costmap_2d包深度理解和代码解析

> 这里主要是补充之前的**costmap_2d**包的理解，主要是落实到具体的结构和代码的解读上。

知乎上的一篇文章总体上概括了一下costmap_2d包http://zhuanlan.zhihu.com/p/28162685

***<u>Costmap通过各层地图订阅话题、接收传感器数据，维护各层地图数据，并最终整合出一张用于路径规划的主地图。</u>***

* 这里有一张别人整理好的**costmap_2d**包的结构图（整理的很详细，需要细致的看一下）

![](/home/pengyang/图片/costmap_2d包结构图.png)

* **Costmap2DROS**类是对整个代价地图内容的封装。
* **LayeredCostmap**类是**Costmap2DROS**的类成员，**它是“主地图”，也能够管理各层地图，因为它含有指向各层子地图的指针（数据成员plugin指针组，指向各层地图），能够调用子地图的类方法，开启子地图的更新。并且，各层子地图最后都会合并到主地图上，提供给规划器的使用**。它含有**Costmap2D**类成员，这个类就是底层地图，用于记录地图数据。
* **CostmapLayer**类派生自**Layer**类和**Costmap2D**类。**Layer**类中含有子地图层用到的一些函数，如更新size、更新bound、和主地图合并等；**Costmap2D**类存储该层维护的地图数据。由**CostmapLayer**类派生出**StaticLayer**类和**ObstacleLayer**类，即静态层和障碍层，前者获取静态地图，后者通过传感器数据不断更新，获得能反映障碍物信息的子地图。

* 由**Layer**类单独派生出**InflationLayer**类，即膨胀层，用它来反映障碍物在地图上向周边的膨胀。<u>由于它的父类中不含**Costmap2D**类，所以其实膨胀层自身没有栅格地图要维护，这一点和另外两层有区别。</u>

* 下图是**Costmap2DROS**类主要的结构，也是上面几段话的一个总结。

![](/home/pengyang/图片/Costmap2DROS类结构.png)

##  1.costmap_2d_ros.h

```C++
#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

//通过下面的几个头文件可以看出来costmap_2d_ros.h包含了代价地图基本所有重要的头文件，说明Costmap2DROS这个类是NO.1
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. 
 一个用于2D成本图的ROS包装器。处理订阅主题，这些主题以PointCloud或LaserScan消息的形式提供有关障碍物的观察情况。*/
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(std::string name, tf::TransformListener& tf);
  ~Costmap2DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  void resume();

  void updateMap();

  /**
   * @brief Reset each individual layer
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent()
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
  Costmap2D* getCostmap()
    {
      return layered_costmap_->getCostmap();
    }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID()
    {
      return global_frame_;
    }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID()
    {
      return robot_base_frame_;
    }
  LayeredCostmap* getLayeredCostmap()
    {
      return layered_costmap_;
    }

  /** @brief Returns the current padded footprint as a geometry_msgs::Polygon. */
  geometry_msgs::Polygon getRobotFootprintPolygon()
  {
    return costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::Point> getRobotFootprint()
  {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::Point> getUnpaddedRobotFootprint()
  {
    return unpadded_footprint_;
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

protected:
  LayeredCostmap* layered_costmap_;
  std::string name_;
  tf::TransformListener& tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

private:
  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);

  void resetOldParameters(ros::NodeHandle& nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  void movementCB(const ros::TimerEvent &event);
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
  ros::Timer timer_;
  ros::Time last_publish_;
  ros::Duration publish_cycle;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  tf::Stamped<tf::Pose> old_pose_;
  Costmap2DPublisher* publisher_;
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;

  boost::recursive_mutex configuration_mutex_;

  ros::Subscriber footprint_sub_;
  ros::Publisher footprint_pub_;
  std::vector<geometry_msgs::Point> unpadded_footprint_;
  std::vector<geometry_msgs::Point> padded_footprint_;
  float footprint_padding_;
  costmap_2d::Costmap2DConfig old_config_;
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H

```

## 2.costmap_2d_ros.cpp

```C++
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>


using namespace std;

namespace costmap_2d
{

void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name, bool should_delete = true)
{
  if (!old_h.hasParam(name))
    return;

  XmlRpc::XmlRpcValue value;
  old_h.getParam(name, value);
  new_h.setParam(name, value);
  if (should_delete) old_h.deleteParam(name);
}

/*
Costmap2DROS的构造函数，必须提供一个tf参数，tf参数需要提供以下两个坐标系的关系：
  // get two frames
  private_nh.param("global_frame", global_frame_, std::string("/map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
  循环等待直到获得机器人底盘坐标系和global系之间的坐标转换。同时这里还初始化了一些其他的参数
*/
Costmap2DROS::Costmap2DROS(std::string name, tf::TransformListener& tf) :
    layered_costmap_(NULL),
    name_(name),
    tf_(tf),//Used for transforming point clouds
    transform_tolerance_(0.3),//timeout before transform errors
    map_update_thread_shutdown_(false),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
    plugin_loader_("costmap_2d", "costmap_2d::Layer"),
    publisher_(NULL),
    dsrv_(NULL),
    footprint_padding_(0.0)
{
  // Initialize old pose with something
  old_pose_.setIdentity();
  old_pose_.setOrigin(tf::Vector3(1e30, 1e30, 1e30));

  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle g_nh;

  // get our tf prefix获取tf前缀（作用是什么？tf::resolve用到了）
  ros::NodeHandle prefix_nh;
  std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  // get two frames传入两个坐标系
  private_nh.param("global_frame", global_frame_, std::string("/map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  // make sure that we set the frames appropriately based on the tf_prefix（这里用到了刚才获取的tf前缀）
  //确保基于tf前缀正确设置了坐标系
  //tf::resolve或者TransformListener::resolve方法的作用就是使用tf_prefix参数将frame_name解析为frame_id
  global_frame_ = tf::resolve(tf_prefix, global_frame_);
  robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  // we need to make sure that the transform between the robot base frame and the global frame is available
  //如果没有找到这两个坐标系的关系或者超时，则构造函数会一直阻塞在这里
  while (ros::ok()
      && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01),
                               &tf_error))
  {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now())
    {
      ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
               robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap检测是否需要“窗口滚动”，从参数服务器获取以下参数
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  private_nh.param("rolling_window", rolling_window, false);
  private_nh.param("track_unknown_space", track_unknown_space, false);
  private_nh.param("always_send_full_costmap", always_send_full_costmap, false);
  
  //接下来创建一个LayeredCostmap类实例layered_costmap_，作为规划器使用的主地图，并通过它管理各层地图。
  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  //接着，在参数服务器上获取“plugins”参数，这里得到的插件即为各层子地图。每层子地图使用Pluginlib（ROS插件机制）来实例化，各个层可以被独立的编译，且允许使用C++接口对Costmap做出任意的改变。循环将每个plugin即每层子地图添加进layered_costmap_类的指针组对象中，并对每层地图调用其initialize类方法，初始化各层地图。这个函数定义在Layer类中，它向各层地图“通知”主地图的存在，并调用oninitialize类方法（Layer类中的虚函数，被定义在各层地图中）。
  if (!private_nh.hasParam("plugins"))
  {
    resetOldParameters(private_nh);
  }

  if (private_nh.hasParam("plugins"))
  {
    XmlRpc::XmlRpcValue my_list;
    private_nh.getParam("plugins", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string type = static_cast<std::string>(my_list[i]["type"]);
      ROS_INFO("Using plugin \"%s\"", pname.c_str());

      boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);
      layered_costmap_->addPlugin(plugin);
      plugin->initialize(layered_costmap_, name + "/" + pname, &tf_);
    }
  }

  // subscribe to the footprint topic
  //订阅footprint话题，回调函数为setUnpaddedRobotFootprintPolygon。当话题上收到footprint时，回调函数会将接收到的footprint根据参数footprint_padding_的值进行“膨胀”，得到“膨胀”后的padded_footprint_，传递给各级地图。
  std::string topic_param, topic;
  if (!private_nh.searchParam("footprint_topic", topic_param))
  {
    topic_param = "footprint_topic";
  }

  private_nh.param(topic_param, topic, std::string("footprint"));
  footprint_sub_ = private_nh.subscribe(topic, 1, &Costmap2DROS::setUnpaddedRobotFootprintPolygon, this);
  //创建另一个话题，该话题上将发布的内容是根据机器人当前位置计算出来的实时footprint的位置。
  if (!private_nh.searchParam("published_footprint_topic", topic_param))
  {
    topic_param = "published_footprint";
  }

  private_nh.param(topic_param, topic, std::string("oriented_footprint"));
  footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);
  //主要回调函数setUnpaddedRobotFootprint（）将接收到的footprint根据参数footprint_padding_的值进行“膨胀”，得到“膨胀”后的padded_footprint_，传递给各级地图。
  setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh));
  //创建地图的发布器实例，Costmap2DPublisher类是用于地图发布的封装类。
  publisher_ = new Costmap2DPublisher(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap",
                                      always_send_full_costmap);

  // create a thread to handle updating the map创建地图更新线程的控制量
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a time r to check if the robot is moving创建一个timer去检测机器人是否在移动
  robot_stopped_ = false;
  //回调函数movementCB通过比较前后两个pose的差来判断机器人是否在移动
  timer_ = private_nh.createTimer(ros::Duration(.1), &Costmap2DROS::movementCB, this);

  //开启动态参数配置，它的回调函数Costmap2DROS::reconfigureCB会在节点启动时运行一次，加载.cfg文件的配置参数。这个函数给对应参数赋值，并创建了一个Costmap2DROS::mapUpdateLoop函数的线程，即地图更新线程。(下面有更新线程的函数定义，着重看一下)
  //开启参数动态配置
  dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/" + name));
  //回调函数reconfigureCB 除了对一些类成员的配置值做赋值以外，还会开启一个更新map的线程 
  dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = boost::bind(&Costmap2DROS::reconfigureCB, this, _1,
                                                                              _2);
  dsrv_->setCallback(cb);
}

    
//下面全是上面代码中用到的一些函数的定义
void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL)
    delete publisher_;

  delete layered_costmap_;
  delete dsrv_;
}

void Costmap2DROS::resetOldParameters(ros::NodeHandle& nh)
{
  ROS_INFO("Loading from pre-hydro parameter style");
  bool flag;
  std::string s;
  std::vector < XmlRpc::XmlRpcValue > plugins;

  XmlRpc::XmlRpcValue::ValueStruct map;
  SuperValue super_map;
  SuperValue super_array;

  if (nh.getParam("static_map", flag) && flag)
  {
    map["name"] = XmlRpc::XmlRpcValue("static_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::StaticLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    ros::NodeHandle map_layer(nh, "static_layer");
    move_parameter(nh, map_layer, "map_topic");
    move_parameter(nh, map_layer, "unknown_cost_value");
    move_parameter(nh, map_layer, "lethal_cost_threshold");
    move_parameter(nh, map_layer, "track_unknown_space", false);
  }

  ros::NodeHandle obstacles(nh, "obstacle_layer");
  if (nh.getParam("map_type", s) && s == "voxel")
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::VoxelLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    move_parameter(nh, obstacles, "origin_z");
    move_parameter(nh, obstacles, "z_resolution");
    move_parameter(nh, obstacles, "z_voxels");
    move_parameter(nh, obstacles, "mark_threshold");
    move_parameter(nh, obstacles, "unknown_threshold");
    move_parameter(nh, obstacles, "publish_voxel_map");
  }
  else
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::ObstacleLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);
  }

  move_parameter(nh, obstacles, "max_obstacle_height");
  move_parameter(nh, obstacles, "raytrace_range");
  move_parameter(nh, obstacles, "obstacle_range");
  move_parameter(nh, obstacles, "track_unknown_space", true);
  nh.param("observation_sources", s, std::string(""));
  std::stringstream ss(s);
  std::string source;
  while (ss >> source)
  {
    move_parameter(nh, obstacles, source);
  }
  move_parameter(nh, obstacles, "observation_sources");

  ros::NodeHandle inflation(nh, "inflation_layer");
  move_parameter(nh, inflation, "cost_scaling_factor");
  move_parameter(nh, inflation, "inflation_radius");
  map["name"] = XmlRpc::XmlRpcValue("inflation_layer");
  map["type"] = XmlRpc::XmlRpcValue("costmap_2d::InflationLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  super_array.setArray(&plugins);
  nh.setParam("plugins", super_array);
}

void Costmap2DROS::reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level)
{
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(config, old_config_);

  old_config_ = config;

  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

void Costmap2DROS::readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                                           const costmap_2d::Costmap2DConfig &old_config)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if (new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius)
  {
    return;
  }

  if (new_config.footprint != "" && new_config.footprint != "[]")
  {
    std::vector<geometry_msgs::Point> new_footprint;
    if (makeFootprintFromString(new_config.footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        ROS_ERROR("Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setUnpaddedRobotFootprint(makeFootprintFromRadius(new_config.robot_radius));
  }
}

void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::movementCB(const ros::TimerEvent &event)
{
  // don't allow configuration to happen while this check occurs
  // boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  tf::Stamped < tf::Pose > new_pose;

  if (!getRobotPose(new_pose))
  {
    ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else if (fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3
      && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3)
  {
    old_pose_ = new_pose;
    robot_stopped_ = true;
  }
  else
  {
    old_pose_ = new_pose;
    robot_stopped_ = false;
  }
}

//这个函数循环调用UpdateMap函数，更新地图。并以publish_cycle为周期，发布更新后的地图。
void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle只有当频率不为0的时候才更新线程
  if (frequency == 0.0)
    return;

  ros::NodeHandle nh;
  ros::Rate r(frequency);
  while (nh.ok() && !map_update_thread_shutdown_)
  {
    struct timeval start, end;//这里struct timeval是Linux系统中时间的一种定义，指的是当创建这个结构体时的时间距离Epoch时间是多少，结构体中包含了tv_sec（秒）和tv_usec（微秒）
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);

    updateMap();//按照给定的频率循环更新地图

    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;//计算出地图更新的时间
    ROS_DEBUG("Map update time: %.9f", t_diff);
    //更新地图边界及发布
    if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      ros::Time now = ros::Time::now();
      if (last_publish_ + publish_cycle < now)
      {
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
               r.cycleTime().toSec());
  }
}
    
//这个函数首先调用类内getRobotPose函数，通过tf转换，将机器人底盘系的坐标转换到global系，得到机器人的位姿。然后通过layered_costmap_调用LayeredCostmap类的updateMap函数，更新地图。
void Costmap2DROS::updateMap()
{
  if (!stop_updates_)
  {
    // get global pose将机器人底盘系的坐标转换到global系，得到机器人的位姿
    tf::Stamped < tf::Pose > pose;
    if (getRobotPose (pose))
    {
      double x = pose.getOrigin().x(),
             y = pose.getOrigin().y(),
             yaw = tf::getYaw(pose.getRotation());
      //通过layered_costmap_调用LayeredCostmap类的updateMap函数，更新地图。
      layered_costmap_->updateMap(x, y, yaw);
	  //更新机器人的实时足迹，通过footprint_pub_发布。
      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = ros::Time::now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      footprint_pub_.publish(footprint);

      initialized_ = true;
    }
  }
}

//start函数在Movebase中被调用，这个函数对各层地图插件调用activate函数，激活各层地图。
void Costmap2DROS::start()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused//检查地图是否暂停或者停止
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    //如果停止，需要重新订阅话题
    //“Layer”是一个类，active是其中一个类方法
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (ros::ok() && !initialized_)
    r.sleep();
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    tf_.transformPose(global_frame_, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const
{
  tf::Stamped<tf::Pose> global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf::getYaw(global_pose.getRotation());
  transformFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw,
                     padded_footprint_, oriented_footprint);
}

}  // namespace costmap_2d
```

##  3.costmap_2d.h

```C++
#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

namespace costmap_2d
{

// convenient for storing x/y point pairs
struct MapLocation
{
  unsigned int x;
  unsigned int y;
};

/**
 * @class Costmap2D
 * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
 */
class Costmap2D
{
  friend class CostmapTester;  // Need this for gtest to work correctly
public:
  /**
   * @brief  Constructor for a costmap
   * @param  cells_size_x The x size of the map in cells栅格地图的x尺寸,表示x方向最大距离（x方向像素点个数）
   * @param  cells_size_y The y size of the map in cells栅格地图的y尺寸,表示y方向最大距离（y方向像素点个数）
   * @param  resolution The resolution of the map in meters/cell地图的分辨率（栅格点之间的距离）
   * @param  origin_x The x origin of the map地图原点的x位置（地图的下标计数原点）
   * @param  origin_y The y origin of the map地图原点的y位置
   * @param  default_value Default Value
   */
  Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value = 0);

  /**
   * @brief  Copy constructor for a costmap, creates a copy efficiently
   * @param map The costmap to copy
   */
  Costmap2D(const Costmap2D& map);

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  Costmap2D& operator=(const Costmap2D& map);

  /**
   * @brief  Turn this costmap into a copy of a window of a costmap passed in
   * @param  map The costmap to copy
   * @param win_origin_x The x origin (lower left corner) for the window to copy, in meters
   * @param win_origin_y The y origin (lower left corner) for the window to copy, in meters
   * @param win_size_x The x size of the window, in meters
   * @param win_size_y The y size of the window, in meters
   */
  bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                         double win_size_y);

  /**
   * @brief  Default constructor
   */
  Costmap2D();

  /**
   * @brief  Destructor
   */
  virtual ~Costmap2D();

  /**
   * @brief  Get the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @return The cost of the cell
   */
  unsigned char getCost(unsigned int mx, unsigned int my) const;

  /**
   * @brief  Set the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @param cost The cost to set the cell to
   */
  void setCost(unsigned int mx, unsigned int my, unsigned char cost);

  /**
   * @brief  Convert from map coordinates to world coordinates
   * @param  mx The x map coordinate
   * @param  my The y map coordinate
   * @param  wx Will be set to the associated world x coordinate
   * @param  wy Will be set to the associated world y coordinate
   */
  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false otherwise
   */
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates without checking for legal bounds
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates <b>are not guaranteed to lie within the map.</b>
   */
  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;

  /**
   * @brief  Convert from world coordinates to map coordinates, constraining results to legal bounds.
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  mx Will be set to the associated map x coordinate
   * @param  my Will be set to the associated map y coordinate
   * @note   The returned map coordinates are guaranteed to lie within the map.
   */
  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;

  /**
   * @brief  Given two map coordinates... compute the associated index
   * @param mx The x coordinate
   * @param my The y coordinate
   * @return The associated index
   */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * size_x_ + mx;
  }

  /**
   * @brief  Given an index... compute the associated map coordinates
   * @param  index The index
   * @param  mx Will be set to the x coordinate
   * @param  my Will be set to the y coordinate
   */
  inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
  {
    my = index / size_x_;
    mx = index - (my * size_x_);
  }

  /**
   * @brief  Will return a pointer to the underlying unsigned char array used as the costmap
   * @return A pointer to the underlying unsigned char array storing cost values
   */
  unsigned char* getCharMap() const;

  /**
   * @brief  Accessor for the x size of the costmap in cells
   * @return The x size of the costmap
   */
  unsigned int getSizeInCellsX() const;

  /**
   * @brief  Accessor for the y size of the costmap in cells
   * @return The y size of the costmap
   */
  unsigned int getSizeInCellsY() const;

  /**
   * @brief  Accessor for the x size of the costmap in meters
   * @return The x size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  double getSizeInMetersX() const;

  /**
   * @brief  Accessor for the y size of the costmap in meters
   * @return The y size of the costmap (returns the centerpoint of the last legal cell in the map)
   */
  double getSizeInMetersY() const;

  /**
   * @brief  Accessor for the x origin of the costmap
   * @return The x origin of the costmap
   */
  double getOriginX() const;

  /**
   * @brief  Accessor for the y origin of the costmap
   * @return The y origin of the costmap
   */
  double getOriginY() const;

  /**
   * @brief  Accessor for the resolution of the costmap
   * @return The resolution of the costmap
   */
  double getResolution() const;

  void setDefaultValue(unsigned char c)
  {
    default_value_ = c;
  }

  unsigned char getDefaultValue()
  {
    return default_value_;
  }

  /**
   * @brief  Sets the cost of a convex polygon to a desired value
   * @param polygon The polygon to perform the operation on
   * @param cost_value The value to set costs to
   * @return True if the polygon was filled... false if it could not be filled
   */
  bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);

  /**
   * @brief  Get the map cells that make up the outline of a polygon
   * @param polygon The polygon in map coordinates to rasterize
   * @param polygon_cells Will be set to the cells contained in the outline of the polygon
   */
  void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

  /**
   * @brief  Get the map cells that fill a convex polygon
   * @param polygon The polygon in map coordinates to rasterize
   * @param polygon_cells Will be set to the cells that fill the polygon
   */
  void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);

  /**
   * @brief  Move the origin of the costmap to a new location.... keeping data when it can
   * @param  new_origin_x The x coordinate of the new origin
   * @param  new_origin_y The y coordinate of the new origin
   */
  virtual void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief  Save the costmap out to a pgm file
   * @param file_name The name of the file to save
   */
  bool saveMap(std::string file_name);

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  /**
   * @brief  Given distance in the world... convert it to cells
   * @param  world_dist The world distance
   * @return The equivalent cell distance
   */
  unsigned int cellDistance(double world_dist);

  // Provide a typedef to ease future code maintenance
  typedef boost::recursive_mutex mutex_t;
  mutex_t* getMutex()
  {
    return access_;
  }

protected:
  /**
   * @brief  Copy a region of a source map into a destination map
   * @param  source_map The source map
   * @param sm_lower_left_x The lower left x point of the source map to start the copy
   * @param sm_lower_left_y The lower left y point of the source map to start the copy
   * @param sm_size_x The x size of the source map
   * @param  dest_map The destination map
   * @param dm_lower_left_x The lower left x point of the destination map to start the copy
   * @param dm_lower_left_y The lower left y point of the destination map to start the copy
   * @param dm_size_x The x size of the destination map
   * @param region_size_x The x size of the region to copy
   * @param region_size_y The y size of the region to copy
   */
  template<typename data_type>
    void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y)
    {
      // we'll first need to compute the starting points for each map
      data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
      data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

      // now, we'll copy the source map into the destination map
      for (unsigned int i = 0; i < region_size_y; ++i)
      {
        memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
        sm_index += sm_size_x;
        dm_index += dm_size_x;
      }
    }

  /**
   * @brief  Deletes the costmap, static_map, and markers data structures
   */
  virtual void deleteMaps();

  /**
   * @brief  Resets the costmap and static_map to be unknown space
   */
  virtual void resetMaps();

  /**
   * @brief  Initializes the costmap, static_map, and markers data structures
   * @param size_x The x size to use for map initialization
   * @param size_y The y size to use for map initialization
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  /**
   * @brief  Raytrace a line and apply some action at each step
   * @param  at The action to take... a functor
   * @param  x0 The starting x coordinate
   * @param  y0 The starting y coordinate
   * @param  x1 The ending x coordinate
   * @param  y1 The ending y coordinate
   * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
   */
  template<class ActionType>
    inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                             unsigned int max_length = UINT_MAX)//对于离散的平面点，指定两个点，这个函数可以找到两个点之间的其他点，使得这些中间组成一个尽可能趋近直线的点集。
    {
      int dx = x1 - x0;
      int dy = y1 - y0;

      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);

      int offset_dx = sign(dx);
      int offset_dy = sign(dy) * size_x_;

      unsigned int offset = y0 * size_x_ + x0;

      // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
      double dist = hypot(dx, dy);
      double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

      // if x is dominant
      if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
      }

      // otherwise y is dominant
      int error_x = abs_dy / 2;
      bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
    }

private:
  /**
   * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
   */
  template<class ActionType>
    inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                            int offset_b, unsigned int offset, unsigned int max_length)
    {
      unsigned int end = std::min(max_length, abs_da);
      for (unsigned int i = 0; i < end; ++i)
      {
        at(offset);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
          offset += offset_b;
          error_b -= abs_da;
        }
      }
      at(offset);
    }

  inline int sign(int x)
  {
    return x > 0 ? 1.0 : -1.0;
  }

  mutex_t* access_;
protected:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  unsigned char* costmap_;//地图数据
  unsigned char default_value_;

  class MarkCell
  {
  public:
    MarkCell(unsigned char* costmap, unsigned char value) :
        costmap_(costmap), value_(value)
    {
    }
    inline void operator()(unsigned int offset)
    {
      costmap_[offset] = value_;
    }
  private:
    unsigned char* costmap_;
    unsigned char value_;
  };

  class PolygonOutlineCells
  {
  public:
    PolygonOutlineCells(const Costmap2D& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) :
        costmap_(costmap), char_map_(char_map), cells_(cells)
    {
    }

    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset)
    {
      MapLocation loc;
      costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const Costmap2D& costmap_;
    const unsigned char* char_map_;
    std::vector<MapLocation>& cells_;
  };
};
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_H
```

##  4.costmap_2d.cpp

```C++
#include <costmap_2d/costmap_2d.h>
#include <cstdio>

using namespace std;

namespace costmap_2d
{
//Costmap2D类是记录地图数据的底层，它记录地图的x、y方向的尺寸，地图的分辨率（在实际地图中一米所代表的单元格数目），地图原点位置，以及用unsigned char类型记录地图内容。Costmap2D类提供了一些对地图进行基本操作的函数，如：地图复制、用index/点坐标来设置/获取地图上该点的cost值、地图坐标和世界坐标之间的转换、获取地图大小/分辨率/原点、设置多边形边缘及内部点的cost值。
    //设置多边形cost Costmap2D::setConvexPolygonCost()它主要是用来设置机器人足迹内cell的cost(着重看下)
Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                     double origin_x, double origin_y, unsigned char default_value) :
    size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
    origin_y_(origin_y), costmap_(NULL), default_value_(default_value)//传入地图边长，地图的原点位置，分辨率，以及设定代价值
{
  access_ = new mutex_t();

  // create the costmap
  initMaps(size_x_, size_y_);//重新分配size_x*size_y 空间，并设置默认值
  resetMaps();
}

void Costmap2D::deleteMaps()
{
  // clean up data
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                          double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  resetMaps();
}

void Costmap2D::resetMaps()
{
  boost::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  boost::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
    memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}
//作用：将代价地图作为一个地图窗口副本放入地图
bool Costmap2D::copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                                  double win_size_y)
{
  // check for self windowing
  if (this == &map)
  {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  // clean up old data
  deleteMaps();

  // compute the bounds of our new map
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y)
      || !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y))
  {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }

  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  resolution_ = map.resolution_;
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  // initialize our various maps and reset markers for inflation
  initMaps(size_x_, size_y_);

  // copy the window of the static map and the costmap that we're taking
  copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
  return true;
}

Costmap2D& Costmap2D::operator=(const Costmap2D& map)
{
  // check for self assignement
  if (this == &map)
    return *this;

  // clean up old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // initialize our various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

Costmap2D::Costmap2D(const Costmap2D& map) :
    costmap_(NULL)
{
  access_ = new mutex_t();
  *this = map;
}

// just initialize everything to NULL by default
Costmap2D::Costmap2D() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new mutex_t();
}

Costmap2D::~Costmap2D()
{
  deleteMaps();
  delete access_;
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
  double cells_dist = max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}
    
//返回costmap_地图指针必须要分清楚该指针是指向master costmap,还是其他分层的costmap
unsigned char* Costmap2D::getCharMap() const
{
  return costmap_;
}

//获取cosmap地图某个点(单元)的cost值
unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}

//设置cosmap地图某个点（单元）的cost值
void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

//+0.5 为了去除边界，防止计算到地图外面，(这里map 是指像素坐标系，单位为int，world 是指/map世界坐标系)
//作用：从地图坐标（mx, my）向世界坐标(wx, wy)转换,一定会转换成功，地图坐标系小于世界坐标系
void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}
//作用：从世界坐标（wx, wy）向地图坐标(mx, my)转换,不一定会转换成功，地图坐标系小于世界坐标系，如果转换成功（合法界限），则为True；否则为false
bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;

  return false;
}

//作用：从世界坐标（wx, wy）向地图坐标(mx, my)转换,地图没有边界
void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const
{
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);
}

//作用：从世界坐标（wx, wy）向地图坐标(mx, my)转换,地图有边界，从世界坐标转换为地图坐标，将结果约束为合法边界，返回的地图坐标保证位于地图内。
void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_)
  {
    mx = 0;
  }
  else if (wx >= resolution_ * size_x_ + origin_x_)
  {
    mx = size_x_ - 1;
  }
  else
  {
    mx = (int)((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_)
  {
    my = 0;
  }
  else if (wy >= resolution_ * size_y_ + origin_y_)
  {
    my = size_y_ - 1;
  }
  else
  {
    my = (int)((wy - origin_y_) / resolution_);
  }
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = min(max(cell_ox, 0), size_x);
  lower_left_y = min(max(cell_oy, 0), size_y);
  upper_right_x = min(max(cell_ox + size_x, 0), size_x);
  upper_right_y = min(max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  // make sure to clean up
  delete[] local_map;
}

//先将世界系下的多边形顶点转换到地图坐标系，并存放进map_polygon数组中
bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value)
{
  // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i)
  {
    MapLocation loc;
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
    {
      // ("Polygon lies outside map bounds, so we can't fill it");
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<MapLocation> polygon_cells;

  // get the cells that fill the polygon
  //接着调用convexFillCells函数，通过机器人顶点坐标数组map_polygon得到多边形边缘及内部的全部cell，存放在polygon_cells中，并通过循环对多边形边缘及内部各cell的cost赋值。
  convexFillCells(map_polygon, polygon_cells);

  // set the cost of those cells设置这些单元格的代价值
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

//这个函数循环调用raytraceLine()函数，不断获取相邻单元格中点之间的连线，最终组成多边形边上的cell，需要注意的是需要将最后一点和第一点连接起来，形成闭合。
void Costmap2D::polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  for (unsigned int i = 0; i < polygon.size() - 1; ++i)
  {
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty())//将最后一点和起点连接起来
  {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}

//获取多边形边缘及内部函数cell Costmap2D::convexFillCells（），首先确保给定的多边形顶点不少于3个，接着调用类内polygonOutlineCells函数，通过给定的顶点提取多边形边上的cell。
void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  // we need a minimum polygon of a triangle//多边形的边数不能小于3
  if (polygon.size() < 3)
    return;

  // first get the cells that make up the outline of the polygon首先获得轮廓点之间连线的列表，存放在polygon_cells中
  polygonOutlineCells(polygon, polygon_cells);

  // quick bubble sort to sort points by x 对边上的cell点的x做排序，使其按x坐标升序排列。
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1)
  {
    if (polygon_cells[i].x > polygon_cells[i + 1].x)
    {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
        --i;
    }
    else
      ++i;
  }
  //遍历所有x，对每个相同的x，检查y，获得y最大和最小的polygon cell，将范围内的所有cell填充进polygon_cells，获得多边形边缘及内部的所有cell。
  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  // walk through each column and mark cells inside the polygon
  for (unsigned int x = min_x; x <= max_x; ++x)
  {
    if (i >= polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].y < polygon_cells[i + 1].y)
    {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    }
    else
    {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x)
    {
      if (polygon_cells[i].y < min_pt.y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].y > max_pt.y)
        max_pt = polygon_cells[i];
      ++i;
    }

    MapLocation pt;
    // loop though cells in the column
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
    {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}

//作用：使用米为单位进行x、y边长计算
double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

//作用：获取原点的位置（x, y）
double Costmap2D::getOriginX() const
{
  return origin_x_;
}

double Costmap2D::getOriginY() const
{
  return origin_y_;
}

//作用：获取地图的分辨率，就是按照什么样的规格进行对x y边长进行刻度    
double Costmap2D::getResolution() const
{
  return resolution_;
}

bool Costmap2D::saveMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++)
  {
    for (unsigned int ix = 0; ix < size_x_; ix++)
    {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

}  // namespace costmap_2d
```

* ***1-4我所参考的博文是这篇https://blog.csdn.net/Neo11111/article/details/104798065***

*******************************************

##  5.layered_costmap.h

```C++
#ifndef COSTMAP_2D_LAYERED_COSTMAP_H_
#define COSTMAP_2D_LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>

namespace costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 实例化不同的层插件并将它们聚合成一个master_map
 */
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  bool isCurrent();

  Costmap2D* getCostmap()
  {
    return &costmap_;
  }

  bool isRolling()
  {
    return rolling_window_;
  }

  bool isTrackingUnknown()
  {
    return costmap_.getDefaultValue() == costmap_2d::NO_INFORMATION;
  }

  std::vector<boost::shared_ptr<Layer> >* getPlugins()
  {
    return &plugins_;
  }

  void addPlugin(boost::shared_ptr<Layer> plugin)
  {
    plugins_.push_back(plugin);
  }

  bool isSizeLocked()
  {
    return size_locked_;
  }

  void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  bool isInitialized()
  {
      return initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getCircumscribedRadius() { return circumscribed_radius_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getInscribedRadius() { return inscribed_radius_; }

private:
  Costmap2D costmap_;
  std::string global_frame_;

  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

  bool current_;
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_;

  std::vector<boost::shared_ptr<Layer> > plugins_;

  bool initialized_;
  bool size_locked_;
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::Point> footprint_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYERED_COSTMAP_H_

```

##  6.layered_costmap.cpp

```C++
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{
LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
//调用costmap_ 的setDefaultValue 方法，实际上设定了类costmap_2D 的一个成员变量default_value_ 这个值在class costmap_2D 中的resetMaps函数中被调用，是这样使用的：memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char)); 实际存储地图的变量就是class costmap_2D 的 costmap_ 数据成员。
  if (track_unknown)
    costmap_.setDefaultValue(255);
  else
    costmap_.setDefaultValue(0);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}
//函数LayeredCostmap::resizeMap() 就是给class costmap_2D 的 costmap_ 成员的大小重新做分配。然后根据plugin对每一层的地图调用其父类Costmap2D成员的initial 方法，实际效果就是将plugin所指向的每一层地图的大小都设置为和LayeredCostmap::costmap_ 数据成员一样的空间大小。
void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
}
    
//函数 LayeredCostmap::updateMap() 完成对每一层地图的更新，更新过程分为两步updateBounds和updateCosts：
void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.将一些线程进行时不安全的插件锁上
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;
//更新Bounds过程由于传入的参数是&minx_, &miny_, &maxx_, &maxy_ 构成了一个矩形范围。由于针对不同的类的实例(指的是不同的地图层)，调用不同的类的方法即(*plugin)->updateBounds()
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }//如果每层地图更新的边界
  }

  int x0, xn, y0, yn;
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
    return;

  costmap_.resetMap(x0, y0, xn, yn);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d

```

