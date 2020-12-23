#  clear_costmap_recovery包代码解读

* 该包为导航堆栈提供了一种恢复行为，其尝试通过将导航堆栈使用的代价地图恢复到给定区域外的静态地图来清除空间（会将global_costmap和local_costmap中给定半径（`reset_distance`默认值3.0）范围之外的区域进行清理，**即将栅格占有或者非占有状态清除为未知**）。`clear_costmap_recovery::ClearCostmapRecovery`是一种简单的恢复行为，其继承了nav_core包中接口nav_core::RecoveryBehavior ，以插件方式在move_base node中使用。

##  clear_costmap_recovery.h

```C++
#ifndef CLEAR_COSTMAP_RECOVERY_H_
#define CLEAR_COSTMAP_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>

namespace clear_costmap_recovery{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class ClearCostmapRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
      ClearCostmapRecovery();

      /**
       * @brief  Initialization function for the ClearCostmapRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      void initialize(std::string name, tf::TransformListener* tf, 
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the ClearCostmapRecovery recovery behavior. Reverts the
       * costmap to the static map outside of a user-specified window and
       * clears unknown space around the robot.
       */
      void runBehavior();

    private:
      void clear(costmap_2d::Costmap2DROS* costmap);      
      void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double reset_distance_;//给定的半径，机器人将清除在全局和局部代价地图上此半径之外的地图，使其变为静态地图
      std::set<std::string> clearable_layers_; ///< Layer names which will be cleared.
  };
};
#endif  
```

> ***对见到的set容器进行一个简单的了解：`std::set<std::string>`是一个关联容器（经常用到的关联容器有map，关联容器和顺序容器有着根本的不同：<u>关联容器中的元素是按关键字来保存和访问的。与之相对，顺序容器中的元素是按它们在容器中的位置来顺序保存和访问的。</u>关联容器不支持顺序容器的位置相关的操作。原因是<u>关联容器中元素是根据关键字存储的</u>，这些操作对关联容器没有意义。而且<u>关联容器也不支持构造函数或插入操作这些接受一个元素值和一个数量值得操作。</u>）***

> ***set(关键字即值，即只保存关键字的容器)，map是关键字----值对的集合，与之相对，<u>set就是关键字的简单集合。</u>当只是想知道一个值是否存在时，set是最有用的。<u>在set中每个元素的值都唯一，而且系统能根据元素的值自动进行排序。</u>Set中元素的值不能直接被改变。set内部采用的是一种非常高效的平衡检索二叉树：红黑树，也称为RB树(Red-Black Tree)。RB树的统计性能要好于一般平衡二叉树。***

##  clear_costmap_recovery.cpp

```C++
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;//在下面进行了一些解释，是表示代价值

namespace clear_costmap_recovery {
ClearCostmapRecovery::ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false) {} 

void ClearCostmapRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){//初始化函数
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);//半径默认为3m
    
    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacles") );//默认清理的就是代价地图中的障碍物层
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);

    for(unsigned i=0; i < clearable_layers.size(); i++) {
        ROS_INFO("Recovery behavior will clear layer %s", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);//将需要清除的哪层地图放入到clearable_layers_中，默认清楚的是障碍物层
    }


    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void ClearCostmapRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
  clear(global_costmap_);//在全局地图中按照规定的半径，清理之外的地图
  clear(local_costmap_);//在局部地图中按照规定的半径，清理之外的地图
}

//下面是具体的清理函数，其中ClearCostmapRecovery::clear()函数调用了ClearCostmapRecovery::clearMap()函数
void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  tf::Stamped<tf::Pose> pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.getOrigin().x();
  double y = pose.getOrigin().y();

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');//反向查找字符串‘/’，返回字符串的下表
    if( slash != std::string::npos ){//如果找到了将待清除地图的名字分别取出来
        name = name.substr(slash+1);
    }

    if(clearable_layers_.count(name)!=0){
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);//真正的清除操作
    }
  }
}

//清除地图的核心函数
void ClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, 
                                        double pose_x, double pose_y){
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
 
  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  unsigned char* grid = costmap->getCharMap();
  for(int x=0; x<(int)costmap->getSizeInCellsX(); x++){
    bool xrange = x>start_x && x<end_x;
                   
    for(int y=0; y<(int)costmap->getSizeInCellsY(); y++){
      if(xrange && y>start_y && y<end_y)
        continue;
      int index = costmap->getIndex(x,y);
      if(grid[index]!=NO_INFORMATION){//将栅格更新为没有信息
        grid[index] = NO_INFORMATION;
      }
    }
  }

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

};
```

* 一些知识点的补充


>ROS的代价地图（costmap）采用网格（grid）形式，每个网格的值（cell  cost）从0~255。分成三种状态：被占用（有障碍）、自由区域（无障碍）、未知区域；以激光雷达为传感器（或者kinect之类的深度相机的伪激光雷达），根据激光测量的障碍物距离机器人中心的距离，结合机器人的内切和外切半径，搞一个映射，利用bresenham算法（计算方法参考https://www.cnblogs.com/zjiaxing/p/5543386.html）可以填充由激光雷达的位置到障碍物之间的栅格概率了。

> 虽然代价地图中每个cell可用255个不同值中任何一个值，可是下层数据结构仅需要3个值。  具体来说在这种下层结构中，每个cell仅需要3个值来表示cell的3种状态：free，occupied，unknown。  当投影到代价地图时候，每种状态被赋一个特定的代价值，也就是说每个cell的cost值是由这个cell对应的各层中对应的cell的状态进行加权得到的。 如果列有一定量的占用就被赋代价值**costmap_2d::LETHAL_OBSTACLE**， 如果列有一定量的unknown cells 就被赋代价值**costmap_2d::NO_INFORMATION**, 剩余其它列赋代价值为**costmap_2d::FREE_SPACE**

>  boost::shared_ptr是可以**共享所有权**的指针。如果有多个shared_ptr共同管理同一个对象时，只有这些shared_ptr全部与该对象脱离关系之后，被管理的对象才会被释放。（具体的实现看一下https://blog.csdn.net/dddxxxx/article/details/87929470）

