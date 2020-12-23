#  Navigation Move_Base代码阅读

## 结构示意图       

![move_base 流程图](https://img-blog.csdnimg.cn/20200301020626758.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L05lbzExMTEx,size_16,color_FFFFFF,t_70)

![move_base结构图](https://imgconvert.csdnimg.cn/aHR0cHM6Ly9pbWFnZXMyMDE4LmNuYmxvZ3MuY29tL2Jsb2cvMTIyMDA5My8yMDE4MDUvMTIyMDA5My0yMDE4MDUyNDA5MTA1NDUwMC0xNzYyNzc5OTU3LnBuZw?x-oss-process=image/format,png)

##  代码分析

###  1. move_base_node.cpp

这里初始化了节点“move_base_node”，Action服务的定义、全局规划器、局部规划器的调用都将在这个节点中进行。然后实例化了MoveBase这个类，既`move_base(tf)`，上述工作以类成员函数的形式定义在这个类中。实例化之后，Action开始监听服务请求，并通过`ros::spin()`传递到Action的回调函数中（`MoveBase::executeCb()`）进行处理。

###  2. move_base.h

#### <u>***这里对MoveBase类的类成员进行了声明，以下为比较重要的几个类成员函数。***</u>

1. 构造函数 MoveBase::MoveBase | 初始化Action
2. 控制主体 MoveBase::executeCb | 收到目标，触发全局规划线程，循环执行局部规划
3. 全局规划线程 void MoveBase::planThread | 调用全局规划
4. 全局规划 MoveBase::makePlan | 调用全局规划器类方法，得到全局规划路线
5. 局部规划 MoveBase::executeCycle | 传入全局路线，调用局部规划器类方法，得到速度控制指令

>  * 构造函数，传入的参数是tf
>
>  ```c++
>  MoveBase(tf2_ros::Buffer& tf);
>  ```
>
>  * 析构函数
>
>  ```c++
>  virtual ~MoveBase();
>  ```
>
>  * 控制闭环、全局规划、 到达目标返回true，没有到达返回false
>
>  ```c++
>  bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);
>  ```
>
>  * 清除costmap
>
>  ```c++
>  bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
>  ```
>
>  * 当action不活跃时，调用此函数，返回plan
>
>  ```c++
>  bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
>  ```
>
>  * 新的全局规划，goal 规划的目标点，plan 规划
>
>  ```c++
>  bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
>  ```
>
>  * 从参数服务器加载导航恢复行为
>
>  ```c++
>  bool loadRecoveryBehaviors(ros::NodeHandle node);
>  ```
>
>  * 加载默认导航恢复行为
>
>  ```c++
>  void loadDefaultRecoveryBehaviors();
>  ```
>
>  * 清除机器人局部规划框的障碍物，size_x 局部规划框的长x， size_y 局部规划框的宽y
>
>  ```c++
>  void clearCostmapWindows(double size_x, double size_y);
>  ```
>
>  * 发布速度为0的指令
>
>  ```c++
>  void publishZeroVelocity();
>  ```
>
>  * 重置move_base action的状态，设置速度为0
>
>  ```c++
>  void resetState();
>  ```
>
>  * 周期性地唤醒规划器
>
>  ```c++
>  void wakePlanner(const ros::TimerEvent& event);
>  ```
>
>  * 其它函数
>
>  ```c++
>  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
>  
>  void planThread();
>  
>  void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
>  
>  bool isQuaternionValid(const geometry_msgs::Quaternion& q);
>  
>  bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);
>  
>  double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
>  
>  geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
>  ```
>
>  * 数据成员
>
>  ```c++     
>  tf::TransformListener& tf_;
>  
>  MoveBaseActionServer* as_; //就是actionlib的server端
>  
>  boost::shared_ptr&lt;nav_core::BaseLocalPlanner&gt; tc_;//局部规划器，加载并创建实例后的指针
>  costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;//costmap的实例化指针
>  
>  boost::shared_ptr&lt;nav_core::BaseGlobalPlanner&gt; planner_;//全局规划器，加载并创建实例后的指针
>  std::string robot_base_frame_, global_frame_;
>  
>  std::vector&lt;boost::shared_ptr&lt;nav_core::RecoveryBehavior&gt; &gt; recovery_behaviors_;//可能是出错后的恢复
>  unsigned int recovery_index_;
>  
>  geometry_msgs::PoseStamped global_pose_;
>  double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
>  double planner_patience_, controller_patience_;
>  int32_t max_planning_retries_;
>  uint32_t planning_retries_;
>  double conservative_reset_dist_, clearing_radius_;
>  ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
>  ros::Subscriber goal_sub_;
>  ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
>  bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
>  double oscillation_timeout_, oscillation_distance_;
>  
>  MoveBaseState state_;
>  RecoveryTrigger recovery_trigger_;
>  
>  ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
>  geometry_msgs::PoseStamped oscillation_pose_;
>  pluginlib::ClassLoader&lt;nav_core::BaseGlobalPlanner&gt; bgp_loader_;
>  pluginlib::ClassLoader&lt;nav_core::BaseLocalPlanner&gt; blp_loader_;
>  pluginlib::ClassLoader&lt;nav_core::RecoveryBehavior&gt; recovery_loader_;
>  
>  //触发哪种规划器
>  std::vector&lt;geometry_msgs::PoseStamped&gt;* planner_plan_;//保存最新规划的路径，传给latest_plan_
>  std::vector&lt;geometry_msgs::PoseStamped&gt;* latest_plan_;//在executeCycle中传给controller_plan_
>  std::vector&lt;geometry_msgs::PoseStamped&gt;* controller_plan_;
>  
>  //规划器线程
>  bool runPlanner_;
>  boost::recursive_mutex planner_mutex_;
>  boost::condition_variable_any planner_cond_;
>  geometry_msgs::PoseStamped planner_goal_;
>  boost::thread* planner_thread_;
>  
>  
>  boost::recursive_mutex configuration_mutex_;
>  dynamic_reconfigure::Server&lt;move_base::MoveBaseConfig&gt; *dsrv_;
>  
>  void reconfigureCB(move_base::MoveBaseConfig &amp;config, uint32_t level);
>  
>  move_base::MoveBaseConfig last_config_;
>  move_base::MoveBaseConfig default_config_;
>  bool setup_, p_freq_change_, c_freq_change_;
>  bool new_global_plan_;<<
>  ```

##  3. move_base.cpp

#### <u>***文件move_base.cpp定义了上面头文件里写的函数，入口是构造函数MoveBase::MoveBase***</u>

首先，初始化了一堆参数：

```c++
tf_(tf), //tf::TransformListener&  引用？取址？(这里是引用吧）
as_(NULL), //MoveBaseActionServer* 指针
planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),//costmap的实例化指针
bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), //nav_core::BaseGlobalPlanner类型的插件（全局规划器）
blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),//nav_core::BaseLocalPlanner类型的插件（局部规划器）
recovery_loader_("nav_core", "nav_core::RecoveryBehavior"), //nav_core::RecoveryBehavior类型的插件
planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),//三种规划器，看触发哪种
runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), //配置参数
new_global_plan_(false)  //配置参数
```

创建move_base action,绑定回调函数。定义一个名为move_base的SimpleActionServer。该服务器的Callback为moveBase::executeCb

当执行as_->start()时，调用MoveBase::executeCb函数

```c++
as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
```

executeCb是Action的回调函数，它是MoveBase控制流的主体，它调用了MoveBase内另外几个作为子部分的重要成员函数，先后完成了全局规划和局部规划。在函数的开始部分，它对Action收到的目标进行四元数检测、坐标系转换，然后将其设置为全局规划的目标，并设置了一些标志位。

调用回调函数```void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal){}```，具体如下：

如果目标非法，则直接返回：

```c++
if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
	as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid 	quaternion");
	return;
}
```

其中，isQuaternionValid(const geometry_msgs::Quaternion& q)函数如下，主要用于检查四元数的合法性：

```c++
bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //1、首先检查四元数是否元素完整（这里检查四元数中是否有哪个数是无限的，有的话则为无效四元数）
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
        ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
        return false;
    }
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    //2、检查四元数是否趋近于0
    if(tf_q.length2() < 1e-6){
        ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
        return false;
    }
    //3、对四元数规范化，转化为vector
    tf_q.normalize();
    tf::Vector3 up(0, 0, 1);
    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));
    if(fabs(dot - 1) > 1e-3){
        ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");//z轴如果不接近垂直，说明这个四元数也是无效的
        return false;
    }

    return true;
}
```

将目标的坐标系统一转换为全局坐标系：

```c++
geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
```

其中，函数```geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){}```如下：

```c++
geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }
```

现在设置目标点并唤醒路径规划线程：

```c++
boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);//可以理解这里使用互斥锁将当前的全局规划线程给锁住，保证线程正常的运行
planner_goal_ = goal;//用接收到的目标goal来更新全局变量，即全局规划目标，这个值在planThread中会被用来做全局规划的当前目标
runPlanner_ = true;//全局规划标志位设为真
planner_cond_.notify_one();//开始全局规划并于此处阻塞
lock.unlock();//解锁
```

然后发布goal，设置控制频率：

```c++
current_goal_pub_.publish(goal);//全局规划完成后，发布目标到current_goal话题上
std::vector<geometry_msgs::PoseStamped> global_plan;//创建一个全局规划容器
ros::Rate r(controller_frequency_);//局部规划频率(这里指的是在全局规划时控制规划时的频率)
```

开启costmap更新：

```c++
if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }
```

重置时间标志位：

```c++
last_valid_control_ = ros::Time::now();//上一次有效的局部规划时间设为现在
last_valid_plan_ = ros::Time::now();//上一次有效的全局规划时间设为现在
last_oscillation_reset_ = ros::Time::now();//上一次震荡重置时间设为现在
planning_retries_ = 0;
```



