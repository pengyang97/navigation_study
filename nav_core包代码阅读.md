#  nav_core包解读

* 打开nav_core包可以看到里面是这么个样子![](/home/pengyang/图片/nav_core包截图.png)

**package.xml**说这个包提供了三个导航用到的，机器人特定动作的一般接口，分别是`BaseGlobalPlanner`,`BaseLocalPlanner`,`RecoveryBehavior`,即全局规划器，局部规划器和恢复行为规划器，接口的作用一般是为了统一不同规划器的输出、输入，使得后续程序可以适应不同的规划器。

**CMakeList.txt**其中`catkin_package`的作用是，当其他包`find_package(nav_core)`时，应该迭代依赖`std_msgs`,`geometry_msgs`,`tf`,`costmap_2d`四个包。

**include**文件夹包含了三个文件：`base_global_planner.h`,`base_local_planner.h`,`recovery_behavior.h`，这三个包分别定义了三个纯虚类，如图所示：

![](/home/pengyang/图片/nav_core结构.png)

##  1.base_global_planner.h

```C++
#ifndef NAV_CORE_BASE_GLOBAL_PLANNER_H
#define NAV_CORE_BASE_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace nav_core {
  /**
   * @class BaseGlobalPlanner
   * @brief Provides an interface for global planners used in navigation. All global planners written as plugins for the navigation stack must adhere to this interface.
   */
  class BaseGlobalPlanner{
    public:
      /**
       * @brief Given a goal pose in the world, compute a plan
       	  	给个目标点，规划个路径
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       		给了三个参数，如果找到了可行的路径，返回真，否则返回false
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @param cost The plans calculated cost
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost)
      {
        cost = 0;
        return makePlan(start, goal, plan);
      }

      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       		全局规划器的初始化函数
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       		两个参数分别是:name,规划器的名字;costmap_ros,指向规划所使用的指向costmap的指针
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseGlobalPlanner(){}

    protected:
      BaseGlobalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_GLOBAL_PLANNER_H
```

​		声明了一个纯虚类，用户在实现全局规划器时需要继承自该类，并给出这几个函数的具体实现。其中`makePlan`函数在`move_base`的`void planThread()`中被调用，用于根据参数一（起始点）以及参数二（终点）来规划路径，并通过引用的形式返回给参数三。

目前Navigation Stack实现的全局规划器有：

* global_planner:一个快速的，内插值的路径规划器，其使用更灵活地代替navfn
* navfn:一个基于栅格的全局规划器，利用导航函数来计算路径
* carrot_planner:一个简单的全局规划器，其接收用户指定的全局点，并尝试让机器人更可能靠近目标点（目标点可以为障碍物）。

##  2.base_local_planner.h

```C++
#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

namespace nav_core {
  /**
   * @class BaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the navigation stack must adhere to this interface.局部规划器接口
   */
  class BaseLocalPlanner{
    public:
      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       给定机器人的当前位置、方向和速度，计算发送到机器人底盘的速度命令
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base将要传给机器人的速度指令
       * @return True if a valid velocity command was found, false otherwise找到一个有效的速度指令则为真，反之为false
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;

      /**
       * @brief  Check if the goal pose has been achieved by the local planner检查局部规划器的目标位姿是否到达
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached() = 0;

      /**
       * @brief  Set the plan that the local planner is following
       		设置局部规划器正在执行的路径
       * @param plan The plan to pass to the local planner
       		即将被传递给局部规划器的路径
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief  Constructs the local planner构建局部规划器
       * @param name The name to give this instance of the local planner局部规划器的名称
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
      virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseLocalPlanner(){}

    protected:
      BaseLocalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H
```

​		其中`setPlan`函数在`move_base`中的`MoveBase::executeCycle`函数中被调用，当全局路径规划成功就将其传递到局部规划器。然后在`MoveBase::executeCycle`中调用`computeVelocityCommands`计算出速度发送到`cmd_vel`话题。

目前Navigation_Stack实现的局部路径规划器有：

* base_local_planner:实现DWA和Trajectory Rollout
* dwa_local_planner:相对于上这的DWA，使用了更清晰、更容易的接口来实现一个更容易明白的DWA模块，并为机器人提供了更灵活的y轴（这里的y轴是什么？yaw轴？）

##  3.recovery_behavior.h

```C++
#ifndef NAV_CORE_RECOVERY_BEHAVIOR_H
#define NAV_CORE_RECOVERY_BEHAVIOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

namespace nav_core {
  /**
   * @class RecoveryBehavior
   * @brief Provides an interface for recovery behaviors used in navigation. All recovery behaviors written as plugins for the navigation stack must adhere to this interface.
   */
  class RecoveryBehavior{
    public:
      /**
       * @brief  Initialization function for the RecoveryBehavior
       * @param tf A pointer to a transform listener指向tf监听者的指针
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 指向全局costmap的指针
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 指向局部costmap的指针
       */
      virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) = 0;

      /**
       * @brief   Runs the RecoveryBehavior
       */
      virtual void runBehavior() = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~RecoveryBehavior(){}

    protected:
      RecoveryBehavior(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_RECOVERY_BEHAVIOR_H
```

`move_base`中多处用到了`runBehavior()`函数

目前Navigation Stack实现的恢复规划器有：

* clear_costmap_recovery：实现了清除代价地图的恢复行为。

  遍历所有层，然后如果某层在可清理的列表里就清除掉它的costmap。默认可清理列表中只有obstacle layer即障碍物层，即机器人实时扫描建立的costmap。

* rotate_recovery：实现了旋转的恢复行为，360度旋转来尝试清除空间

  转一圈看看有没有路。在runBehavior里只需要发指令让小车转一圈，有没有路是local costmap在转一圈过程中建立发现的。

* move_slow_and_clear：实现了缓慢移动的恢复行为。

  清理 costmap 然后什么都不管，按照前进速度和转角速度走。从代码里可以看到，根据指定的距离，这是通过先清除全局 costmap 跟局部  costmap 一圈的 obstacle layer 的障碍，然后直接发指令实现的。由于只清除了 obstacle layer ，其实  static layer 的障碍还在，而且这里清不清除跟发指令关系不大，该撞上去的还是会撞的，相当于闭着眼睛往前走。