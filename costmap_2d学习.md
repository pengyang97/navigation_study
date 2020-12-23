#  costmap_2d学习

> ​		自己根据[costmap_2d-ROS WIKI](https://wiki.ros.org/costmap_2d)的官方教程以及网上其他博主的博客并结合firefly机器人本身的代码对costmap_2d这个功能包进行了学习，以下是自己学的一写笔记。

* 先了解一些相关概念

*Voxel*：体素，是体积的像素。用来在三维空间中表示一个显示基本点的单位。类似于二维平面下的像素。Voxel是三维空间中定义一个点的图象信息的单位。光有三维坐标位置还不行，还要有颜色等信息，这就是Voxel的含义。

*footprint*：足迹，即机器人的轮廓。在ROS中，它由二维数组表示[x0,y0] ; [x1,y1] ;  [x2,y2]……这里不需要重复第一个坐标。该占位面积将用于计算内切圆和外接圆的半径，用于以适合此机器人的方式对障碍物进行膨胀。为了安全起见，我们通常将足迹设计的稍大于机器人的实际轮廓。

*cost*：代价或者占用，0-255的取值，表示机器人位于该网格点(grid cell)的代价，或者说是机器人的frootprint中心cell走到该网格点的代价。

* costmap的网格代价计算

costmap是机器人收集传感器信息建立和更新的二维或三维地图。激光雷达或者深度相机作为传感器跑出的2D或3D  SLAM地图，都不能直接用于实际的导航，必须将地图转化为costmap,ROS中costmap通常采用grid(网格)形式。栅格地图一个栅格占1个字节，也就是八位，可以存0-255中数据，也就是每个cell cost（网格的值）为0~255。虽然costmap中的每个单元格都可以有255个不同的成本值中的一个，但它使用的基础结构只能表示三个。具体地说，Occupied被占用（有障碍）, Free自由区域（无障碍）, Unknown Space未知区域。

<img src="http://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmap_rviz.png" alt="costmap_rviz"  />

​																					二维地图中机器人和障碍物的投影

​		在上图中，红色cell（图中红色蓝色区域都是一系列cell堆叠出来的）代表的是代价地图中的障碍，蓝色cell代表的是通过机器人内切圆半径计算的障碍物膨胀，红色多边形代表的是机器人footprint(机器人轮廓的垂直投影)。 **为了使机器人不碰到障碍物，机器人的footprint绝对不允许与红色cell相交，机器人的中心绝对不允许与蓝色cell相交。**

![机器人模型以及在当前前进方向上各grid的代价分布示意图](http://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png)

​															机器人模型以及在当前前进方向上各grid的代价分布示意图

​		膨胀是指成本值从占用的单元向外传播的过程，这些值随距离的增加而减小。这里定义了五个与机器人相关的costmap值的相关标志：

(1)	Lethal（致命的）:单元中存在实际的障碍物，机器人的中心（center cell）在如果在该网格中，此时机器人必然与障碍物冲突。（代价值为254）

(2)	Inscribed（内切）：网格中心位于机器人的内切圆轮廓内，此时机器人也必然与障碍物冲突。（代价值为253）

(3)	Possibly circumscribed（可能受限）：网格中心位于机器人的外切圆与内切圆轮廓之间，此时机器人相当于位于障碍物附近，所以不一定冲突，取决于机器人的方位或者说姿态。 使用”可能“一词是因为它可能不是真正的障碍单元，而是一些用户偏好（用户自己规定一些区域，不想让机器人通过），将特定的成本值放入地图中。（代价值为128）

(4)	Freespace（自由空间）：网格中心位于机器人外切圆之外，属于没有障碍物的空间。 （代价值为1）

(5)	Unknown（未知）：未知的空间。（代价值为0）

* costmap_2d地图的构成

costmap自动订阅ROS上传感器的话题，并相应地更新自己。每个传感器用于标记（将障碍物信息插入成本地图）、清除（从成本地图中删除障碍物信息），或两者兼而有之。标记操作只是一个数组的索引，用来改变单元的代价值。然而，清除操作包括对报告的每个观测值从传感器原点向外通过网格进行光线跟踪。如果使用三维结构来存储障碍物信息，则将每列的障碍物信息投影到成本图中。

​		costmap由多层组成，每种功能的地图放置一层。 例如图3所示，静态地图是一层，障碍物是另一层。缺省情况下，障碍物层维护的是3D信息，3D障碍物数据可以让层更加灵活的标记和清除障碍物。例如在costmap_2d包中，StaticLayer（静态地图层）是第一层，ObstacleLayer（障碍物层）是第二层，InflationLayer（膨胀层）是第三层。这三层组合成了master map（最终的costmap），供给路线规划模块使用。另外，自己也可以定义一层障碍物来不让机器人通过某片自由的区域，通过接口`costmap_2d::Costmap2DROS`，并使用pluginlib实例化Costmap2DROS，最后在添加到LayeredCostmap中。

![costmap_2d地图](https://img-blog.csdn.net/20180306201610663)

​																							costmap_2d地图的构成

​		costmap_2d包提供了一种可配置框架来维护机器人在代价地图上应该如何导航的信息。 代价地图使用来自**传感器的数据和来自静态地图中的信息**，通过`costmap_2d::Costmap2DROS`来存储和更新现实世界中障碍物信息。`costmap_2d::Costmap2DROS`给用户提供了纯2D的接口，这意味着查询障碍只能在列上进行。例如，在XY平面上位于同一位置的桌子和鞋，虽然在Z方向上有差异但是它们在`costmap_2d::Costmap2DROS`对象代价地图中对应的cell上拥有相同的代价值。 这种设计对平面空间进行路径规划是有帮助的。

​		costmap_2d提供的ROS化功能接口主要就是`costmap_2d::Costmap2DROS`，它使用 `costmap_2d::LayeredCostmap`来跟踪每一层。 **每一层在 Costmap2DROS中以插件方式被实例化，并被添加到 `LayeredCostmap`**。  **每一层可以独立编译**，且可使用C++接口实现对代价地图的随意修改，<u>即LayerdCostmap为Costmap2DROS（用户接口）提供了加载地图层的插件机制，每个插件（即地图层）都是Layer类型的</u>。 **`costmap_2d::Costmap2D`** 类中实现了用来存储和访问2D代价地图的的基本数据结构。

![](https://img-blog.csdn.net/20180306210507998?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvamlua2luZzAx/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

​																						costmap中的各层的继承关系

* costmap初始化流程

在navigation的主节点move_base中，建立了两个costmap。其中**planner_costmap_ros**是用于**全局导航的地图**，**controller_costmap_ros**是用于**局部导航用的地图**。

![costmap初始化流程](https://img-blog.csdn.net/20170504190526248?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvbHF5Z2FtZQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

​																									costmap初始化流程

（1） Costmap初始化首先获得全局坐标系和机器人坐标系的转换  
（2） 加载各个Layer，例如StaticLayer，ObstacleLayer，InflationLayer。  
（3） 设置机器人的轮廓  
（4） 实例化了一个Costmap2DPublisher来发布可视化数据。  
（5） 通过一个movementCB函数不断检测机器人是否在运动  
（6） 开启动态参数配置服务，服务启动了更新map的线程。

* costmap中各层的更新

costmap初始化过程中各层加载的调用过程：

![](https://img-blog.csdn.net/20170506161254343?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvbHF5Z2FtZQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

在move_base刚启动时就建立了两个costmap，分别是local_costmap和global_costmap，而这两个costmap都加载了三个Layer插件，它们的初始化过程如上图所示。

StaticLayer：主要为处理gmapping或者amcl（这里amcl产生的静态地图是？）等产生的静态地图。 
ObstacLayer：主要处理机器人移动过程中产生的障碍物信息。 
InflationLayer：主要处理机器人导航地图上的障碍物信息膨胀（让地图上的障碍物比实际障碍物的大小更大一些），尽可能使机器人更安全的移动。 
costmap在mapUpdateLoop线程中执行更新地图的操作，每个层的工作流程如下：

(1)	StaticLayer工作流程 

![](https://img-blog.csdn.net/20170506161337831?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvbHF5Z2FtZQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

​		上图是StaticLayer的工作流程，updateBounds阶段将更新的界限设置为整张地图，updateCosts阶段根据rolling参数（是否采用滚动窗口）设置的值，如果是，那静态地图会随着机器人移动而移动，则首先要获取静态地图坐标系到全局坐标系的转换，再更新静态地图层到master map里。

(2)	ObstacleLayer工作流程 

![](https://img-blog.csdn.net/20170506161422176?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvbHF5Z2FtZQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

​		上图是ObstacleLayer的工作流程，updateBounds阶段将获取传感器传来的障碍物信息经过处理后放入一个观察队列中，updateCosts阶段则将障碍物的信息更新到master map。 

(3)	inflationLayer工作流程 

![](https://img-blog.csdn.net/20170506161457408?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvbHF5Z2FtZQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

​		上图是inflationLayer的工作流程，updateBounds阶段由于本层没有维护的map，所以维持上一层地图调用的Bounds值（处理区域）。updateCosts阶段用了一个CellData结构存储master  map中每个grid点的信息，其中包括这个点的二维索引和这个点附近最近的障碍物的二维索引。改变每个障碍物Cell附近前后左右四个Cell的cost值，更新到master map就完成了障碍物的膨胀。

* costmap更新—障碍物标记和清除

代价地图自动订阅传感器发布的主题并基于数据进行相应自我更新。 对每个传感器来说，其可以用来执行mark（将障碍物信息插入到代价地图），也可以用来执行clear（从代价地图移除障碍物）或者二者都执行。marking操作就是索引到数组内修改cell的代价。然而对于clearing操作，每次观测报告都需要传感器源向外发射线，由射线穿过的珊格组成（是不是clear要清除该cell对应的诸多层中对应cell的状态）。 如果存储的障碍物信息是3D的，需要将每一列的障碍物信息投影成2D后才能放入到代价地图。

​		costmap以[参数update_frequency](http://wiki.ros.org/costmap_2d#Rate_parameters) 指定的周期进行costmap更新。每个周期传感器数据进来后，都要在代价地图底层占用结构上执行标记和清除障碍操作，并且这种结构会被投影到代价地图附上相应代价值。 这完成之后，对代价赋值为`costmap_2d::LETHAL_OBSTACLE`的每个cell执行障碍物的膨胀操作，即从每个代价cell向外传播代价值，直到用户定义的膨胀半径为止。这里确实只需要对状态为`LETHAL_OBSTACLE`的cell进行膨胀操作即可。

costmap的更新在mapUpdateLoop线程中实现，此线程分为两个阶段： 
（阶段一）UpdateBounds：这个阶段会更新每个Layer的更新区域（增量更新？只更新变化的部分而不是全部更新），这样在每个运行周期内减少了数据拷贝的操作时间。StaticLayer的Static map只在第一次做更新，Bounds 范围是整张Map的大小，而且在**UpdateBounds过程中没有对Static  Map层的数据做过任何的更新**（静态地图，一次写入，后面读取，很少变更）。ObstacleLayer在这个阶段主要的操作是更新Obstacles  Map层的数据，然后更新Bounds（清空了Master层对应的bounds内的数据）。InflationLayer则保持上一次的Bounds。 
（阶段二）UpdateCosts：这个阶段将各层数据逐一拷贝到Master Map，可以通过下图观察Master  Map的生成流程。（图来源于David Lu的《Layered Costmaps for Context-Sensitive  Navigation》）

![](https://img-blog.csdn.net/20170504190719658?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvbHF5Z2FtZQ==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast) 

​		在（a）中，初始有三个Layer和Master costmap。Static Layer和Obstacles  Layer维护它们自己的地图，而inflation  Layer并没有。为了更新costmap,算法首先在各层上调用自己的UpdateBounds方法（**Static map不做更新**）（b）。为了决定新的bounds,Obstacles Layer利用新的传感器数据更新它的costmap。然后每个层轮流用UpdateCosts方法更新Master  costmap的某个区域,从Static Layer开始（c），然后是Obstacles Layer(d)，最后是inflation  Layer(e)。

（持续更新中......）