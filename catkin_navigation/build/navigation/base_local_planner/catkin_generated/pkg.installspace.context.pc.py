# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "angles;costmap_2d;dynamic_reconfigure;geometry_msgs;message_runtime;nav_core;nav_msgs;pluginlib;roscpp;std_msgs;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lbase_local_planner;-ltrajectory_planner_ros".split(';') if "-lbase_local_planner;-ltrajectory_planner_ros" != "" else []
PROJECT_NAME = "base_local_planner"
PROJECT_SPACE_DIR = "/home/pengyang/catkin_navigation/install"
PROJECT_VERSION = "1.14.8"
