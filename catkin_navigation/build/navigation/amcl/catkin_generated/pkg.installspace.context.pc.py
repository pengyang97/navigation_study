# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "rosbag;roscpp;dynamic_reconfigure;tf;nav_msgs;std_srvs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lamcl_sensors;-lamcl_map;-lamcl_pf".split(';') if "-lamcl_sensors;-lamcl_map;-lamcl_pf" != "" else []
PROJECT_NAME = "amcl"
PROJECT_SPACE_DIR = "/home/pengyang/catkin_navigation/install"
PROJECT_VERSION = "1.14.8"
