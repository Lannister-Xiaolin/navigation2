novabot_navigation ROS2 Package
===========================

### 一、说明
此包用来代替navigation2中的行为树调用框架，全局路径和局部路径不会以一定频率规划，而是每次规划一定长度，当走机器人走过一般路径时再以上次规划的终点作为起点规划一段同样长度的路径。如果之前规划的路径没有碰到障碍物，则不会重新规划，否则则以当前机器人的位置为起点重复之前的规划步骤。提供的对外action接口和参数也与navigation2一致,具体使用可以参考novabot_bringup包。

### 二、服务
提供一个std_srvs/srv/SetBool类型的服务——/use_straight_planner，以供在覆盖割草时设置优先使用直线规划器。

### 三、依赖
navigation2   teb_local_planner

