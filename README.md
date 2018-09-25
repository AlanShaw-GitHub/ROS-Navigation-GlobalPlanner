# ROS-Navigation-GlobalPlanner
A universal RRT algorithm implementation under ROS Navigation GlobalPlanner toolbox.



### 一些说明

我实现的rrt算法使用了yaml文件进行配置选择不同的算法，`use_grid_path`参数为0或者1时用的是gridpath和gradientpath，就是旧的算法，当变成2时就会使用我的rrt算法，之前的a*之类的都不会用到了。

源代码的各个部分都做了小修改以适应新算法的架构（包括改进后的A*算法），rrt算法实现主要在`src/rrt.h` 和 　`include/global_planner/rrt.h`

还有就是在引用我的包的时候要配置一下`move_base.launch`改成  `<param name="base_global_planner" value="my_planner/GlobalPlanner"/>`（之前是global_planner::GlobalPlanner），以及引用yaml： `<rosparam file="$(find nav_sim)/cfg/base_global_planner_params.yaml" command="load" />`
