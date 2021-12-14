## 算法节点

### ROS2读取速腾激光雷达的rosbag

##### 作用

- 速腾给了ros2 录的rosbag包，编写订阅，并读取成点云N*3形式

##### 测试

* 使用`ros2 bag play xxx` 播放rosbag

* 根据[速腾激光雷达github](https://github.com/RoboSense-LiDAR/rslidar_sdk)打开 rviz2 设置指定的topic，可以看到点云可视化图

##### 读取rosbag的topic

* 写在readpc2.py文件中
* 结果保存在`pcd`变量中

##### 难点

- ROS2中没有point_cloud2.py这个模块

##### 解决方法

* 自己把ros1中的point_cloud2.py文件夹copy过来



#### 参考文献

[速腾激光雷达github](https://github.com/RoboSense-LiDAR/rslidar_sdk)

[解决思路](https://answers.ros.org/question/357944/why-is-point_cloud2py-missing-from-sensor_msgs-in-ros2/)

[point_cloud2.py文件](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py)

