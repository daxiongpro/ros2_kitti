## 项目介绍

使用Ros2对KITTI数据集进行可视化

## 安装与使用

#### 安装ros2 foxy

```bash
# 安装方式看参考文献 “ROS2 FOY 官网“、“小鱼ROS2 FOXY 教学”
```

#### 克隆项目

```bash
git clone https://github.com/daxiongpro/my_ros_kitti.git
cd my_ros_kitti
```

#### 编译

```bash
# source以下ros的环境，如果不想每次都source，可以加入.bashrc或者.zshrc的环境变量中
source /opt/ros/foxy/setup.bash # source /opt/ros/foxy/setup.zsh # 如果你电脑装了zsh
# 编译
colcon build
```

#### 发布

```bash
# 保证在工作空间下，即clone下来的根目录
source install/setup.bash # source install/setup.zsh 
ros2 run kitti_tutorial kitti_node
```

#### RVIZ2进行显示

- 终端中打开rviz2

```bash
rviz2
```

- 在rviz2中依次点击 "ADD" -> "By topic" -> “{{ 你想要加入的topic}}”

## 参考文献

[ROS2 FOY 官网](https://docs.ros.org/en/foxy/)

[小鱼ROS2 FOXY 教学](https://github.com/fishros/d2l-ros2-foxy)

[ROS1 KITTY 代码](https://github.com/seaside2mm/ros-kitti-project)

[ROS1 KITTY 代码对应的视频教程](https://www.bilibili.com/video/BV1qV41167d2)

[pycharm 配置ROS环境](https://blog.csdn.net/Wentage_Tsai/article/details/102764992)

