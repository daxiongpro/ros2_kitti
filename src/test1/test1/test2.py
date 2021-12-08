#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 导入话题消息类型
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import Image

class WriterNode(Node):
    """
    创建一个李四节点，并在初始化时输出一个话
    """

    def __init__(self, name):
        # 创建并初始化订阅者成员属性submoney
        self.submoney = self.create_subscription(
            Image, "dingyue_kitti", self.recv_money_callback, 10)

    def timer_callback(self):
        """
        定时器回调函数
        """

        msg = String()
        msg.data = '第%d回：潋滟湖 %d 次偶遇胡艳娘' % (self.i, self.i)
        self.pubnovel.publish(msg)  # 将小说内容发布出去
        self.get_logger().info('李四:我发布了艳娘传奇："%s"' % msg.data)  # 打印一下发布的数据，供我们看
        self.i += 1  # 章节编号+1

    def recv_money_callback(self, money):
        """
        4. 编写订阅回调处理逻辑
        """
        
        self.get_logger().info('dingyue chenggong ')


def main(args=None):
    """
    ros2运行该节点的入口函数，可配置函数名称
    """
    rclpy.init(args=args)  # 初始化rclpy
    node = WriterNode("dingyue")  # 新建一个节点
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # rcl关闭
