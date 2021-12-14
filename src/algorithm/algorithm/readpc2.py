import point_cloud2 as pc2
# import pcl
# import sensor_msgs.msg._point_field
import numpy as np

from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.node import Node


class PC2Subscriber(Node):
    def __init__(self):
        super().__init__('pc2_subscriber')
        self.subscription = self.create_subscription(PointCloud2, 'rslidar_points', self.listener_callback, 10)
        # self.subscription  # prevent unused variable warning

    # def listener_callback(self, msg):
    #     self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_callback(self, pc: PointCloud2):
        print(pc.data)
        pc = pc2.read_points(pc, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = []
        for p in pc:
            pc_list.append([p[0], p[1], p[2]])

        return pc_list


def main(args=None):
    rclpy.init(args=args)
    pc2_subscriber = PC2Subscriber()
    rclpy.spin(pc2_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pc2_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
