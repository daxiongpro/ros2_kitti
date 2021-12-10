# import sensor_msgs.point_cloud2 as pc2
import pcl
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PC2Subscriber(Node):
    def __init__(self):
        super().__init__('pc2_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        # self.subscription  # prevent unused variable warning

    # def listener_callback(self, msg):
    #     self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_callback(self, data):
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = []
        for p in pc:
            pc_list.append([p[0], p[1], p[2]])

        p = pcl.PointCloud()
        p.from_list(pc_list)
        seg = p.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        indices, model = seg.segment()


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
