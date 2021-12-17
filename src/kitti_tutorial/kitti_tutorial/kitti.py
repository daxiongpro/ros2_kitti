# /home/daxiongpro/datasets/kitti_ros_tracking/data_tracking_image_2/training/image_02/0000/0000000000.png
from .data_utils import *
from .publish_utils import *
from .utils import *
from .kitti_utils import *
import rclpy
from rclpy.node import Node

DATASET_PATH = '/home/daxiongpro/datasets/'
DATA_PATH = DATASET_PATH + 'kitti_raw_data/2011_09_26/2011_09_26_drive_0005_sync'
carlib_path = os.path.join(DATASET_PATH, 'kitti_raw_data/2011_09_26')


class KittiNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s！" % name)
        self.cam_pub = self.create_publisher(Image, 'kitti_cam', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, 'kitti_point_cloud', 10)
        self.bridge = CvBridge()
        self.ego_pub = self.create_publisher(Marker, 'kitti_ego_car', 10)
        self.imu_pub = self.create_publisher(Imu, 'kitti_imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'kitti_gps', 10)
        self.box3d_pub = self.create_publisher(MarkerArray, 'kitti_3dbox', 10)
        self.imu_odom_pub = self.create_publisher(MarkerArray, 'kitti_imu_odom', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame = 0
        self.df_tracking = read_tracking(os.path.join(DATA_PATH, 'training/label_02/0000.txt'))

    def timer_callback(self):
        df_tracking_frame = self.df_tracking[self.df_tracking.frame == self.frame]

        # read data
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png' % self.frame))
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin' % self.frame))
        # include imu and gps info
        imu_data = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt' % self.frame))
        boxes_2d, types = get_2d_box_and_type(df_tracking_frame)
        corner_3d_velos, track_ids, tracker, centers = get_3d_info(df_tracking_frame,
                                                                   carlib_path, imu_data)

        # publish
        publish_camera(self.cam_pub, self.bridge, image, boxes_2d, types)
        # publish_camera(self.cam_pub, self.bridge, image)
        publish_point_cloud(self.pcl_pub, point_cloud[::2])
        publish_ego_car(self.ego_pub)
        publish_imu(self.imu_pub, imu_data)
        # gps rviz cannot visulize, only use rostopic echo
        publish_gps(self.gps_pub, imu_data)
        publish_3dbox(self.box3d_pub, corner_3d_velos, track_ids, types)
        # publish_imu_odom(self.imu_odom_pub, tracker, centers)

        self.get_logger().info("kitti published %d " % self.frame)
        self.frame += 1
        # self.frame %= 154
        if self.frame == 154:
            self.frame = 0
            for track_id in tracker:
                tracker[track_id].reset()


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    node = KittiNode('hello_kitti')  # 新建一个节点
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # rcl关闭
