# /home/daxiongpro/datasets/kitti_ros_tracking/data_tracking_image_2/training/image_02/0000/0000000000.png
from .data_utils import *
from .publish_utils import *
from .utils import *
from .kitti_utils import *
import rclpy
from rclpy.node import Node

DATASET_PATH = '/home/daxiongpro/datasets/'
DATA_PATH = DATASET_PATH + 'kitti_raw_data/2011_09_26/2011_09_26_drive_0005_sync'


class KittiNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s！" % name)
        self.cam_pub = self.create_publisher(Image, 'kitti_cam', 10)

        self.pcl_pub = self.create_publisher(PointCloud2, 'kitti_point_cloud', 10)
        self.bridge = CvBridge()
        self.ego_pub = self.create_publisher(Marker, 'kitti_ego_car', 10)
        # self.imu_pub = self.create_publisher(Imu, 'kitti_imu', 10)
        # self.gps_pub = self.create_publisher.Publisher(
        #     NavSatFix, 'kitti_gps', queue_size=10)
        # self.box3d_pub = self.create_publisher.Publisher(
        #     MarkerArray, 'kitti_3dbox', queue_size=10)
        # self.imu_odom_pub = self.create_publisher.Publisher(MarkerArray,
        #                                                'kitti_imu_odom', queue_size=10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame = 0

    def timer_callback(self):
        # rate = rospy.Rate(10)
        # rate = self.create_rate(10)

        # df_tracking = read_tracking('/root/kitti/training/label_02/0000.txt')
        # calib = Calibration('/root/kitti/RawData/2011_09_26/', from_video=True)

        # tracker = {}  # save all obj odom
        # prev_imu_data = None

        # while rclpy.ok():  # ???

        # read file
        # df_tracking_frame = df_tracking[df_tracking.frame == frame]

        # boxes_2d = np.array(
        #     df_tracking_frame[['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
        # boxes_3d = np.array(df_tracking_frame[[
        #                     'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])

        # types = np.array(df_tracking_frame['type'])
        # track_ids = np.array(df_tracking_frame['track_id'])
        # track_ids = np.append(track_ids, 1000)  # append ego car

        # read data
        image = read_camera(os.path.join(
            DATA_PATH, 'image_02/data/%010d.png' % self.frame))

        point_cloud = read_point_cloud(os.path.join(
            DATA_PATH, 'velodyne_points/data/%010d.bin' % self.frame))
        # # include imu and gpss info
        # imu_data = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt' % self.frame))

        # corner_3d_velos = []
        # centers = {}  # current frame tracker. track id:center
        # for track_id, box_3d in zip(track_ids, boxes_3d):
        #     corner_3d_cam2 = compute_3d_box_cam2(*box_3d)
        #     corner_3d_velo = calib.project_rect_to_velo(
        #         np.array(corner_3d_cam2).T)
        #     corner_3d_velos += [corner_3d_velo]  # one bbox 8 x 3 array
        #     centers[track_id] = np.mean(corner_3d_velo, axis=0)[
        #         :2]  # get ccenter of every bbox, don't care about height

        # # for ego car, we set its id = -1, center [0,0]
        # centers[-1] = np.array([0, 0])

        # if prev_imu_data is None:
        #     for track_id in centers:
        #         tracker[track_id] = Object(centers[track_id], 20)
        # else:
        #     displacement = 0.1 * np.linalg.norm(imu_data[['vf', 'vl']])
        #     yaw_change = float(imu_data.yaw - prev_imu_data.yaw)
        #     print(track_id)
        #     for track_id in centers:  # for one frame id
        #         if track_id in tracker:
        #             tracker[track_id].update(
        #                 centers[track_id], displacement, yaw_change)
        #         else:
        #             tracker[track_id] = Object(centers[track_id], 20)
        #     for track_id in tracker:  # for whole ids tracked by prev frame,but current frame did not
        #         if track_id not in centers:  # dont know its center pos
        #             tracker[track_id].update(None, displacement, yaw_change)

        # prev_imu_data = imu_data

        # publish
        # publish_camera(self.cam_pub, self.bridge, image, boxes_2d, types)
        publish_camera(self.cam_pub, self.bridge, image)
        publish_point_cloud(self.pcl_pub, point_cloud[::2])
        # publish_ego_car(self.ego_pub)
        # publish_imu(self.imu_pub, imu_data)
        # # gps rviz cannot visulize, only use rostopic echo
        # publish_gps(gps_pub, imu_data)
        # publish_3dbox(box3d_pub, corner_3d_velos, track_ids, types)
        # publish_imu_odom(imu_odom_pub, tracker, centers)

        self.get_logger().info("kitti published %d " % self.frame)
        # cv2.imshow("test", image)
        # cv2.waitKey(10)
        # rate.sleep()
        self.frame += 1
        if self.frame == 154:
            self.frame = 0
            # for track_id in tracker:
            #     tracker[track_id].reset()


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy
    node = KittiNode('hello_kitti')  # 新建一个节点
    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # rcl关闭
