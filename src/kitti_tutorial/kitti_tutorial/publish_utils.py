import math
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix, PointField
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2
import numpy as np

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from builtin_interfaces.msg import Time
from builtin_interfaces.msg import Duration

FRAME_ID = "map"  # the base coordinate name in rviz
RATE = 10
LIFETIME = 1.0 / RATE  # 1/rate
DETECTION_COLOR_MAP = {
    'Car': (255, 255, 0),
    'Pedestrian': (0, 226, 255),
    'Cyclist': (141, 40, 255)
}  # color for detection, in format bgr

# connect vertic
LINES = [[0, 1], [1, 2], [2, 3], [3, 0]]  # lower face
LINES += [[4, 5], [5, 6], [6, 7], [7, 4]]  # upper face
LINES += [[4, 0], [5, 1], [6, 2], [7, 3]]  # connect lower face and upper face
LINES += [[4, 1], [5, 0]]  # front face and draw x


def publish_camera(cam_pub, bridge, image, borders_2d_cam2s=None, object_types=None, log=False):
    """
    Publish image in bgr8 format
    If borders_2d_cam2s is not None, publish also 2d boxes with color specified by object_types
    If object_types is None, set all color to cyan
    """
    if borders_2d_cam2s is not None:
        for i, box in enumerate(borders_2d_cam2s):
            top_left = int(box[0]), int(box[1])
            bottom_right = int(box[2]), int(box[3])
            if object_types is None:
                cv2.rectangle(image, top_left, bottom_right, (255, 255, 0), 2)
            else:
                cv2.rectangle(image, top_left, bottom_right,
                              DETECTION_COLOR_MAP[object_types[i]], 2)

    image_temp = bridge.cv2_to_imgmsg(image, "bgr8")
    # header = Header(stamp=rospy.Time.now())
    image_temp.header.frame_id = FRAME_ID  # mdzz wasted me so many time!!!!!!
    cam_pub.publish(image_temp)


def publish_point_cloud(pcl_pub, point_cloud, frame_id='map'):
    def create_point_cloud(points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message

        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html

        """
        # In a PointCloud2 message, the point cloud is stored as an byte 
        # array. In order to unpack it, we also include some parameters 
        # which desribes the size of each individual point.

        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [sensor_msgs.PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header(frame_id=parent_frame)

        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),  # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )

    pcd = create_point_cloud(point_cloud[:, :3], frame_id)
    pcl_pub.publish(pcd)


def publish_ego_car(ego_car_pub):
    # publish left and right 45 degree FOV lines and ego car model mesh
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = Time()
    marker.id = 0
    marker.action = Marker.ADD
    marker.lifetime = Duration()
    marker.type = Marker.LINE_STRIP
    # line
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.2  # line width

    marker.points = []

    # check the kitti axis model
    marker.points.append(Point(x=5.0, y=-5.0, z=0.0))  # left up
    marker.points.append(Point(x=0.0, y=0.0, z=0.0))  # center
    marker.points.append(Point(x=5.0, y=5.0, z=0.0))  # right up

    ego_car_pub.publish(marker)


def publish_imu(imu_pub, imu_data):
    """
    Publish IMU data
    http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
    """

    def quaternion_from_euler(roll, pitch, yaw):
        """
        https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    imu = Imu()
    imu.header.frame_id = FRAME_ID
    imu.header.stamp = Time()
    q = quaternion_from_euler(float(imu_data.roll),
                              float(imu_data.pitch),
                              float(imu_data.yaw))  # prevent the data from being overwritten

    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    imu.linear_acceleration.x = float(imu_data.af)
    imu.linear_acceleration.y = float(imu_data.al)
    imu.linear_acceleration.z = float(imu_data.au)
    imu.angular_velocity.x = float(imu_data.wf)
    imu.angular_velocity.y = float(imu_data.wl)
    imu.angular_velocity.z = float(imu_data.wu)

    imu_pub.publish(imu)


def publish_gps(gps_pub, gps_data):
    """
    Publish GPS data
    """
    gps = NavSatFix()
    gps.header.frame_id = FRAME_ID
    gps.header.stamp = Time()
    gps.latitude = float(gps_data.lat)
    gps.longitude = float(gps_data.lon)
    gps.altitude = float(gps_data.alt)

    gps_pub.publish(gps)


def publish_3dbox(box3d_pub, corners_3d_velos, track_ids, types=None, publish_id=True):
    """
    Publish 3d boxes in velodyne coordinate, with color specified by object_types
    If object_types is None, set all color to cyan
    corners_3d_velos : list of (8, 4) 3d corners
    """
    marker_array = MarkerArray()
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        # 3d box
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = Time()
        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime = Duration(LIFETIME)
        marker.type = Marker.LINE_LIST

        b, g, r = DETECTION_COLOR_MAP[types[i]]
        if types is None:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        else:
            marker.color.r = r / 255.0
            marker.color.g = g / 255.0
            marker.color.b = b / 255.0
        marker.color.a = 1.0
        marker.scale.x = 0.1

        marker.points = []
        for l in LINES:
            p1 = corners_3d_velo[l[0]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))
        marker_array.markers.append(marker)

        # add track id
        if publish_id:
            track_id = track_ids[i]
            text_marker = Marker()
            text_marker.header.frame_id = FRAME_ID
            text_marker.header.stamp = Time()

            text_marker.id = track_id + 1000
            text_marker.action = Marker.ADD
            text_marker.lifetime = Duration(LIFETIME)
            text_marker.type = Marker.TEXT_VIEW_FACING

            p4 = corners_3d_velo[4]  # upper front left corner

            text_marker.pose.position.x = p4[0]
            text_marker.pose.position.y = p4[1]
            text_marker.pose.position.z = p4[2] + 0.5

            text_marker.text = str(track_id)

            text_marker.scale.x = 1
            text_marker.scale.y = 1
            text_marker.scale.z = 1

            if types is None:
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
            else:
                b, g, r = DETECTION_COLOR_MAP[types[i]]
                text_marker.color.r = r / 255.0
                text_marker.color.g = g / 255.0
                text_marker.color.b = b / 255.0
            text_marker.color.a = 1.0
        marker_array.markers.append(text_marker)

    box3d_pub.publish(marker_array)


def publish_imu_odom(imu_odom_pub, tracker, centers):
    marker_array = MarkerArray()

    for track_id in centers:

        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = Time()

        marker.action = Marker.ADD
        marker.lifetime = Duration(LIFETIME)
        marker.type = Marker.LINE_STRIP
        marker.id = track_id

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.2

        marker.points = []
        for p in tracker[track_id].locations:
            marker.points.append(Point(p[0], p[1], 0))

        marker_array.markers.append(marker)
    imu_odom_pub.publish(marker_array)
