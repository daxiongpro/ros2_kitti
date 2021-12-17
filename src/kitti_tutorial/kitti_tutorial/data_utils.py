#!/usr/miniconda3/envs/ros2/bin/python

import cv2
import numpy as np
import os
import pandas as pd

from .kitti_utils import Calibration
from .utils import compute_3d_box_cam2, Object

TRACKING_COLUMN_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top',
                         'bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']

IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 'ax', 'ay', 'az', 'af',
                    'al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu', 'posacc', 'velacc', 'navstat', 'numsats', 'posmode',
                    'velmode', 'orimode']


def read_camera(path):
    return cv2.imread(path)


def read_point_cloud(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def read_tracking(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = TRACKING_COLUMN_NAMES
    df = df[df['track_id'] >= 0]  # remove DontCare objects
    df.loc[df.type.isin(['Bus', 'Truck', 'Van', 'Tram']), 'type'] = 'Car'  # Set all vehicle type to Car
    df = df[df.type.isin(['Car', 'Pedestrian', 'Cyclist'])]
    return df


def read_imu(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df


def get_2d_box_and_type(df_tracking_frame):
    boxes_2d = np.array(df_tracking_frame[['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
    types = np.array(df_tracking_frame['type'])  # car, bike, person
    return boxes_2d, types


def get_3d_info(df_tracking_frame, carlib_path, imu_data):
    calib = Calibration(carlib_path, from_video=True)

    tracker = {}  # save all obj odom
    prev_imu_data = None
    boxes_3d = np.array(df_tracking_frame[['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']])

    track_ids = np.array(df_tracking_frame['track_id'])
    track_ids = np.append(track_ids, 1000)  # append ego car

    corner_3d_velos = []
    centers = {}  # current frame tracker. track id:center
    for track_id, box_3d in zip(track_ids, boxes_3d):
        corner_3d_cam2 = compute_3d_box_cam2(*box_3d)
        corner_3d_velo = calib.project_rect_to_velo(np.array(corner_3d_cam2).T)
        corner_3d_velos += [corner_3d_velo]  # one bbox 8 x 3 array
        # get ccenter of every bbox, don't care about height
        centers[track_id] = np.mean(corner_3d_velo, axis=0)[:2]

        # for ego car, we set its id = -1, center [0,0]

        centers[-1] = np.array([0, 0])

        if prev_imu_data is None:
            for track_id in centers:
                tracker[track_id] = Object(centers[track_id], 20)
        else:
            displacement = 0.1 * np.linalg.norm(imu_data[['vf', 'vl']])
            yaw_change = float(imu_data.yaw - prev_imu_data.yaw)
            print(track_id)
            for track_id in centers:  # for one frame id
                if track_id in tracker:
                    tracker[track_id].update(
                        centers[track_id], displacement, yaw_change)
                else:
                    tracker[track_id] = Object(centers[track_id], 20)
            for track_id in tracker:  # for whole ids tracked by prev frame,but current frame did not
                if track_id not in centers:  # dont know its center pos
                    tracker[track_id].update(None, displacement, yaw_change)

        prev_imu_data = imu_data
    return corner_3d_velos, track_ids, tracker, centers
