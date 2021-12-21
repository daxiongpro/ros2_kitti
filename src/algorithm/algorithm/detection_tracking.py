import matplotlib;
import matplotlib.pyplot as plt

matplotlib.use('TkAgg')
print(matplotlib.get_backend())
import argparse
import glob
from pathlib import Path

import os, numpy as np, sys, cv2
from PIL import Image

from re import I
import torch
from skimage import io
from PIL import Image
from torch.utils import data
from .tracking.pcdet.config import cfg, cfg_from_yaml_file
from .tracking.pcdet.utils import box_utils, calibration_kitti
from .tracking.pcdet.datasets import DatasetTemplate
from .tracking.pcdet.models import build_network, load_data_to_gpu
from .tracking.pcdet.utils import common_utils
from .tracking.mot.AB3DMOT_libs.model import AB3DMOT
from .tracking.mot.AB3DMOT_libs.kitti_utils import compute_box_3d, draw_projected_box3d, Calibration
from .tracking.Xinshuo_PyToolbox.xinshuo_visualization import random_colors

max_color = 30
colors = random_colors(max_color)
det_str2id = {'Pedestrian': 1, 'Car': 2, 'Cyclist': 3}
det_id2str = {1: 'Pedestrian', 2: 'Car', 3: 'Cyclist'}

width = 1242
height = 374


def show_image_with_boxes(img, objects_res, calib, height_threshold=0):
    img2 = np.copy(img)

    for obj in objects_res:
        box3d_pts_2d, _ = compute_box_3d(obj, calib.P2)
        color_tmp = tuple([int(tmp * 255) for tmp in colors[obj.id % max_color]])
        img2 = draw_projected_box3d(img2, box3d_pts_2d, color=color_tmp)
        text = 'ID: %d' % obj.id
        if box3d_pts_2d is not None:
            img2 = cv2.putText(img2, text, (int(box3d_pts_2d[4, 0]), int(box3d_pts_2d[4, 1]) - 8),
                               cv2.FONT_HERSHEY_TRIPLEX, 0.5, color=color_tmp)

    img = Image.fromarray(np.uint8(img2))
    img = img.resize((width, height))
    return img


def get_template_prediction(num_samples):
    ret_dict = {
        'name': np.zeros(num_samples), 'truncated': np.zeros(num_samples),
        'occluded': np.zeros(num_samples), 'alpha': np.zeros(num_samples),
        'bbox': np.zeros([num_samples, 4]), 'dimensions': np.zeros([num_samples, 3]),
        'location': np.zeros([num_samples, 3]), 'rotation_y': np.zeros(num_samples),
        'score': np.zeros(num_samples), 'boxes_lidar': np.zeros([num_samples, 7])
    }
    return ret_dict


def generate_single_sample_dict(box_dict, class_names):
    pred_scores = box_dict['pred_scores'].cpu().numpy()
    pred_boxes = box_dict['pred_boxes'].cpu().numpy()
    pred_labels = box_dict['pred_labels'].cpu().numpy()
    pred_dict = get_template_prediction(pred_scores.shape[0])
    if pred_scores.shape[0] == 0:
        return pred_dict
    pred_boxes_camera = box_utils.boxes3d_kitti_fakelidar_to_lidar(pred_boxes)
    pred_dict['name'] = np.array(class_names)[pred_labels - 1]
    id = []
    for name in pred_dict['name']:
        id.append(det_str2id[name])
    pred_dict['id'] = np.array(id).reshape(-1, 1)
    pred_dict['alpha'] = -np.arctan2(-pred_boxes[:, 1], pred_boxes[:, 0]) + pred_boxes_camera[:, 6]
    # pred_dict['bbox'] = pred_boxes_img
    pred_dict['dimensions'] = pred_boxes_camera[:, 3:6]
    pred_dict['location'] = pred_boxes_camera[:, 0:3]
    pred_dict['rotation_y'] = pred_boxes_camera[:, 6].reshape(-1, 1)
    pred_dict['score'] = pred_scores.reshape(-1, 1)
    pred_dict['boxes_lidar'] = pred_boxes

    return pred_dict


class Object3d(object):
    ''' 3d object label
                # bbox3d_tmp = d[0:7]
                # id_tmp = d[7]
                # ori_tmp = d[8]
                # type_tmp = det_id2str[d[9]]
                # bbox2d_tmp_trk = d[10:14]
                # conf_tmp = d[14]
     '''

    def __init__(self, data):
        data[1:] = [float(x) for x in data[1:]]

        # extract label, truncation, occlusion
        self.type = det_id2str[data[9]]  # 'Car', 'Pedestrian', ...
        # self.truncation = data[1] # truncated pixel ratio [0..1]
        # self.occlusion = int(data[2]) # 0=visible, 1=partly occluded, 2=fully occluded, 3=unknown
        self.alpha = data[8]  # object observation angle [-pi..pi]

        # extract 3d bounding box information
        self.h = data[1]  # box height
        self.w = data[2]  # box width
        self.l = data[0]  # box length (in meters)
        self.t = (data[3], data[4], data[5])  # location (x,y,z) in camera coord.
        self.ry = data[6]  # yaw angle (around Y-axis in camera coordinates) [-pi..pi]
        self.score = float(data[10])
        self.id = int(data[7])


class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, search_id='0000',
                 ext='.bin'):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        if search_id is None:
            return
        self.root_path = root_path
        self.search_id = search_id
        file_list = os.listdir(self.root_path / 'image_02' / search_id)
        file_list.sort()
        self.file_idx = []
        for file in file_list:
            self.file_idx.append(file.split('.')[0])

    def __len__(self):
        return len(self.file_idx)

    def get_image_shape(self, idx):
        img_file = self.root_path / 'image_02' / self.search_id / ('%s.png' % idx)
        assert img_file.exists()
        return np.array(io.imread(img_file).shape[:2], dtype=np.int32)

    def get_image_rgb_with_normal(self, idx):
        """
        return img with normalization in rgb mode
        :param idx:
        :return: imback(H,W,3)
        """
        img_file = self.root_path / 'image_02' / self.search_id / ('%s.png' % idx)
        im = np.array(Image.open(img_file))
        return im

    def get_calib(self, idx):
        calib_file = self.root_path / 'calib' / ('%s.txt' % self.search_id)
        assert calib_file.exists()
        return calibration_kitti.Calibration(calib_file)

    def get_lidar(self, idx):
        lidar_file = self.root_path / 'velodyne' / self.search_id / ('%s.bin' % idx)
        # print(lidar_file)
        assert lidar_file.exists()
        return np.fromfile(str(lidar_file), dtype=np.float32).reshape(-1, 4)

    def __getitem__(self, index):
        idx = self.file_idx[index]
        points = self.get_lidar(idx)
        calib = self.get_calib(idx)
        image_shape = self.get_image_shape(idx)
        image = self.get_image_rgb_with_normal(idx)
        input_dict = {
            'points': points,
            'frame_id': index,
            'calib': calib,
            'image_shape': image_shape,
            'image': image,
            'frame_id': idx
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict


def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='src/algorithm/algorithm/tracking/tools/cfgs/kitti_models/3DSSD_openPCDet.yaml',
                        help='specify the config for demo')
    parser.add_argument('--ckpt', type=str, default='src/algorithm/algorithm/tracking/tracking_dataset/final.pth',
                        help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

    args = parser.parse_args()

    cfg_from_yaml_file(args.cfg_file, cfg)

    return args, cfg


# points [x,y,z,i] (N,4) type: tensor
def detection_and_tracking(points):
    points=torch.tensor(points).float()
    args, cfg = parse_config()
    logger = common_utils.create_logger()
    data_dict = {}
    data_dict["batch_size"] = 1
    data_dict["points"] = torch.cat((torch.zeros(points.shape[0], 1), points), dim=1).cuda()
    model = build_network(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=cfg)
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()
    mot_tracker = AB3DMOT()
    object_res = []
    with torch.no_grad():
        pred_dicts, _ = model.forward(data_dict)
        # single_pred_dict = generate_single_sample_dict(pred_dicts[0],data_dict['calib'][0],data_dict['image_shape'][0],cfg.CLASS_NAMES)
        single_pred_dict = generate_single_sample_dict(pred_dicts[0], cfg.CLASS_NAMES)
        if single_pred_dict['score'].shape[0] != 0:
            # bbox = single_pred_dict['bbox']
            loc = single_pred_dict['location']
            dims = single_pred_dict['dimensions']
            ori_array = single_pred_dict['alpha'].reshape((-1, 1))  # orientation
            other_array = np.concatenate((single_pred_dict['id'], single_pred_dict['score']), axis=-1)
            additional_info = np.concatenate((ori_array, other_array), axis=1)
            dets = np.concatenate((dims, loc, single_pred_dict['rotation_y']), axis=1)
            dets_all = {'dets': dets, 'info': additional_info}
            trackers = mot_tracker.update(dets_all)
            for d in trackers:
                objects = Object3d(d)
                object_res.append(objects)
    return object_res


def main():

    logger.info('-----------------Detection_Tracking-------------------------')
    search_id = '0011'

    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
        root_path=Path(args.data_path), ext=args.ext, logger=logger, search_id=search_id
    )

    with torch.no_grad():
        for index, data_dict in enumerate(demo_dataset):
            points = torch.tensor(data_dict["points"])
            objects = detection_and_tracking(points, cfg, args, logger=logger)
            print("frame_id", data_dict['frame_id'])
            for object in objects:
                print("%d %s %f %f %f %f %f %f %f %f\n" % (
                object.id, object.type, object.score, object.t[0], object.t[1], object.t[2], object.h, object.w,
                object.l, object.ry))
    logger.info('Demo done.')


"""
the function to use
name: detection_and_tracking(points,cfg,args,logger=None,search_id="0011"):
input: 
    points  [x,y,z,i] (N,4) type: tensor 
    cfg,args  use  "args, cfg = parse_config()" to get
    logger: use "logger = common_utils.create_logger()" to get
    search_id: no need
output:
    objects type: list[]
    for object in objects:
        object is a class:

        
        .type = det_id2str[data[9]] # 'Car', 'Pedestrian', ...
        .h = data[1] # box height
        .w = data[2] # box width
        .l = data[0] # box length (in meters)
        .t = (data[3],data[4],data[5]) # location (x,y,z) in camera coord.
        .ry = data[6] # yaw angle (around Y-axis in camera coordinates) [-pi..pi]
        .score = float(data[10])
        .id = int(data[7])

"""
if __name__ == '__main__':
    main()
