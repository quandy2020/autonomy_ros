# Override the built-in print function with a timestamp version
import builtins
import json
import os
from datetime import datetime

import cv2
import jsonlines
import numpy as np
import pandas as pd
import torch
from PIL import Image
from scipy.interpolate import CubicSpline
from torch.utils.data import Dataset
from tqdm import tqdm

original_print = builtins.print


def print(*args, **kwargs):
    try:
        rank = int(os.environ.get('RANK', 0))
        if rank == 0:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            original_print(f"[{timestamp}]", *args, **kwargs)
    except Exception:  # Catch any exception to prevent crashes
        pass


builtins.print = print


class NavDP_Base_Datset(Dataset):
    def __init__(
        self,
        root_dirs,
        preload_path=False,
        memory_size=8,
        predict_size=24,
        batch_size=64,
        image_size=224,
        scene_data_scale=1.0,
        trajectory_data_scale=1.0,
        pixel_channel=7,
        action_dim=3,
        debug=False,
        preload=False,
        random_digit=False,
        prior_sample=False,
        dataset_repeat=1,
        obstacle_sample_n=2048,
        target_segment_weights=None,
        sample_interval=4,
        use_pointcloud=True,
    ):

        self.dataset_dirs = np.array([p for p in os.listdir(root_dirs)])
        self.dataset_repeat = max(1, int(dataset_repeat))
        self.memory_size = memory_size
        self.image_size = image_size
        self.scene_scale_size = scene_data_scale
        self.trajectory_data_scale = trajectory_data_scale
        self.predict_size = predict_size
        self.action_dim = action_dim
        self.debug = debug
        self.obstacle_sample_n = int(obstacle_sample_n)
        self.use_pointcloud = bool(use_pointcloud)

        self.trajectory_data_dir = []
        self.trajectory_rgb_path = []
        self.trajectory_depth_path = []
        self.trajectory_afford_path = []
        self.trajectory_scene_id = []  # 每个 episode 所属场景的 ID (基于 afford_path 去重)
        self.random_digit = random_digit
        self.prior_sample = prior_sample
        self.sample_interval = max(1, int(sample_interval))
        self.pixel_channel = pixel_channel
        self.target_segment_weights = target_segment_weights
        self.item_cnt = 0
        self.batch_size = batch_size
        self.batch_time_sum = 0.0
        self._last_time = None

        if preload is False:
            for group_dir in self.dataset_dirs:  # gibson_zed, 3dfront ...
                all_scene_dirs = np.array([p for p in os.listdir(os.path.join(root_dirs, group_dir))])
                select_scene_dirs = all_scene_dirs[
                    np.arange(0, all_scene_dirs.shape[0], 1 / self.scene_scale_size).astype(np.int32)
                ]

                for scene_dir in tqdm(select_scene_dirs):
                    chunk_name = os.listdir(os.path.join(root_dirs, group_dir, scene_dir, 'data'))[0]
                    data_dir = os.path.join(root_dirs, group_dir, scene_dir, f'data/{chunk_name}')
                    afford_dir = os.path.join(root_dirs, group_dir, scene_dir, 'meta/pointcloud.ply')
                    with jsonlines.open(
                        os.path.join(root_dirs, group_dir, scene_dir, 'meta/episodes_stats.jsonl'), 'r'
                    ) as reader:
                        episode_info = list(reader)
                    rgb_dir = os.path.join(
                        root_dirs, group_dir, scene_dir, f"videos/{chunk_name}/observation.images.rgb/"
                    )
                    rgb_paths = [os.path.join(rgb_dir, p) for p in sorted(os.listdir(rgb_dir))]

                    depth_dir = os.path.join(
                        root_dirs, group_dir, scene_dir, f"videos/{chunk_name}/observation.images.depth/"
                    )
                    depth_paths = [os.path.join(depth_dir, p) for p in sorted(os.listdir(depth_dir))]

                    data_paths = [os.path.join(data_dir, p) for p in sorted(os.listdir(data_dir))]

                    for episode_idx, episode in enumerate(episode_info):
                        image_start_index = episode['image_index']['min']
                        image_end_index = episode['image_index']['max']
                        episode_rgb_path = np.array(rgb_paths)[image_start_index : image_end_index + 1].tolist()
                        episode_depth_path = np.array(depth_paths)[image_start_index : image_end_index + 1].tolist()

                        try:
                            self.trajectory_data_dir.append(data_paths[episode_idx])
                            self.trajectory_rgb_path.append(episode_rgb_path)
                            self.trajectory_depth_path.append(episode_depth_path)
                            self.trajectory_afford_path.append(afford_dir)
                            self.trajectory_scene_id.append(afford_dir)
                        except Exception as e:
                            import pdb

                            print(f"Error processing episode {episode_idx}: {e}")
                            pdb.set_trace()

            save_dict = {
                'trajectory_data_dir': self.trajectory_data_dir,
                'trajectory_rgb_path': self.trajectory_rgb_path,
                'trajectory_depth_path': self.trajectory_depth_path,
                'trajectory_afford_path': self.trajectory_afford_path,
                'trajectory_scene_id': self.trajectory_scene_id,
            }
            with open(preload_path, 'w') as f:
                json.dump(save_dict, f, indent=4)

            self.trajectory_data_dir = self.trajectory_data_dir * self.dataset_repeat
            self.trajectory_rgb_path = self.trajectory_rgb_path * self.dataset_repeat
            self.trajectory_depth_path = self.trajectory_depth_path * self.dataset_repeat
            self.trajectory_afford_path = self.trajectory_afford_path * self.dataset_repeat
            self.trajectory_scene_id = self.trajectory_scene_id * self.dataset_repeat
        else:
            load_dict = json.load(open(preload_path, 'r'))
            self.trajectory_data_dir = load_dict['trajectory_data_dir'] * self.dataset_repeat
            self.trajectory_rgb_path = load_dict['trajectory_rgb_path'] * self.dataset_repeat
            self.trajectory_depth_path = load_dict['trajectory_depth_path'] * self.dataset_repeat
            self.trajectory_afford_path = load_dict['trajectory_afford_path'] * self.dataset_repeat
            self.trajectory_scene_id = load_dict.get(
                'trajectory_scene_id',
                [str(p) for p in load_dict['trajectory_afford_path']],
            ) * self.dataset_repeat

        # ---- 构建场景索引 ----
        self._build_scene_index()

    def __len__(self):
        return len(self.trajectory_data_dir)

    def _build_scene_index(self):
        """构建场景索引: 将 episode 按场景 (afford_path) 分组.

        注意: 当 dataset_repeat > 1 时, episode 索引会出现重复 (如 idx=0 和 idx=N 对应同一 episode).
        这里只使用原始 (未 repeat) 的索引区间, 避免 sample_episodes_from_scenes 返回重复索引.
        """
        num_raw = len(self.trajectory_data_dir) // self.dataset_repeat
        self.episodes_by_scene = {}  # scene_id -> List[episode_index] (原始索引, 未 repeat)
        self.unique_scene_ids = []   # 去重后的场景 ID 列表
        for idx in range(num_raw):
            scene_id = self.trajectory_scene_id[idx]
            if scene_id not in self.episodes_by_scene:
                self.episodes_by_scene[scene_id] = []
                self.unique_scene_ids.append(scene_id)
            self.episodes_by_scene[scene_id].append(idx)

    def sample_episodes_from_scenes(self, num_scenes: int, num_episodes: int, rng=None, return_scene_ids=False):
        """随机选 num_scenes 个场景, 在所有选中场景内共采集 num_episodes 个 episode.

        Args:
            num_scenes: 选取的场景数量
            num_episodes: 总共采集的 episode 数量 (从选中场景中均匀随机抽取)
            rng: numpy random Generator (可选, 用于可复现)
            return_scene_ids: 是否返回选中的场景 ID 列表

        Returns:
            episode_indices: List[int], 选中的 episode 索引列表
            scene_ids (可选): List[str], 选中的场景 ID 列表 (仅当 return_scene_ids=True 时返回)
        """
        if rng is None:
            rng = np.random.default_rng()
        # 随机选 num_scenes 个场景
        selected_scenes = rng.choice(self.unique_scene_ids, size=min(num_scenes, len(self.unique_scene_ids)), replace=False)
        # 收集这些场景的所有 episode index
        candidate_indices = []
        for sid in selected_scenes:
            candidate_indices.extend(self.episodes_by_scene[sid])
        # 从中随机抽 num_episodes 个
        num_episodes = min(num_episodes, len(candidate_indices))
        chosen = rng.choice(candidate_indices, size=num_episodes, replace=False).tolist()
        
        if return_scene_ids:
            return chosen, selected_scenes.tolist()
        else:
            return chosen

    def load_image(self, image_url):
        image = Image.open(image_url)
        image = np.array(image, np.uint8)
        return image

    def load_depth(self, depth_url):
        depth = Image.open(depth_url)
        depth = np.array(depth, np.uint16)
        return depth

    def load_pointcloud(self, pcd_url):
        import open3d as o3d

        pcd = o3d.io.read_point_cloud(pcd_url)
        return pcd

    def process_image(self, image_path):
        image = self.load_image(image_path)
        H, W, C = image.shape
        prop = self.image_size / max(H, W)
        image = cv2.resize(image, (-1, -1), fx=prop, fy=prop)
        pad_width = max((self.image_size - image.shape[1]) // 2, 0)
        pad_height = max((self.image_size - image.shape[0]) // 2, 0)
        pad_image = np.pad(
            image, ((pad_height, pad_height), (pad_width, pad_width), (0, 0)), mode='constant', constant_values=0
        )
        image = cv2.resize(pad_image, (self.image_size, self.image_size))
        image = np.array(image, np.float32) / 255.0
        return image

    def process_depth(self, depth_path):
        depth = self.load_depth(depth_path) / 10000.0
        H, W = depth.shape
        prop = self.image_size / max(H, W)
        depth = cv2.resize(depth, (-1, -1), fx=prop, fy=prop)
        pad_width = max((self.image_size - depth.shape[1]) // 2, 0)
        pad_height = max((self.image_size - depth.shape[0]) // 2, 0)
        pad_depth = np.pad(
            depth, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant', constant_values=0
        )
        pad_depth[pad_depth > 5.0] = 0
        pad_depth[pad_depth < 0.1] = 0
        depth = cv2.resize(pad_depth, (self.image_size, self.image_size))
        depth = np.array(depth, np.float32)
        return depth[:, :, np.newaxis]

    def process_data_parquet(self, index):
        if not os.path.isfile(self.trajectory_data_dir[index]):
            raise FileNotFoundError(self.trajectory_data_dir[index])
        df = pd.read_parquet(self.trajectory_data_dir[index])
        camera_intrinsic = np.vstack(np.array(df['observation.camera_intrinsic'].tolist()[0])).reshape(3, 3)
        camera_extrinsic = np.vstack(np.array(df['observation.camera_extrinsic'].tolist()[0])).reshape(4, 4)
        trajectory_length = len(df['action'].tolist())
        camera_trajectory = np.array([np.stack(frame) for frame in df['action']], dtype=np.float64).reshape(-1, 4, 4)
        return camera_intrinsic, camera_extrinsic, camera_trajectory, trajectory_length

    def process_obstacle_points(self, index):
        """处理障碍物点云，区分真实障碍物和膨胀障碍物。
        
        点云颜色编码：
        - 黑色 [0, 0, 0]: 路径点（path_points），不加载
        - 灰色 [0.4, 0.4, 0.4]: 真实障碍物，仅用于eval可视化
        - 蓝色 [0, 0, 0.5]: 膨胀障碍物，用于碰撞和占用奖励计算
        - 其他颜色: 不加载
        
        Returns:
            scene_obstacle_points: [N, 3] 世界坐标系下的真实障碍物点 (float32)
            scene_inflation_points: [M, 3] 世界坐标系下的膨胀障碍物点 (float32)
        
        Raises:
            FileNotFoundError: 如果点云文件不存在
        """
        if not self.use_pointcloud:
            empty = np.zeros((0, 3), dtype=np.float32)
            return empty, empty
        pcd_path = self.trajectory_afford_path[index]
        if not os.path.isfile(pcd_path):
            raise FileNotFoundError(
                f"NavDP_Base_Datset: 点云文件不存在: {pcd_path}\n"
                f"障碍物点云是计算碰撞/占用奖励的必要条件，请确保每个场景目录下存在点云文件。"
            )
        scene_pcd = self.load_pointcloud(pcd_path)
        scene_color = np.array(scene_pcd.colors)
        scene_points = np.array(scene_pcd.points)
        
        # 1. 灰色 [0.4, 0.4, 0.4]: 真实障碍物（仅用于eval可视化）
        gray_distance = np.abs(scene_color - np.array([0.4, 0.4, 0.4])).sum(axis=-1)
        gray_mask = gray_distance < 0.05
        scene_obstacle_points = scene_points[gray_mask].astype(np.float32)
        
        # 2. 蓝色 [0, 0, 0.5]: 膨胀障碍物（用于碰撞和占用奖励计算）
        blue_distance = np.abs(scene_color - np.array([0, 0, 0.5])).sum(axis=-1)
        blue_mask = blue_distance < 0.05
        scene_inflation_points = scene_points[blue_mask].astype(np.float32)
        
        return scene_obstacle_points, scene_inflation_points

    def _obstacles_to_local(self, world_points, base_frame_extrinsic, base_extrinsic, sample_radius=3.0, filter_fov=True):
        """Transform world-frame obstacle points to the local-robot frame anchored
        at `base_frame_extrinsic` (same frame used by process_actions for pred_actions).

        不进行降采样，只保留sample_radius范围内的点。
        可选：只保留前方180度（左右各90度）范围内的障碍物。

        Args:
            world_points: 世界坐标系下的障碍物点 [N, 3]
            base_frame_extrinsic: 起点时刻的相机外参 [4, 4]
            base_extrinsic: 机器人基座外参 [4, 4]
            sample_radius: 采样半径（米），默认3.0m
            filter_fov: 是否过滤前方视场角范围外的障碍物（默认True，保留前方180度）

        Returns: np.float32 array of shape [M, 2] (xy only)，M为sample_radius范围内且（可选）前方FOV内的点数。
                 如果没有点，返回空数组。
        """
        if world_points is None or world_points.shape[0] == 0:
            return np.zeros((0, 2), dtype=np.float32)

        _, local = self.relative_pose(
            base_frame_extrinsic[0:3, 0:3],
            base_frame_extrinsic[0:3, 3],
            np.eye(3),
            world_points,
            base_extrinsic,
        )
        local_xy = local[:, 0:2].astype(np.float32)

        # 只保留sample_radius范围内的点，不进行随机降采样
        distances = np.linalg.norm(local_xy, axis=-1)
        in_range_mask = distances <= sample_radius
        local_xy = local_xy[in_range_mask]

        # 过滤前方视场角范围外的障碍物（前方180度，左右各90度）
        if filter_fov and local_xy.shape[0] > 0:
            # 局部坐标系：前方为x轴正方向，左侧为y轴正方向
            # angle = atan2(y, x)，前方180度满足 |angle| <= 90度
            angles = np.arctan2(local_xy[:, 1], local_xy[:, 0])
            half_fov_rad = np.deg2rad(90.0)  # 左右各90度
            in_fov_mask = np.abs(angles) <= half_fov_rad
            local_xy = local_xy[in_fov_mask]

        return local_xy

    def process_memory(self, rgb_paths, depth_paths, start_step, memory_digit=1):
        memory_index = np.arange(start_step - (self.memory_size - 1) * memory_digit, start_step + 1, memory_digit)
        outrange_sum = (memory_index < 0).sum()
        memory_index = memory_index[outrange_sum:]
        context_image = np.zeros((self.memory_size, self.image_size, self.image_size, 3), np.float32)
        context_image[outrange_sum:] = np.array([self.process_image(rgb_paths[i]) for i in memory_index])
        context_depth = self.process_depth(depth_paths[start_step])
        return context_image, context_depth, memory_index

    def process_pixel_goal(self, image_url, target_point, camera_intrinsic, camera_extrinsic):
        image = Image.open(image_url)
        image = np.array(image, np.uint8)
        resize_image = self.process_image(image_url)

        coordinate = np.array([-target_point[1], target_point[0], camera_extrinsic[2, 3] * 0.8])
        camera_coordinate = np.matmul(camera_extrinsic[0:3, 0:3], coordinate[:, None])
        pixel_coord_x = camera_intrinsic[0, 2] + (camera_coordinate[0] / camera_coordinate[2]) * camera_intrinsic[0, 0]
        pixel_coord_y = camera_intrinsic[1, 2] + (-camera_coordinate[1] / camera_coordinate[2]) * camera_intrinsic[1, 1]
        pixel_mask = np.zeros_like(image)
        visible_flag = False

        if (
            pixel_coord_x > 0
            and pixel_coord_x < image.shape[1]
            and pixel_coord_y > 0
            and pixel_coord_y < image.shape[0]
        ):
            pixel_mask = cv2.rectangle(
                pixel_mask,
                (int(pixel_coord_x - np.random.randint(6, 12)), int(pixel_coord_y - np.random.randint(6, 12))),
                (int(pixel_coord_x + np.random.randint(6, 12)), int(pixel_coord_y + np.random.randint(6, 12))),
                (255, 255, 255),
                -1,
            )
            visible_flag = True

        H, W, C = pixel_mask.shape
        prop = self.image_size / max(H, W)
        pixel_mask = cv2.resize(pixel_mask, (-1, -1), fx=prop, fy=prop)
        pad_width = max((self.image_size - pixel_mask.shape[1]) // 2, 0)
        pad_height = max((self.image_size - pixel_mask.shape[0]) // 2, 0)
        pad_mask = np.pad(
            pixel_mask, ((pad_height, pad_height), (pad_width, pad_width), (0, 0)), mode='constant', constant_values=0
        )
        mask = cv2.resize(pad_mask, (self.image_size, self.image_size), interpolation=cv2.INTER_NEAREST)
        mask = np.array(mask, np.float32) / 255.0
        mask = mask.mean(axis=-1)[:, :, None]
        return np.concatenate((resize_image, mask), axis=-1), visible_flag

    def relative_pose(self, R_base, T_base, R_world, T_world, base_extrinsic):
        R_base = np.matmul(R_base, np.linalg.inv(base_extrinsic[0:3, 0:3]))
        if len(T_world.shape) == 1:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_frame = np.dot(R_world, R_base.T)
            T_frame = np.dot(np.linalg.inv(homo_RT), np.array([*T_world, 1]).T)[0:3]
            T_frame = np.array([T_frame[1], -T_frame[0], T_frame[2]])  # [:T[1],-T[0],T[2]
            return R_frame, T_frame
        else:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_frame = np.dot(R_world, R_base.T)
            T_frame = np.dot(
                np.linalg.inv(homo_RT), np.concatenate((T_world, np.ones((T_world.shape[0], 1))), axis=-1).T
            ).T[:, 0:3]
            T_frame = T_frame[:, [1, 0, 2]]
            T_frame[:, 1] = -T_frame[:, 1]
            return R_frame, T_frame

    def absolute_pose(self, R_base, T_base, R_frame, T_frame, base_extrinsic):
        R_base = np.matmul(R_base, np.linalg.inv(base_extrinsic[0:3, 0:3]))
        if len(T_frame.shape) == 1:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_world = np.dot(R_frame, R_base)
            T_world = np.dot(homo_RT, np.array([-T_frame[1], T_frame[0], T_frame[2], 1]).T)[0:3]
        else:
            homo_RT = np.eye(4)
            homo_RT[0:3, 0:3] = R_base
            homo_RT[0:3, 3] = T_base
            R_world = np.dot(R_frame, R_base)
            T_world = np.dot(
                homo_RT,
                np.concatenate(
                    (np.stack((-T_frame[:, 1], T_frame[:, 0], T_frame[:, 2]), axis=-1), np.ones((T_frame.shape[0], 1))),
                    axis=-1,
                ).T,
            ).T[:, 0:3]
        return R_world, T_world

    def xyz_to_xyt(self, xyz_actions, init_vector):
        xyt_actions = []
        for i in range(0, xyz_actions.shape[0] - 1):
            current_vector = xyz_actions[i + 1] - xyz_actions[i]
            dot_product = np.dot(init_vector[0:2], current_vector[0:2])
            cross_product = np.cross(init_vector[0:2], current_vector[0:2])
            theta = np.arctan2(cross_product, dot_product)
            xyt_actions.append([xyz_actions[i][0], xyz_actions[i][1], theta])
        return np.array(xyt_actions)

    def process_actions(self, extrinsics, base_extrinsic, start_step, end_step, pred_digit=1):
        label_linear_pos = []
        for f_ext in extrinsics[start_step : end_step + 1]:
            R, T = self.relative_pose(
                extrinsics[start_step][0:3, 0:3],
                extrinsics[start_step][0:3, 3],
                f_ext[0:3, 0:3],
                f_ext[0:3, 3],
                base_extrinsic,
            )
            label_linear_pos.append(T)
        label_actions = np.array(label_linear_pos)

        # this is usesd for action augmentations:
        # (1) apply random rotation to the future steps
        # (2) interpolate between the rotated actions and origin actions
        rotate_yaw_angle = np.random.uniform(-np.pi / 3, np.pi / 3)
        rotate_matrix = np.array(
            [
                [np.cos(rotate_yaw_angle), -np.sin(rotate_yaw_angle)],
                [np.sin(rotate_yaw_angle), np.cos(rotate_yaw_angle)],
            ],
            np.float32,
        )
        rotate_local_actions = np.matmul(rotate_matrix, label_actions[:, 0:2].T).T
        rotate_local_actions = np.stack(
            (rotate_local_actions[:, 0], rotate_local_actions[:, 1], np.zeros_like(rotate_local_actions[:, 0])), axis=-1
        )
        rotate_world_points = []
        for act in rotate_local_actions:
            w_rot, w_act = self.absolute_pose(
                extrinsics[start_step, 0:3, 0:3], extrinsics[start_step, 0:3, 3], np.eye(3), act, base_extrinsic
            )
            rotate_world_points.append(w_act)
        rotate_world_points = np.array(rotate_world_points)
        origin_world_points = extrinsics[start_step : end_step + 1, 0:3, 3]
        mix_anchor_points = rotate_world_points

        t = np.linspace(0, 1, mix_anchor_points.shape[0])
        cs_x = CubicSpline(t, mix_anchor_points[:, 0])
        cs_y = CubicSpline(t, mix_anchor_points[:, 1])
        cs_z = CubicSpline(t, mix_anchor_points[:, 2])
        interpolate_nums = origin_world_points.shape[0]
        t_fine = np.linspace(0, 1, int(interpolate_nums))
        x_fine = cs_x(t_fine)
        y_fine = cs_y(t_fine)
        z_fine = cs_z(t_fine)
        result_augment_points = np.stack((x_fine, y_fine, z_fine), axis=-1)
        local_label_points = []
        local_augment_points = []
        for f_ext, g_ext in zip(origin_world_points, result_augment_points):
            Rf, Tf = self.relative_pose(
                extrinsics[start_step][0:3, 0:3], extrinsics[start_step][0:3, 3], np.eye(3), f_ext, base_extrinsic
            )
            Rg, Tg = self.relative_pose(
                extrinsics[start_step][0:3, 0:3], extrinsics[start_step][0:3, 3], np.eye(3), g_ext, base_extrinsic
            )
            local_label_points.append(Tf)
            local_augment_points.append(Tg)
        local_label_points = np.array(local_label_points)
        local_augment_points = np.array(local_augment_points)
        action_indexes = np.clip(np.arange(self.predict_size + 1) * pred_digit, 0, label_actions.shape[0] - 2)
        return local_label_points, local_augment_points, origin_world_points, result_augment_points, action_indexes

    def rank_steps(self, extrinsics, obstacle_points, pred_digit=4):
        points_score = []
        trajectory = extrinsics[:, 0:2, 3]
        # bev_points = obstacle_points[:, 0:2]
        for i in range(0, trajectory.shape[0] - 1):
            future_actions = trajectory[i : min(i + self.predict_size * pred_digit, trajectory.shape[0] - 1)]
            future_bound = [
                np.min(future_actions[:, 0]) - 1,
                np.min(future_actions[:, 1]) - 1,
                np.max(future_actions[:, 0]) + 1,
                np.max(future_actions[:, 1]) + 1,
            ]
            within_bound_points = (
                (obstacle_points[:, 0] > future_bound[0])
                & (obstacle_points[:, 1] > future_bound[1])
                & (obstacle_points[:, 0] < future_bound[2])
                & (obstacle_points[:, 1] < future_bound[3])
            )
            points_score.append(np.sum(within_bound_points))
        points_score = np.array(points_score) / (np.array(points_score).max() + 1e-8)
        probs = np.exp(points_score / 0.2) / np.sum(np.exp(points_score / 0.2))
        start_choice = np.random.choice(np.arange(probs.shape[0]), p=probs)
        target_choice_candidates = np.arange(start_choice + 1, trajectory.shape[0])
        target_choice_p = (target_choice_candidates - start_choice) / (
            (target_choice_candidates - start_choice).max() + 1e-8
        )
        target_choice_p = np.exp(target_choice_p / 0.2) / np.exp(target_choice_p / 0.2).sum()
        target_choice = np.random.choice(target_choice_candidates, p=target_choice_p)
        return start_choice, target_choice

    def _sample_target_by_segments(self, start_choice: int, max_target: int) -> int:
        """根据分段权重采样目标帧.

        将 [start_choice+1, max_target] 等分为 N 段 (N = len(target_segment_weights)),
        每段按权重决定被选中的概率, 段内再均匀随机取一个帧.

        分段方式: 将 total_len = hi - lo + 1 个候选帧按顺序分配到 N 段,
        前 (total_len % N) 段各多分 1 帧, 保证每个帧恰好属于一段, 无遗漏无重叠.

        Args:
            start_choice: 起始帧索引.
            max_target: 目标帧最大值 (含).

        Returns:
            采样得到的目标帧索引.
        """
        lo = start_choice + 1
        hi = max_target
        if lo > hi:
            return hi

        # 无权重配置时退化为原始均匀随机
        if self.target_segment_weights is None or len(self.target_segment_weights) == 0:
            return np.random.randint(lo, hi + 1)

        num_segments = len(self.target_segment_weights)
        total_len = hi - lo + 1  # 可选帧总数

        # 构建每段边界: boundaries[i] 为第 i 段起始帧, boundaries[num_segments] = hi + 1
        # 前 (total_len % N) 段各分 ceil(total_len/N) 帧, 后面各分 floor(total_len/N) 帧
        base_size = total_len // num_segments
        remainder = total_len % num_segments
        boundaries = [lo]
        for i in range(num_segments):
            seg_size = base_size + (1 if i < remainder else 0)
            boundaries.append(boundaries[-1] + seg_size)
        # boundaries[-1] == lo + total_len == hi + 1

        weights = np.array(self.target_segment_weights, dtype=np.float64)
        probs = weights / weights.sum()

        # 先按概率选段, 再在段内均匀采样
        seg_idx = np.random.choice(num_segments, p=probs)
        seg_lo = boundaries[seg_idx]
        seg_hi = boundaries[seg_idx + 1]  # 右端点不含, np.random.randint 左闭右开
        # Guard: when total_len < num_segments, some segments are empty
        # (seg_lo == seg_hi). Pick a non-empty segment in that case.
        if seg_lo >= seg_hi:
            for fallback_idx in range(num_segments):
                flo, fhi = boundaries[fallback_idx], boundaries[fallback_idx + 1]
                if fhi > flo:
                    seg_lo, seg_hi = flo, fhi
                    break
            else:
                # All segments empty — should not happen given lo <= hi,
                # but return lo as safe fallback.
                return lo
        return int(np.random.randint(seg_lo, seg_hi))

    def __getitem__(self, index):
        # 直接调用实现，遇到错误直接抛出
        return self._getitem_impl(index)

    def _getitem_impl(self, index):
        import os
        import time

        if self._last_time is None:
            self._last_time = time.time()
        start_time = time.time()

        (
            camera_intrinsic,
            camera_extrinsic,
            trajectory_extrinsics,
            trajectory_length,
        ) = self.process_data_parquet(index)
        
        # camera_extrinsic 是第0帧相机外参（用于 process_actions 的 base_extrinsic）
        # trajectory_base_extrinsic 是机器人基座外参
        # 注意：投影障碍物奖励需要与深度图同帧的 camera_extrinsic，
        # 那个在下面通过 trajectory_extrinsics[memory_start_choice] 获取
        trajectory_base_extrinsic = camera_extrinsic

        trajectory_obstacle_points, trajectory_inflation_points = self.process_obstacle_points(index)

        if self.prior_sample:
            pixel_start_choice, target_choice = self.rank_steps()
            memory_start_choice = np.random.randint(pixel_start_choice, target_choice)
        else:
            pixel_start_choice = np.random.randint(0, trajectory_length // 2)
            target_choice = self._sample_target_by_segments(
                pixel_start_choice, trajectory_length - 1
            )
            memory_start_choice = np.random.randint(pixel_start_choice, target_choice)

        if self.random_digit:
            memory_digit = np.random.randint(2, 8)
            pred_digit = memory_digit
        else:
            memory_digit = self.sample_interval
            pred_digit = self.sample_interval

        memory_images, depth_image, memory_index = self.process_memory(
            self.trajectory_rgb_path[index],
            self.trajectory_depth_path[index],
            memory_start_choice,
            memory_digit=memory_digit,
        )
        (
            target_local_points,
            augment_local_points,
            target_world_points,
            augment_world_points,
            action_indexes,
        ) = self.process_actions(
            trajectory_extrinsics, trajectory_base_extrinsic, memory_start_choice, target_choice, pred_digit=pred_digit
        )

        # convert the xyz points into xy-theta points
        init_vector = target_local_points[1] - target_local_points[0]
        self._last_sample_debug = {
            "index": int(index),
            "init_vector": np.asarray(init_vector, dtype=np.float32).copy(),
            "base_heading_angle": float(np.arctan2(init_vector[1], init_vector[0])),
            "memory_start_choice": int(memory_start_choice),
            "target_choice": int(target_choice),
        }
        target_xyt_actions = self.xyz_to_xyt(target_local_points, init_vector)
        augment_xyt_actions = self.xyz_to_xyt(augment_local_points, init_vector)
        # based on the prediction length to decide the final prediction trajectories
        pred_actions = target_xyt_actions[action_indexes]
        augment_actions = augment_xyt_actions[action_indexes]
        if trajectory_obstacle_points.shape[0] != 0:
            pred_distance = (
                np.abs(target_world_points[:, np.newaxis, 0:2] - trajectory_obstacle_points[np.newaxis, :, 0:2])
                .sum(axis=-1)
                .min(axis=-1)
            )
            augment_distance = (
                np.abs(augment_world_points[:, np.newaxis, 0:2] - trajectory_obstacle_points[np.newaxis, :, 0:2])
                .sum(axis=-1)
                .min(axis=-1)
            )
            pred_critic = (
                -5.0 * (pred_distance[action_indexes[:-1]] < 0.1).mean()
                + 0.5 * (pred_distance[action_indexes][1:] - pred_distance[action_indexes][:-1]).sum()
            )
            augment_critic = (
                -5.0 * (augment_distance[action_indexes[:-1]] < 0.1).mean()
                + 0.5 * (augment_distance[action_indexes][1:] - augment_distance[action_indexes][:-1]).sum()
            )
        else:
            pred_distance = np.ones(pred_actions.shape[0], dtype=np.float32)
            augment_distance = np.ones(pred_actions.shape[0], dtype=np.float32)
            pred_critic = 2.0
            augment_critic = 2.0

        point_goal = target_xyt_actions[-1]
        image_goal = np.concatenate(
            (
                self.process_image(self.trajectory_rgb_path[index][target_choice]),
                self.process_image(self.trajectory_rgb_path[index][memory_start_choice]),
            ),
            axis=-1,
        )

        # process pixel projection
        pixel_target_local_points, _, _, _, _ = self.process_actions(
            trajectory_extrinsics, trajectory_base_extrinsic, pixel_start_choice, target_choice, pred_digit=pred_digit
        )
        pixel_init_vector = pixel_target_local_points[1] - pixel_target_local_points[0]
        pixel_xyt_actions = self.xyz_to_xyt(pixel_target_local_points, pixel_init_vector)
        pixel_goal, pixel_flag = self.process_pixel_goal(
            self.trajectory_rgb_path[index][pixel_start_choice],
            pixel_xyt_actions[-1],
            camera_intrinsic,
            trajectory_base_extrinsic,
        )
        # pixel channel == 7 represents the navdp works pixel navigation under asynchronous pace,
        # pixel_mask (1), the history image with the assigned pixel goal (3), current image (3)
        # if pixel_channel == 4, pixel goal is assigned at current frame, therefore,
        # only pixel_mask (1) and current image (3) are needed
        if self.pixel_channel == 7:
            pixel_goal = np.concatenate((pixel_goal, memory_images[-1]), axis=-1)

        pred_actions = (pred_actions[1:] - pred_actions[:-1]) * 4.0
        augment_actions = (augment_actions[1:] - augment_actions[:-1]) * 4.0

        # Transform obstacles into the same local-robot frame as pred_actions
        # (anchored at extrinsics[memory_start_choice], matching process_actions).
        # 真实障碍物（灰色）：仅用于eval可视化
        obstacle_local_points = self._obstacles_to_local(
            trajectory_obstacle_points,
            trajectory_extrinsics[memory_start_choice],
            trajectory_base_extrinsic,
        )
        
        # 膨胀障碍物（蓝色）：用于碰撞和占用奖励计算
        inflation_local_points = self._obstacles_to_local(
            trajectory_inflation_points,
            trajectory_extrinsics[memory_start_choice],
            trajectory_base_extrinsic,
        )
        
        # 保存世界坐标的完整障碍物点云和外参（用于 reward 计算和可视化）
        # 使用与 _obstacles_to_local 相同的起点时刻外参
        world_obstacle_points = trajectory_obstacle_points.astype(np.float32)
        world_inflation_points = trajectory_inflation_points.astype(np.float32)
        base_frame_extrinsic = trajectory_extrinsics[memory_start_choice].astype(np.float32)
        base_extrinsic = trajectory_base_extrinsic.astype(np.float32)

        pred_actions = np.pad(
            pred_actions,
            ((0, 0), (0, self.action_dim - pred_actions.shape[-1])),
            mode='constant',
            constant_values=(0, 0),
        )
        augment_actions = np.pad(
            augment_actions,
            ((0, 0), (0, self.action_dim - augment_actions.shape[-1])),
            mode='constant',
            constant_values=(0, 0),
        )

        # Summarize avg time of batch
        end_time = time.time()
        self.item_cnt += 1
        self.batch_time_sum += end_time - start_time
        if self.item_cnt % self.batch_size == 0:
            avg_time = self.batch_time_sum / self.batch_size
            print(
                f'__getitem__ pid={os.getpid()}, avg_time(last {self.batch_size})={avg_time:.2f}s, cnt={self.item_cnt}'
            )
            self.batch_time_sum = 0.0
        point_goal = torch.tensor(point_goal, dtype=torch.float32)
        image_goal = torch.tensor(image_goal, dtype=torch.float32)
        pixel_goal = torch.tensor(pixel_goal, dtype=torch.float32)
        memory_images = torch.tensor(memory_images, dtype=torch.float32)
        depth_image = torch.tensor(depth_image, dtype=torch.float32)
        pred_actions = torch.tensor(pred_actions, dtype=torch.float32)
        augment_actions = torch.tensor(augment_actions, dtype=torch.float32)
        pred_critic = torch.tensor(pred_critic, dtype=torch.float32)
        augment_critic = torch.tensor(augment_critic, dtype=torch.float32)
        obstacle_local_points = torch.tensor(obstacle_local_points, dtype=torch.float32)
        inflation_local_points = torch.tensor(inflation_local_points, dtype=torch.float32)
        # 添加世界坐标的完整障碍物点云和外参（用于 reward 计算和可视化）
        world_obstacle_points = torch.tensor(world_obstacle_points, dtype=torch.float32)
        world_inflation_points = torch.tensor(world_inflation_points, dtype=torch.float32)
        base_frame_extrinsic = torch.tensor(base_frame_extrinsic, dtype=torch.float32)
        base_extrinsic = torch.tensor(base_extrinsic, dtype=torch.float32)
        # 添加相机内参和相机外参（用于投影障碍物奖励）
        # 注意：camera_extrinsic 必须与 depth_image 同帧！
        # depth_image 来自 memory_start_choice 帧（process_memory 中 depth_paths[memory_start_choice]），
        # 所以 camera_extrinsic 也必须使用同一帧的外参（trajectory_extrinsics[memory_start_choice]），
        # 而不是第0帧的 observation.camera_extrinsic（那个仅用于 process_actions 的 base_extrinsic）。
        camera_intrinsic_tensor = torch.tensor(camera_intrinsic, dtype=torch.float32)
        camera_extrinsic_tensor = torch.tensor(
            trajectory_extrinsics[memory_start_choice], dtype=torch.float32
        )
        return (
            point_goal,                      # 0
            image_goal,                      # 1
            pixel_goal,                      # 2
            memory_images,                   # 3
            depth_image,                     # 4
            pred_actions,                    # 5
            augment_actions,                 # 6
            pred_critic,                    # 7
            augment_critic,                  # 8
            float(pixel_flag),              # 9
            obstacle_local_points,           # 10 [N, 2] 局部坐标系真实障碍物（灰色，仅用于可视化）
            world_obstacle_points,           # 11 [M, 3] 世界坐标系真实障碍物
            base_frame_extrinsic,           # 12 [4, 4] 起点时刻相机外参
            base_extrinsic,                 # 13 [4, 4] 机器人基座外参
            inflation_local_points,          # 14 [K, 2] 局部坐标系膨胀障碍物（蓝色，用于奖励计算）
            world_inflation_points,          # 15 [L, 3] 世界坐标系膨胀障碍物
            camera_intrinsic_tensor,         # 16 [3, 3] 相机内参（用于投影障碍物奖励）
            camera_extrinsic_tensor,        # 17 [4, 4] 深度图同帧的相机外参（用于投影障碍物奖励）
        )


def navdp_collate_fn(batch):
    """Collate function for NavDP dataset.
    
    注意：障碍物点云是变长数组，使用列表存储而非stack。
    """
    collated = {
        "batch_pg": torch.stack([item[0] for item in batch]),
        "batch_ig": torch.stack([item[1] for item in batch]),
        "batch_tg": torch.stack([item[2] for item in batch]),
        "batch_rgb": torch.stack([item[3] for item in batch]),
        "batch_depth": torch.stack([item[4] for item in batch]),
        "batch_labels": torch.stack([item[5] for item in batch]),
        "batch_augments": torch.stack([item[6] for item in batch]),
        "batch_label_critic": torch.stack([item[7] for item in batch]),
        "batch_augment_critic": torch.stack([item[8] for item in batch]),
        # 障碍物点云是变长数组，使用列表存储
        "batch_obstacle_points": [item[10] for item in batch],      # 局部坐标系真实障碍物
        "batch_world_obstacle_points": [item[11] for item in batch], # 世界坐标系真实障碍物
        "batch_base_frame_extrinsic": torch.stack([item[12] for item in batch]),  # 起点时刻相机外参
        "batch_base_extrinsic": torch.stack([item[13] for item in batch]),        # 机器人基座外参
        "batch_inflation_points": [item[14] for item in batch],      # 局部坐标系膨胀障碍物
        "batch_world_inflation_points": [item[15] for item in batch], # 世界坐标系膨胀障碍物
        # 相机参数（用于投影障碍物奖励）
        "batch_camera_intrinsic": torch.stack([item[16] for item in batch]),  # 相机内参 [B, 3, 3]
        "batch_camera_extrinsic": torch.stack([item[17] for item in batch]),    # 相机外参 [B, 4, 4]
    }
    return collated


if __name__ == "__main__":
    os.makedirs("./navdp_dataset_test/", exist_ok=True)
    dataset = NavDP_Base_Datset(
        "/mnt/data/liuyu/InternDate-N1-v05/vln-n1",
        "./navdp_dataset_test/dataset_lerobot_v05_with_interiorgs.json",
        8,
        24,
        224,
        trajectory_data_scale=1.0,
        scene_data_scale=1.0,
        preload=False,
    )

    for i in range(10):
        (
            point_goal,
            image_goal,
            pixel_goal,
            memory_images,
            depth_image,
            pred_actions,
            augment_actions,
            pred_critic,
            augment_critic,
            pixel_flag,
            obstacle_local_points,
            world_obstacle_points,
            base_frame_extrinsic,
            base_extrinsic,
            inflation_local_points,
            world_inflation_points,
            camera_intrinsic,
            camera_extrinsic,
        ) = dataset.__getitem__(i)
        if pixel_flag == 1.0:
            pixel_obs = pixel_goal.numpy()[:, :, 0:3] * 255
            pixel_obs[pixel_goal[:, :, 3] == 1] = np.array([0, 0, 255])

            draw_current_image = cv2.cvtColor(image_goal[:, :, 3:6].numpy() * 255, cv2.COLOR_BGR2RGB)
            draw_current_image = cv2.putText(
                draw_current_image, "Current-Image", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255)
            )

            draw_goal_image = cv2.cvtColor(image_goal[:, :, 0:3].numpy() * 255, cv2.COLOR_BGR2RGB)
            draw_goal_image = cv2.putText(
                draw_goal_image, "Image-Goal", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255)
            )

            draw_pixel_image = cv2.cvtColor(pixel_obs.copy(), cv2.COLOR_BGR2RGB)
            draw_pixel_image = cv2.putText(
                draw_pixel_image, "Pixel-Goal", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255)
            )

            goal_info_image = np.concatenate((draw_current_image, draw_goal_image, draw_pixel_image), axis=1)
            goal_info_image = cv2.putText(
                goal_info_image,
                "PointGoal=[{:.3f}, {:.3f}, {:.3f}]".format(*point_goal),
                (190, 210),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
            )
            cv2.imwrite("./navdp_dataset_test/goal_information_%d.png" % i, goal_info_image)
