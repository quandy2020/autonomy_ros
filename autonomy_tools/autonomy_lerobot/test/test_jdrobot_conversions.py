# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

import math
import unittest

import numpy as np

from autonomy_lerobot.conversions import (
    camera_info_to_intrinsic,
    depth_rgb_to_pointcloud_array,
    depth_to_video_rgb,
    identity_extrinsic,
    image_to_numpy,
    pose_to_action_matrix,
)
from autonomy_lerobot.observation import (
    KEY_ACTION,
    KEY_CAMERA_EXTRINSIC,
    KEY_CAMERA_INTRINSIC,
    KEY_DEPTH,
    KEY_POINTCLOUD,
    KEY_RGB,
    KEY_SEMANTIC,
)
from autonomy_lerobot.collection_params import (
    JDROBOT_COLLECTION_ROS_PARAMS,
    jdrobot_collection_parameters,
)
from autonomy_lerobot.bag_to_lerobot import _yaw_from_pose_matrix
from autonomy_lerobot.bag_postprocess import BagPostprocessConfig, BagPostprocessor
from autonomy_lerobot.recorder import (
    _features_from_frame,
    dataset_schema_matches,
    lerobot_encoder_vcodec,
)
from autonomy_lerobot.sam3_segmenter import Sam3GroundSegmenter


class TestJdrobotConversions(unittest.TestCase):
    def test_identity_extrinsic(self) -> None:
        ext = identity_extrinsic()
        self.assertEqual(ext.shape, (16,))
        np.testing.assert_allclose(ext.reshape(4, 4), np.eye(4), rtol=1e-6)

    def test_pose_to_action_matrix_yaw(self) -> None:
        yaw = math.pi / 2.0
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        action = pose_to_action_matrix(1.0, 2.0, 0.5, 0.0, 0.0, qz, qw)
        mat = action.reshape(4, 4)
        self.assertAlmostEqual(mat[0, 3], 1.0, places=5)
        self.assertAlmostEqual(mat[1, 3], 2.0, places=5)
        self.assertAlmostEqual(mat[2, 3], 0.5, places=5)
        self.assertAlmostEqual(mat[3, 3], 1.0, places=5)

    def test_camera_info_to_intrinsic(self) -> None:
        from sensor_msgs.msg import CameraInfo

        msg = CameraInfo()
        msg.k = [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]
        intrinsic = camera_info_to_intrinsic(msg)
        np.testing.assert_allclose(
            intrinsic,
            np.array([320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0], dtype=np.float32),
        )

    def test_depth_to_video_rgb(self) -> None:
        depth = np.array([[0.0, 5.0], [10.0, np.nan]], dtype=np.float32)
        rgb = depth_to_video_rgb(depth, depth_min=0.0, depth_max=10.0)
        self.assertEqual(rgb.shape, (2, 2, 3))
        self.assertEqual(rgb.dtype, np.uint8)
        self.assertEqual(int(rgb[0, 0, 0]), 0)
        self.assertEqual(int(rgb[0, 1, 0]), 127)
        self.assertEqual(int(rgb[1, 0, 0]), 255)
        self.assertEqual(int(rgb[1, 1, 0]), 0)

    def test_image_to_numpy_16uc1_depth_mm_to_meters(self) -> None:
        from sensor_msgs.msg import Image

        msg = Image()
        msg.height = 2
        msg.width = 2
        msg.encoding = '16UC1'
        msg.data = np.array([[0, 1500], [2500, 10000]], dtype=np.uint16).tobytes()
        depth = image_to_numpy(msg)
        self.assertEqual(depth.shape, (2, 2))
        self.assertEqual(depth.dtype, np.float32)
        np.testing.assert_allclose(
            depth,
            np.array([[0.0, 1.5], [2.5, 10.0]], dtype=np.float32),
        )

    def test_depth_rgb_to_pointcloud_center_pixel(self) -> None:
        from sensor_msgs.msg import CameraInfo

        info = CameraInfo()
        info.k = [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]
        depth = np.zeros((480, 640), dtype=np.float32)
        depth[240, 320] = 2.0
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[240, 320] = [10, 20, 30]
        cloud = depth_rgb_to_pointcloud_array(
            depth, info, rgb=rgb, max_points=8, depth_min=0.1, depth_max=5.0, stride=1)
        self.assertEqual(cloud.shape, (8, 6))
        self.assertAlmostEqual(cloud[0, 0], 0.0, places=4)
        self.assertAlmostEqual(cloud[0, 1], 0.0, places=4)
        self.assertAlmostEqual(cloud[0, 2], 2.0, places=4)
        self.assertEqual(int(cloud[0, 3]), 10)
        self.assertEqual(int(cloud[0, 4]), 20)
        self.assertEqual(int(cloud[0, 5]), 30)

    def test_depth_rgb_to_pointcloud_with_mask(self) -> None:
        from sensor_msgs.msg import CameraInfo

        info = CameraInfo()
        info.k = [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]
        depth = np.zeros((480, 640), dtype=np.float32)
        depth[240, 320] = 2.0
        mask = np.zeros((480, 640), dtype=bool)
        cloud = depth_rgb_to_pointcloud_array(
            depth,
            info,
            valid_mask=mask,
            max_points=8,
            depth_min=0.1,
            depth_max=5.0,
            stride=1,
        )
        self.assertTrue(np.allclose(cloud, 0.0))

    def test_bag_postprocessor_adds_pointcloud_without_sam3(self) -> None:
        from sensor_msgs.msg import CameraInfo

        info = CameraInfo()
        info.k = [320.0, 0.0, 320.0, 0.0, 320.0, 240.0, 0.0, 0.0, 1.0]
        depth = np.zeros((480, 640), dtype=np.float32)
        depth[240, 320] = 2.0
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[240, 320] = [10, 20, 30]
        frame: dict[str, np.ndarray | str] = {}
        postprocessor = BagPostprocessor(
            BagPostprocessConfig(
                record_pointcloud=True,
                save_segmentation_viz=False,
                pointcloud_stride=1,
                max_pointcloud_points=8,
                depth_min_m=0.1,
                depth_max_m=5.0,
            )
        )
        postprocessor.process_frame(
            frame=frame,
            rgb=rgb,
            depth=depth,
            camera_info=info,
        )
        self.assertIn(KEY_POINTCLOUD, frame)
        self.assertNotIn(KEY_SEMANTIC, frame)
        cloud = frame[KEY_POINTCLOUD]
        assert isinstance(cloud, np.ndarray)
        self.assertEqual(cloud.shape, (8, 6))
        self.assertAlmostEqual(cloud[0, 2], 2.0, places=4)

    def test_jdrobot_feature_schema_with_pointcloud(self) -> None:
        frame = {
            KEY_CAMERA_INTRINSIC: np.zeros(9, dtype=np.float32),
            KEY_CAMERA_EXTRINSIC: np.eye(4, dtype=np.float32).reshape(-1),
            KEY_ACTION: np.eye(4, dtype=np.float32).reshape(-1),
            KEY_RGB: np.zeros((480, 640, 3), dtype=np.uint8),
            KEY_DEPTH: np.zeros((480, 640, 3), dtype=np.uint8),
            KEY_SEMANTIC: np.zeros((480, 640, 3), dtype=np.uint8),
            KEY_POINTCLOUD: np.zeros((4096, 6), dtype=np.float32),
        }
        features = _features_from_frame(frame, dataset_format='jdrobot')
        self.assertIn(KEY_POINTCLOUD, features)
        self.assertIn(KEY_SEMANTIC, features)
        self.assertEqual(features[KEY_POINTCLOUD]['shape'], (4096, 6))

    def test_dataset_schema_matches_kujiale_with_optional_pointcloud(self) -> None:
        info_with_pc = {
            'robot_type': 'jdrobot',
            'fps': 20,
            'features': {
                'observation.camera_intrinsic': {'shape': [9]},
                'observation.camera_extrinsic': {'shape': [16]},
                'action': {'shape': [16]},
                'observation.images.rgb': {'info': {'video.codec': 'av1'}},
                'observation.images.depth': {'info': {'video.codec': 'av1'}},
                'observation.images.semantic': {'info': {'video.codec': 'av1'}},
                'observation.pointcloud': {'shape': [4096, 6]},
                'timestamp': {},
            },
        }
        ok, reason = dataset_schema_matches(
            info_with_pc,
            dataset_format='jdrobot',
            robot_type='jdrobot',
            fps=20,
            video_vcodec='av1',
        )
        self.assertTrue(ok, reason)

    def test_jdrobot_feature_schema(self) -> None:
        frame = {
            KEY_CAMERA_INTRINSIC: np.zeros(9, dtype=np.float32),
            KEY_CAMERA_EXTRINSIC: np.eye(4, dtype=np.float32).reshape(-1),
            KEY_ACTION: np.eye(4, dtype=np.float32).reshape(-1),
            KEY_RGB: np.zeros((480, 640, 3), dtype=np.uint8),
            KEY_DEPTH: np.zeros((480, 640, 3), dtype=np.uint8),
        }
        features = _features_from_frame(frame, dataset_format='jdrobot')
        self.assertEqual(features[KEY_CAMERA_INTRINSIC]['names'][0], 'K_0')
        self.assertEqual(features[KEY_ACTION]['names'][-1], 'A_15')
        self.assertNotIn('names', features[KEY_RGB])
        self.assertNotIn('names', features[KEY_DEPTH])

    def test_dataset_schema_matches_kujiale(self) -> None:
        kujiale_info = {
            'robot_type': 'jdrobot',
            'fps': 20,
            'features': {
                'observation.camera_intrinsic': {'shape': [9]},
                'observation.camera_extrinsic': {'shape': [16]},
                'action': {'shape': [16]},
                'observation.images.rgb': {
                    'info': {'video.codec': 'av1'},
                },
                'observation.images.depth': {
                    'info': {'video.codec': 'av1'},
                },
                'timestamp': {},
                'frame_index': {},
                'episode_index': {},
                'index': {},
                'task_index': {},
            },
        }
        ok, reason = dataset_schema_matches(
            kujiale_info,
            dataset_format='jdrobot',
            robot_type='jdrobot',
            fps=20,
            video_vcodec='av1',
        )
        self.assertTrue(ok, reason)

    def test_dataset_schema_rejects_habitat(self) -> None:
        habitat_info = {
            'robot_type': 'habitat_diffdrive',
            'fps': 10,
            'features': {
                'observation.images.rgb': {'info': {'video.codec': 'h264'}},
                'observation.state': {'shape': [5]},
                'action': {'shape': [2]},
                'timestamp': {},
            },
        }
        ok, reason = dataset_schema_matches(
            habitat_info,
            dataset_format='jdrobot',
            robot_type='jdrobot',
            fps=20,
            video_vcodec='av1',
        )
        self.assertFalse(ok)
        self.assertIn('robot_type', reason)

    def test_collection_params_match_kujiale(self) -> None:
        self.assertEqual(JDROBOT_COLLECTION_ROS_PARAMS['dataset_format'], 'jdrobot')
        self.assertEqual(JDROBOT_COLLECTION_ROS_PARAMS['record_fps'], 20.0)
        self.assertEqual(JDROBOT_COLLECTION_ROS_PARAMS['video_vcodec'], 'av1')
        self.assertEqual(JDROBOT_COLLECTION_ROS_PARAMS['depth_min_m'], 0.0)
        self.assertEqual(JDROBOT_COLLECTION_ROS_PARAMS['depth_max_m'], 10.0)
        self.assertFalse(JDROBOT_COLLECTION_ROS_PARAMS['record_nav2'])
        self.assertTrue(JDROBOT_COLLECTION_ROS_PARAMS['record_pointcloud'])
        self.assertTrue(JDROBOT_COLLECTION_ROS_PARAMS['pointcloud_from_depth'])
        overrides = jdrobot_collection_parameters(dataset_root='/tmp/r1')
        self.assertEqual(overrides['dataset_root'], '/tmp/r1')
        self.assertEqual(overrides['dataset_format'], 'jdrobot')

    def test_lerobot_encoder_vcodec_maps_av1(self) -> None:
        self.assertEqual(lerobot_encoder_vcodec('av1'), 'libsvtav1')
        self.assertEqual(lerobot_encoder_vcodec('h264'), 'h264')

    def test_yaw_from_pose_matrix(self) -> None:
        yaw = math.pi / 3.0
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        action = pose_to_action_matrix(0.0, 0.0, 0.0, 0.0, 0.0, qz, qw)
        self.assertAlmostEqual(_yaw_from_pose_matrix(action), yaw, places=5)

    def test_sam3_bottom_ground_prior(self) -> None:
        segmenter = Sam3GroundSegmenter.__new__(Sam3GroundSegmenter)
        segmenter._bottom_ground_ratio = 0.25
        ground_mask = np.zeros((8, 4), dtype=np.uint8)
        refined = segmenter._apply_bottom_ground_prior(ground_mask)
        self.assertTrue(np.all(refined[6:, :] == 255))
        self.assertTrue(np.all(refined[:6, :] == 0))

    def test_sam3_default_prompts_cover_stairs(self) -> None:
        prompts = set(Sam3GroundSegmenter.DEFAULT_PROMPTS)
        self.assertIn('stair tread', prompts)
        self.assertIn('stair step', prompts)
        self.assertIn('stairs', prompts)
        self.assertIn('flooring', prompts)


if __name__ == '__main__':
    unittest.main()
