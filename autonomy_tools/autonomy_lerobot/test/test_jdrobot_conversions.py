# Copyright 2026 autonomy_ros contributors
# SPDX-License-Identifier: Apache-2.0

import math
import unittest

import numpy as np

from autonomy_lerobot.conversions import (
    camera_info_to_intrinsic,
    depth_to_video_rgb,
    identity_extrinsic,
    pose_to_action_matrix,
)
from autonomy_lerobot.observation import (
    KEY_ACTION,
    KEY_CAMERA_EXTRINSIC,
    KEY_CAMERA_INTRINSIC,
    KEY_DEPTH,
    KEY_RGB,
)
from autonomy_lerobot.collection_params import (
    JDROBOT_COLLECTION_ROS_PARAMS,
    jdrobot_collection_parameters,
)
from autonomy_lerobot.recorder import (
    _features_from_frame,
    dataset_schema_matches,
    lerobot_encoder_vcodec,
)


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
        self.assertFalse(JDROBOT_COLLECTION_ROS_PARAMS['record_nav2'])
        overrides = jdrobot_collection_parameters(dataset_root='/tmp/r1')
        self.assertEqual(overrides['dataset_root'], '/tmp/r1')
        self.assertEqual(overrides['dataset_format'], 'jdrobot')

    def test_lerobot_encoder_vcodec_maps_av1(self) -> None:
        self.assertEqual(lerobot_encoder_vcodec('av1'), 'libsvtav1')
        self.assertEqual(lerobot_encoder_vcodec('h264'), 'h264')


if __name__ == '__main__':
    unittest.main()
