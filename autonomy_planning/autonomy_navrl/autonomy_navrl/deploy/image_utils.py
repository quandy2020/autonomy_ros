"""ROS Image message conversion helpers."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image


def image_to_numpy(image: Image) -> np.ndarray:
    import cv2

    if image.encoding in ('rgb8', 'bgr8'):
        channels = 3
        dtype = np.uint8
    elif image.encoding == 'mono8':
        channels = 1
        dtype = np.uint8
    else:
        raise ValueError(f'Unsupported RGB encoding: {image.encoding}')
    array = np.frombuffer(image.data, dtype=dtype)
    array = array.reshape((image.height, image.width, channels))
    if image.encoding == 'bgr8':
        array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
    return array


def depth_to_meters(image: Image) -> np.ndarray:
    if image.encoding == '32FC1':
        array = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, image.width)
        return array.copy()
    if image.encoding == '16UC1':
        array = np.frombuffer(image.data, dtype=np.uint16).reshape(image.height, image.width)
        return array.astype(np.float32) / 1000.0
    raise ValueError(f'Unsupported depth encoding: {image.encoding}')
