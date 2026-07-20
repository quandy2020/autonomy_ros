#
# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""SAM3-based ground segmentation for offline bag conversion."""

from __future__ import annotations

import os
from pathlib import Path
from urllib.request import urlretrieve

import cv2
import numpy as np

try:
    import sam3
    import torch
    from sam3 import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor

    _HAS_SAM3 = True
except ImportError:  # pragma: no cover - optional dependency
    _HAS_SAM3 = False


def _share_model_dir() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('autonomy_lerobot')) / 'model'
    except Exception:
        return None


class Sam3GroundSegmenter:
    """Segments traversable ground with SAM3.

    The segmenter keeps all SAM3-specific configuration inside a single class so
    offline conversion code only depends on a small public API:

    - ``segment_ground()``
    - ``semantic_rgb_from_ground_mask()``
    - ``nonground_mask()``
    """

    DEFAULT_PROMPTS = (
        'floor',
        'ground',
        'flooring',
        'stair tread',
        'stair step',
        'stairs',
        'step',
        'pavement',
        'walkway',
    )
    DEFAULT_CHECKPOINT_NAME = 'sam3.pt'
    DEFAULT_MIN_CHECKPOINT_BYTES = 3_000_000_000
    DEFAULT_COMPONENT_MIN_AREA = 1500
    DEFAULT_CLOSE_KERNEL_SIZE = 15
    DEFAULT_OPEN_KERNEL_SIZE = 7
    DEFAULT_DILATE_KERNEL_SIZE = 5
    DEFAULT_DILATE_ITERATIONS = 2
    DEFAULT_BOTTOM_GROUND_RATIO = 0.10
    DEFAULT_HF_ENDPOINT = 'https://hf-mirror.com'
    DEFAULT_BPE_NAME = 'bpe_simple_vocab_16e6.txt.gz'
    DEFAULT_BPE_URL = 'https://raw.githubusercontent.com/openai/CLIP/main/clip/bpe_simple_vocab_16e6.txt.gz'

    def __init__(
        self,
        *,
        device: str = 'cuda',
        confidence: float = 0.3,
        prompts: tuple[str, ...] = DEFAULT_PROMPTS,
        checkpoint_path: str | None = None,
        checkpoint_name: str = DEFAULT_CHECKPOINT_NAME,
        bpe_path: str | None = None,
        bpe_name: str = DEFAULT_BPE_NAME,
        bpe_url: str = DEFAULT_BPE_URL,
        min_checkpoint_bytes: int = DEFAULT_MIN_CHECKPOINT_BYTES,
        component_min_area: int = DEFAULT_COMPONENT_MIN_AREA,
        close_kernel_size: int = DEFAULT_CLOSE_KERNEL_SIZE,
        open_kernel_size: int = DEFAULT_OPEN_KERNEL_SIZE,
        dilate_kernel_size: int = DEFAULT_DILATE_KERNEL_SIZE,
        dilate_iterations: int = DEFAULT_DILATE_ITERATIONS,
        bottom_ground_ratio: float = DEFAULT_BOTTOM_GROUND_RATIO,
        hf_endpoint: str | None = None,
    ) -> None:
        """Initializes the SAM3 segmenter.

        Args:
            device: Torch device string used for inference.
            confidence: Confidence threshold passed to ``Sam3Processor``.
            prompts: Text prompts used to query ground-like regions.
            checkpoint_path: Optional explicit ``sam3.pt`` path.
            checkpoint_name: Default checkpoint filename searched under
                ``autonomy_lerobot/model``.
            bpe_path: Optional explicit tokenizer vocabulary path.
            bpe_name: Default tokenizer vocabulary filename.
            bpe_url: Download URL used when the vocabulary is missing locally.
            min_checkpoint_bytes: Minimum accepted checkpoint size.
            component_min_area: Minimum connected-component area to keep.
            close_kernel_size: Morphological closing kernel size.
            open_kernel_size: Morphological opening kernel size.
            dilate_kernel_size: Dilation kernel size for expanding ground mask.
            dilate_iterations: Dilation iterations after close/open cleanup.
            bottom_ground_ratio: Bottom image ratio forced to ground, useful for
                robot-footprint / stair-edge noise suppression.
            hf_endpoint: Optional Hugging Face endpoint exported before model
                construction. Pass ``None`` to keep the current environment.
        """
        if not _HAS_SAM3:
            raise RuntimeError(
                'sam3 is not installed. Run `pip3 install sam3 torch` before '
                'using --sam3-enable, or disable it with `sam3_enable:=false`.'
            )
        if hf_endpoint:
            os.environ['HF_ENDPOINT'] = hf_endpoint
        self._package_root = Path(__file__).resolve().parents[1]
        self._share_model_dir = _share_model_dir()
        self._prompts = tuple(dict.fromkeys(prompt.strip() for prompt in prompts if prompt.strip()))
        self._checkpoint_name = checkpoint_name
        self._bpe_name = bpe_name
        self._bpe_url = bpe_url
        self._min_checkpoint_bytes = int(min_checkpoint_bytes)
        self._component_min_area = int(component_min_area)
        self._close_kernel = np.ones(
            (int(close_kernel_size), int(close_kernel_size)), dtype=np.uint8
        )
        self._open_kernel = np.ones(
            (int(open_kernel_size), int(open_kernel_size)), dtype=np.uint8
        )
        self._dilate_kernel = np.ones(
            (int(dilate_kernel_size), int(dilate_kernel_size)), dtype=np.uint8
        )
        self._dilate_iterations = max(0, int(dilate_iterations))
        self._bottom_ground_ratio = float(np.clip(bottom_ground_ratio, 0.0, 1.0))
        self._processor = self._build_processor(
            device=device,
            confidence=confidence,
            checkpoint_path=checkpoint_path,
            bpe_path=bpe_path,
        )

    def segment_ground(self, rgb_image: np.ndarray) -> np.ndarray:
        """Returns a uint8 mask where 255 means segmented ground."""
        from PIL import Image

        h, w = rgb_image.shape[:2]
        state = self._processor.set_image(Image.fromarray(rgb_image))
        masks = []
        for prompt in self._prompts:
            prompt_state = self._processor.set_text_prompt(state=state, prompt=prompt)
            prompt_masks = prompt_state.get('masks')
            if prompt_masks is not None and len(prompt_masks) > 0:
                masks.append(prompt_masks)
        if not masks:
            return np.zeros((h, w), dtype=np.uint8)
        merged = np.concatenate(
            [m.squeeze(1).detach().cpu().numpy() for m in masks],
            axis=0,
        )
        return self._postprocess(merged, (h, w))

    @staticmethod
    def semantic_rgb_from_ground_mask(ground_mask: np.ndarray) -> np.ndarray:
        """Encodes ground as green and obstacle as red."""
        semantic = np.zeros((*ground_mask.shape, 3), dtype=np.uint8)
        is_ground = ground_mask > 0
        semantic[is_ground] = (0, 255, 0)
        semantic[~is_ground] = (255, 0, 0)
        return semantic

    @staticmethod
    def nonground_mask(ground_mask: np.ndarray) -> np.ndarray:
        """Returns a boolean mask for obstacle-only pointcloud generation."""
        return ground_mask == 0

    def _build_processor(
        self,
        *,
        device: str,
        confidence: float,
        checkpoint_path: str | None,
        bpe_path: str | None,
    ) -> Sam3Processor:
        if device.startswith('cuda') and torch.cuda.is_available():
            gpu_id = int(device.split(':')[-1]) if ':' in device else 0
            torch.cuda.set_device(gpu_id)
        ckpt_path = self._resolve_checkpoint(checkpoint_path)
        bpe_path = self._resolve_bpe_path(bpe_path)
        model = build_sam3_image_model(
            bpe_path=bpe_path,
            enable_segmentation=True,
            checkpoint_path=ckpt_path,
            load_from_HF=False,
        )
        return Sam3Processor(model, confidence_threshold=confidence)

    def _postprocess(self, masks: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
        combined = np.zeros(shape, dtype=np.uint8)
        for mask in masks:
            binary = (mask > 0.5).astype(np.uint8)
            n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
                binary, connectivity=8
            )
            for idx in range(1, n_labels):
                if stats[idx, cv2.CC_STAT_AREA] >= self._component_min_area:
                    combined[labels == idx] = 255
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self._close_kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, self._open_kernel)
        combined = self._apply_bottom_ground_prior(combined)
        if self._dilate_iterations > 0:
            combined = cv2.dilate(
                combined,
                self._dilate_kernel,
                iterations=self._dilate_iterations,
            )
        return combined

    def _apply_bottom_ground_prior(self, ground_mask: np.ndarray) -> np.ndarray:
        if self._bottom_ground_ratio <= 0.0:
            return ground_mask
        combined = ground_mask.copy()
        height = combined.shape[0]
        start_row = int(height * (1.0 - self._bottom_ground_ratio))
        combined[start_row:, :] = 255
        return combined

    def _candidate_checkpoints(self) -> list[Path]:
        candidates = [self._package_root / 'model' / self._checkpoint_name]
        if self._share_model_dir is not None:
            candidates.append(self._share_model_dir / self._checkpoint_name)
        if _HAS_SAM3:
            candidates.append(
                Path(sam3.__file__).resolve().parent.parent / 'model' / self._checkpoint_name
            )
        return candidates

    def _candidate_bpe_paths(self) -> list[Path]:
        candidates: list[Path] = []
        sam3_pkg = Path(os.path.dirname(sam3.__file__) or '')
        candidates.append(sam3_pkg / 'assets' / self._bpe_name)
        candidates.append(self._package_root / 'model' / self._bpe_name)
        if self._share_model_dir is not None:
            candidates.append(self._share_model_dir / self._bpe_name)
        candidates.append(Path.home() / '.cache' / 'autonomy_lerobot' / self._bpe_name)
        return candidates

    def _resolve_checkpoint(self, explicit_path: str | None = None) -> str:
        if explicit_path:
            candidate = Path(explicit_path)
            if self._is_valid_checkpoint(candidate):
                return str(candidate)
            if not candidate.is_file():
                raise RuntimeError(
                    f'sam3 checkpoint does not exist in this runtime: {candidate}'
                )
            raise RuntimeError(
                f'sam3 checkpoint is too small or incomplete: {candidate} '
                f'(size={candidate.stat().st_size} bytes, '
                f'min={self._min_checkpoint_bytes} bytes)'
            )

        for candidate in self._candidate_checkpoints():
            if self._is_valid_checkpoint(candidate):
                return str(candidate)

        raise RuntimeError(
            'sam3 checkpoint not found. Provide --sam3-checkpoint or place sam3.pt '
            'under `model/` in the source tree or `share/autonomy_lerobot/model/` '
            'after install.'
        )

    def _resolve_bpe_path(self, explicit_path: str | None = None) -> str:
        if explicit_path:
            candidate = Path(explicit_path)
            if candidate.is_file():
                return str(candidate)
            raise RuntimeError(f'sam3 tokenizer vocab does not exist: {candidate}')

        for candidate in self._candidate_bpe_paths():
            if candidate.is_file():
                return str(candidate)

        cache_path = Path.home() / '.cache' / 'autonomy_lerobot' / self._bpe_name
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            urlretrieve(self._bpe_url, cache_path)
        except Exception as exc:
            raise RuntimeError(
                'sam3 tokenizer vocab not found. Download '
                f'`{self._bpe_name}` from {self._bpe_url} and place it under '
                '`autonomy_lerobot/model/`, or ensure the runtime can access the URL.'
            ) from exc
        return str(cache_path)

    def _is_valid_checkpoint(self, path: Path) -> bool:
        return path.is_file() and path.stat().st_size >= self._min_checkpoint_bytes
