from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from math import floor
from typing import Any

import torch
from torch.utils.data import Dataset

from autonomy_internnav.train.dataset import NavDP_Base_Datset


def _normalize_source_format(source_format: str) -> str:
    aliases = {
        "v2": "v2",
        "navdp_v1": "v2",
        "v1": "v2",
        "v3": "v3",
        "lerobot_v3": "v3",
    }
    normalized = aliases.get(str(source_format))
    if normalized is None:
        raise ValueError(f"Unknown NavDP source format {source_format!r}; expected 'v2' or 'v3'.")
    return normalized


@dataclass(frozen=True)
class NavDPMixedSourceSummary:
    name: str
    format: str
    weight: float
    raw_length: int
    scheduled_length: int


class NavDPMixedSourceDataset(Dataset):
    """Weighted map-style composition of NavDP-compatible data sources.

    Each child dataset must return the same sample tuple consumed by
    `navdp_collate_fn`. The mixed dataset owns only virtual-epoch scheduling;
    source-specific loading remains inside the existing source adapters.
    """

    def __init__(self, sources: Sequence[Mapping[str, Any]], epoch_size: int | None = None, **common_kwargs):
        torch.utils.data.Dataset.__init__(self)
        if not sources:
            raise ValueError("NavDPMixedSourceDataset requires at least one source descriptor.")

        self.datasets: list[Dataset] = []
        self.source_names: list[str] = []
        self.source_formats: list[str] = []
        self.source_weights: list[float] = []

        for source_idx, source in enumerate(sources):
            source_dict = dict(source)
            source_name = str(source_dict.pop("name", f"source_{source_idx}"))
            source_format = _normalize_source_format(str(source_dict.pop("format", "")).strip())
            source_weight = float(source_dict.pop("weight", 1.0))
            if source_weight <= 0:
                raise ValueError(f"Source {source_name!r} has non-positive weight: {source_weight}")

            dataset = self._build_source_dataset(source_name, source_format, source_dict, common_kwargs)
            if len(dataset) <= 0:
                raise ValueError(f"Source {source_name!r} produced an empty dataset.")

            self.datasets.append(dataset)
            self.source_names.append(source_name)
            self.source_formats.append(source_format)
            self.source_weights.append(source_weight)

        self.epoch_size = int(epoch_size) if epoch_size is not None else sum(len(dataset) for dataset in self.datasets)
        if self.epoch_size <= 0:
            raise ValueError(f"NavDPMixedSourceDataset epoch_size must be positive, got {self.epoch_size}")

        counts = self._allocate_counts(self.source_weights, self.epoch_size)
        self._source_slots = self._build_interleaved_slots(counts, self.epoch_size)
        self._source_local_indices: list[int] = []
        source_counters = [0 for _ in self.datasets]
        for source_idx in self._source_slots:
            self._source_local_indices.append(source_counters[source_idx])
            source_counters[source_idx] += 1

        self.source_summaries = [
            NavDPMixedSourceSummary(
                name=self.source_names[i],
                format=self.source_formats[i],
                weight=self.source_weights[i],
                raw_length=len(self.datasets[i]),
                scheduled_length=counts[i],
            )
            for i in range(len(self.datasets))
        ]

    @staticmethod
    def _allocate_counts(weights: Sequence[float], total: int) -> list[int]:
        weight_sum = float(sum(weights))
        if weight_sum <= 0:
            raise ValueError("At least one source weight must be positive.")

        ideal_counts = [float(total) * float(weight) / weight_sum for weight in weights]
        counts = [int(floor(count)) for count in ideal_counts]
        remaining = total - sum(counts)
        remainders = sorted(
            enumerate(ideal_counts),
            key=lambda item: (item[1] - floor(item[1]), weights[item[0]]),
            reverse=True,
        )
        for source_idx, _ in remainders[:remaining]:
            counts[source_idx] += 1
        return counts

    @staticmethod
    def _build_interleaved_slots(counts: Sequence[int], total: int) -> list[int]:
        used = [0 for _ in counts]
        slots: list[int] = []
        for step in range(total):
            source_idx = max(
                range(len(counts)),
                key=lambda idx: (((step + 1) * counts[idx] / total) - used[idx], -idx),
            )
            slots.append(source_idx)
            used[source_idx] += 1
        return slots

    @staticmethod
    def _build_source_dataset(
        source_name: str,
        source_format: str,
        source_kwargs: dict[str, Any],
        common_kwargs: Mapping[str, Any],
    ) -> Dataset:
        kwargs = dict(common_kwargs)
        kwargs.update(source_kwargs)

        root_dir = kwargs.pop("root_dir", kwargs.pop("root_dirs", None))
        if root_dir is None:
            raise ValueError(f"Source {source_name!r} is missing `root_dir`.")

        source_format = _normalize_source_format(source_format)

        if source_format == "v2":
            preload_path = kwargs.pop("preload_path", kwargs.pop("dataset_navdp", False))
            return NavDP_Base_Datset(root_dir, preload_path=preload_path, **kwargs)

        if source_format == "v3":
            from autonomy_internnav.train.dataset_lerobot_v3 import NavDP_LerobotV3_Dataset

            kwargs.pop("preload_path", None)
            kwargs.pop("dataset_navdp", None)
            return NavDP_LerobotV3_Dataset(root_dir, **kwargs)

    def __len__(self):
        return self.epoch_size

    def __getitem__(self, index):
        schedule_index = int(index) % self.epoch_size
        source_idx = self._source_slots[schedule_index]
        source_dataset = self.datasets[source_idx]
        local_index = self._source_local_indices[schedule_index] % len(source_dataset)
        return source_dataset[local_index]
