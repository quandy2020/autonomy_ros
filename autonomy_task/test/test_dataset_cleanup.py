"""Tests for dataset temp-dir cleanup."""

from pathlib import Path

from autonomy_task.dataset_cleanup import cleanup_dataset_tree


def test_cleanup_orphan_tmp_dirs(tmp_path: Path) -> None:
    root = tmp_path / 'robot1'
    (root / 'meta').mkdir(parents=True)
    orphan = root / 'tmpabc123'
    orphan.mkdir()
    (orphan / 'observation.images.rgb_streaming.mp4').write_bytes(b'12345')
    (root / 'data').mkdir()

    removed, freed = cleanup_dataset_tree(root)
    assert removed == 1
    assert freed == 5
    assert not orphan.exists()
    assert (root / 'meta').is_dir()


def test_cleanup_skips_non_tmp_dirs(tmp_path: Path) -> None:
    root = tmp_path / 'robot1'
    (root / 'meta').mkdir(parents=True)
    keep = root / 'videos'
    keep.mkdir()
    (keep / 'clip.mp4').write_bytes(b'x')

    removed, _ = cleanup_dataset_tree(root)
    assert removed == 0
    assert keep.is_dir()


def test_cleanup_missing_root(tmp_path: Path) -> None:
    assert cleanup_dataset_tree(tmp_path / 'missing') == (0, 0)
