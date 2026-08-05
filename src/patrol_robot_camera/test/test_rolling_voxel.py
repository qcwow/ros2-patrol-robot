import numpy as np
import pytest

from patrol_robot_camera.rolling_voxel import RollingVoxelMap


def make_map(**overrides):
    parameters = {
        'voxel_size': 0.1,
        'half_extent': 1.0,
        'max_voxels': 100,
        'min_observations': 2,
        'stale_seconds': 5.0,
    }
    parameters.update(overrides)
    return RollingVoxelMap(**parameters)


def test_voxel_requires_repeated_observation():
    voxel_map = make_map()
    points = np.array([[0.12, 0.01, 0.20]], dtype=np.float32)
    colors = np.array([[10, 20, 30]], dtype=np.uint8)

    voxel_map.insert(points, colors, np.zeros(3), 1.0)
    assert voxel_map.snapshot()[0].shape == (0, 3)

    voxel_map.insert(points, colors, np.zeros(3), 2.0)
    stored_points, stored_colors = voxel_map.snapshot()
    np.testing.assert_allclose(stored_points, points)
    np.testing.assert_array_equal(stored_colors, colors)


def test_window_follows_robot_and_discards_old_volume():
    voxel_map = make_map(min_observations=1)
    colors = np.zeros((2, 3), dtype=np.uint8)
    voxel_map.insert(
        np.array([[0.0, 0.0, 0.0], [0.9, 0.0, 0.0]], dtype=np.float32),
        colors,
        np.zeros(3),
        1.0,
    )
    voxel_map.insert(
        np.array([[1.8, 0.0, 0.0]], dtype=np.float32),
        colors[:1],
        np.array([1.1, 0.0, 0.0], dtype=np.float32),
        2.0,
    )

    points, _ = voxel_map.snapshot()
    assert points.shape == (2, 3)
    assert not np.any(np.isclose(points[:, 0], 0.0))


def test_stale_and_capacity_pruning_are_bounded():
    voxel_map = make_map(
        min_observations=1,
        max_voxels=2,
        stale_seconds=1.0,
    )
    colors = np.zeros((3, 3), dtype=np.uint8)
    voxel_map.insert(
        np.array([[0.1, 0, 0], [0.2, 0, 0], [0.3, 0, 0]], dtype=np.float32),
        colors,
        np.zeros(3),
        1.0,
    )
    assert voxel_map.candidate_voxel_count == 2

    voxel_map.insert(
        np.empty((0, 3), dtype=np.float32),
        np.empty((0, 3), dtype=np.uint8),
        np.zeros(3),
        3.0,
    )
    assert voxel_map.candidate_voxel_count == 0


@pytest.mark.parametrize(
    'override',
    [
        {'voxel_size': 0.0},
        {'half_extent': 0.05},
        {'max_voxels': 0},
        {'min_observations': 0},
        {'stale_seconds': 0.0},
    ],
)
def test_invalid_configuration_is_rejected(override):
    with pytest.raises(ValueError):
        make_map(**override)
