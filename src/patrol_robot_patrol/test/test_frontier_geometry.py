import math

from patrol_robot_patrol.frontier_geometry import (
    extract_frontier_clusters,
    reachable_free_cells,
    select_frontier_goal,
    world_to_cell,
)


def make_grid(width=12, height=12):
    data = [-1] * (width * height)
    for row in range(2, 10):
        for column in range(2, 9):
            data[row * width + column] = 0
    return data


def test_extracts_connected_unknown_boundary():
    data = make_grid()

    clusters = extract_frontier_clusters(
        12, 12, data, min_cluster_size=4
    )

    assert len(clusters) == 1
    assert len(clusters[0]) >= 20
    assert all(data[index] == 0 for index in clusters[0])


def test_reachable_free_cells_do_not_cross_obstacle_wall():
    width = 8
    height = 6
    data = [0] * (width * height)
    for row in range(height):
        data[row * width + 4] = 100

    reachable = reachable_free_cells(
        width, height, data, robot_index=2 * width + 2
    )

    assert 2 * width + 3 in reachable
    assert 2 * width + 5 not in reachable


def test_selects_safe_goal_behind_frontier_ring():
    data = make_grid()
    clusters = extract_frontier_clusters(
        12, 12, data, min_cluster_size=4
    )

    goal = select_frontier_goal(
        12,
        12,
        1.0,
        0.0,
        0.0,
        data,
        5.5,
        5.5,
        clusters,
        goal_offset=1.0,
        goal_search_radius=2.0,
        robot_clearance=0.1,
        min_goal_distance=0.5,
        max_goal_distance=20.0,
    )

    assert goal is not None
    goal_index = world_to_cell(
        goal.x, goal.y, 12, 12, 1.0, 0.0, 0.0
    )
    assert data[goal_index] == 0
    assert goal.distance >= 0.5
    assert math.isfinite(goal.yaw)


def test_blacklist_rejects_only_available_goal():
    data = make_grid()
    clusters = extract_frontier_clusters(
        12, 12, data, min_cluster_size=4
    )
    first = select_frontier_goal(
        12,
        12,
        1.0,
        0.0,
        0.0,
        data,
        5.5,
        5.5,
        clusters,
        goal_offset=1.0,
        goal_search_radius=2.0,
        robot_clearance=0.1,
        min_goal_distance=0.5,
        max_goal_distance=20.0,
    )

    rejected = select_frontier_goal(
        12,
        12,
        1.0,
        0.0,
        0.0,
        data,
        5.5,
        5.5,
        clusters,
        goal_offset=1.0,
        goal_search_radius=2.0,
        robot_clearance=0.1,
        min_goal_distance=0.5,
        max_goal_distance=20.0,
        blacklisted_points=[(first.x, first.y)],
        blacklist_radius=100.0,
    )

    assert rejected is None
