from patrol_robot_patrol.localization_health import (
    localization_covariance_is_acceptable,
)


def test_localization_not_required_is_accepted_without_pose():
    assert localization_covariance_is_acceptable(
        required=False,
        pose_received=False,
        position_variance=float('inf'),
        yaw_variance=float('inf'),
        max_position_variance=0.25,
        max_yaw_variance=0.25,
    )


def test_received_converged_pose_remains_acceptable_while_stationary():
    assert localization_covariance_is_acceptable(
        required=True,
        pose_received=True,
        position_variance=0.20,
        yaw_variance=0.06,
        max_position_variance=0.25,
        max_yaw_variance=0.25,
    )


def test_missing_or_high_covariance_pose_is_rejected():
    common = {
        'required': True,
        'max_position_variance': 0.25,
        'max_yaw_variance': 0.25,
    }
    assert not localization_covariance_is_acceptable(
        pose_received=False,
        position_variance=0.10,
        yaw_variance=0.10,
        **common,
    )
    assert not localization_covariance_is_acceptable(
        pose_received=True,
        position_variance=0.26,
        yaw_variance=0.10,
        **common,
    )
    assert not localization_covariance_is_acceptable(
        pose_received=True,
        position_variance=0.10,
        yaw_variance=0.26,
        **common,
    )
