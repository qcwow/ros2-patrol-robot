"""ROS-independent localization acceptance rules."""

from __future__ import annotations


def localization_covariance_is_acceptable(
    *,
    required: bool,
    pose_received: bool,
    position_variance: float,
    yaw_variance: float,
    max_position_variance: float,
    max_yaw_variance: float,
) -> bool:
    """Accept the last converged AMCL estimate while AMCL remains active.

    AMCL is motion-update driven and normally stops publishing ``amcl_pose``
    while the robot is stationary. Message age is therefore useful telemetry,
    but it cannot by itself invalidate an otherwise converged localization.
    The caller separately gates lifecycle state, live scan/odometry, and TF.
    """
    if not required:
        return True
    return bool(
        pose_received
        and position_variance <= max_position_variance
        and yaw_variance <= max_yaw_variance
    )
