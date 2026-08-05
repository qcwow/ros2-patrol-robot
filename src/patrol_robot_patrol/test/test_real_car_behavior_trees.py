from pathlib import Path
import xml.etree.ElementTree as ET


BEHAVIOR_TREE_DIR = Path(__file__).parents[1] / 'behavior_trees'


def _tags(path: Path) -> set[str]:
    return {element.tag for element in ET.parse(path).iter()}


def test_real_car_default_trees_have_no_displacing_recoveries():
    forbidden = {'Spin', 'BackUp', 'DriveOnHeading'}
    for filename in (
        'navigate_to_pose_no_spin.xml',
        'navigate_through_poses_no_recovery.xml',
    ):
        assert _tags(BEHAVIOR_TREE_DIR / filename).isdisjoint(forbidden)


def test_through_poses_uses_explicit_humble_goal_checker():
    root = ET.parse(
        BEHAVIOR_TREE_DIR / 'navigate_through_poses_no_recovery.xml'
    )
    follow_path = root.find('.//FollowPath')
    assert follow_path is not None
    assert follow_path.attrib['goal_checker_id'] == 'transit_goal_checker'
