from pathlib import Path
import json
import tempfile
import unittest

from patrol_robot_patrol.map_artifacts import (
    validate_map_artifact,
    validate_map_manifest,
)


class MapArtifactsTest(unittest.TestCase):
    def test_validates_relative_pgm_and_dimensions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'map.pgm').write_bytes(b'P5\n# test\n4 3\n255\n' + b'\x00' * 12)
            (root / 'map.yaml').write_text(
                'image: map.pgm\nresolution: 0.05\n'
                'origin: [-1.0, -2.0, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n',
                encoding='utf-8',
            )
            result = validate_map_artifact(root / 'map.yaml')
            self.assertTrue(result['valid'])
            self.assertEqual(result['image_width'], 4)
            self.assertEqual(result['image_height'], 3)
            self.assertEqual(result['map_width_meters'], 0.2)

    def test_rejects_missing_image(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.yaml'
            path.write_text(
                'image: missing.pgm\nresolution: 0.05\n'
                'origin: [0, 0, 0]\noccupied_thresh: 0.65\n'
                'free_thresh: 0.25\n',
                encoding='utf-8',
            )
            with self.assertRaises(ValueError):
                validate_map_artifact(path)

    def test_rejects_inverted_thresholds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
            (root / 'map.yaml').write_text(
                'image: map.pgm\nresolution: 0.05\n'
                'origin: [0, 0, 0]\noccupied_thresh: 0.2\n'
                'free_thresh: 0.8\n',
                encoding='utf-8',
            )
            with self.assertRaises(ValueError):
                validate_map_artifact(root / 'map.yaml')

    def test_manifest_must_match_profile_and_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
            yaml_path = root / 'map.yaml'
            yaml_path.write_text(
                'image: map.pgm\nresolution: 0.05\n'
                'origin: [0, 0, 0]\noccupied_thresh: 0.65\n'
                'free_thresh: 0.25\n',
                encoding='utf-8',
            )
            manifest = validate_map_artifact(yaml_path)
            manifest['profile'] = 'real_car'
            yaml_path.with_suffix('.manifest.json').write_text(
                json.dumps(manifest), encoding='utf-8'
            )
            result = validate_map_manifest(yaml_path, 'real_car')
            self.assertTrue(result['manifest_verified'])
            (root / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
            with self.assertRaises(ValueError):
                validate_map_manifest(yaml_path, 'real_car')


if __name__ == '__main__':
    unittest.main()
