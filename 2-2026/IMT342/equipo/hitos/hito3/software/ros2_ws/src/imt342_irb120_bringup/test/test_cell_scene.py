"""Verify the installed STL pose and optional calibrated dimensions."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from struct import unpack_from
from tempfile import TemporaryDirectory
from unittest import TestCase, main

import yaml
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[1]
spec = spec_from_file_location("cell_scene", ROOT / "scripts/cell_scene.py")
scene = module_from_spec(spec)
spec.loader.exec_module(scene)


class CellSceneTest(TestCase):
    def test_nominal_mesh_is_centered_on_cell_floor_and_vertical(self):
        cell = scene.CellScene(ROOT / "config/cell.yaml")
        self.assertFalse(cell.verified)
        self.assertEqual(cell.xyz, (.128, 0., 0.))
        tf = cell.static_transform(Time())
        self.assertEqual((tf.header.frame_id, tf.child_frame_id), ("base_link", "cell_frame"))
        self.assertAlmostEqual(tf.transform.translation.x, .128)
        self.assertAlmostEqual(tf.transform.translation.z, 0.)
        marker, = cell.markers(Time()).markers
        self.assertEqual(marker.type, Marker.MESH_RESOURCE)
        self.assertTrue(marker.mesh_resource.endswith("muro.STL"))
        mesh = (ROOT / "config/cell/muro.STL").read_bytes()
        count, = unpack_from("<I", mesh, 80)
        self.assertEqual(len(mesh), 84 + count * 50)
        vertices = [unpack_from("<3f", mesh, 84 + i * 50 + 12 + j * 12)
                    for i in range(count) for j in range(3)]
        transform = scene.pose_from_rpy(cell.mesh_xyz, (1.5707963267948966, 0., 0.))
        points = [tuple(sum(transform[i][j] * p[j] * cell.scale for j in range(3)) + transform[i][3]
                        for i in range(3)) for p in vertices]
        for axis, minimum, maximum in ((0, -.4, .4), (1, -.4, .4), (2, 0., .2)):
            self.assertAlmostEqual(min(p[axis] for p in points), minimum)
            self.assertAlmostEqual(max(p[axis] for p in points), maximum)
        interior_x = sorted(set(round(p[0] + cell.xyz[0], 3) for p in points))[1:3]
        self.assertEqual(interior_x, [-.222, .478])

    def test_adjusted_wall_dimensions_are_displayed_at_actual_size(self):
        config = yaml.safe_load((ROOT / "config/cell.yaml").read_text())
        config["walls"]["outer_xy_m"] = [.85, .9]
        config["walls"]["thickness_m"] = .04
        with TemporaryDirectory() as directory:
            path = Path(directory) / "cell.yaml"
            path.write_text(yaml.safe_dump(config))
            walls = scene.CellScene(path).markers(Time()).markers
        self.assertEqual(len(walls), 4)
        self.assertTrue(all(wall.type == Marker.CUBE and wall.scale.z == .2 for wall in walls))
        self.assertAlmostEqual(walls[0].pose.position.x - walls[0].scale.x / 2, -.425)
        self.assertAlmostEqual(walls[3].pose.position.y + walls[3].scale.y / 2, .45)


if __name__ == "__main__":
    main()
