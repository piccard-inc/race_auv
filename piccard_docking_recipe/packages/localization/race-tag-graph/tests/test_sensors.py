"""The estimator's sensor boundary (#265, research #36): it reads only EKF odometry, the 3D detections and /tf_static,
refuses truth and the lab fuser's output by raising, and no estimator module imports the evaluation side."""
import ast
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import synthetic
from race_tag_graph import factors, sensors

PACKAGE = Path(__file__).resolve().parent.parent
ESTIMATOR_MODULES = ("sensors", "factors", "solvers", "dock", "geometry", "metrics", "writers")
FORBIDDEN_TOPICS = ("/piccard/ground_truth/upstream_docking_pose", "/piccard/ground_truth/tf_static",
                    "/race_auv/stonefish/odometry", "/race_station/stonefish/odometry",
                    "/race_station/dock_point/pose", "/race_station/contacts/station_frame", "/tf")


class SensorBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.telemetry = synthetic.write(Path(cls.tmp.name) / "telemetry.jsonl", duration=40.0)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_only_the_sensor_topics_are_read(self):
        stream = sensors.read_telemetry(self.telemetry)
        self.assertTrue(stream.odometry and stream.detections and stream.tf_static)
        self.assertEqual({a.camera for a in stream.detections}, {"cam_front", "cam_down"})
        self.assertIn("odometry covariance not recorded; the configured odometry noise model applies", stream.notes)
        self.assertIn("tag orientations not recorded; position-only landmark factors", stream.notes)

    def test_every_forbidden_record_kind_is_refused(self):
        for kind in [*sensors.FORBIDDEN_KINDS, "planner", "thruster"]:
            with self.subTest(kind=kind), self.assertRaises(sensors.TruthAccessError):
                sensors.read_telemetry(self.telemetry, kinds={"odometry", kind})

    def test_every_forbidden_topic_is_refused(self):
        for topic in FORBIDDEN_TOPICS:
            with self.subTest(topic=topic), self.assertRaises(sensors.TruthAccessError):
                sensors.check_request(topics=[topic])
        sensors.check_request(kinds=sensors.SENSOR_KINDS, topics=sensors.SENSOR_TOPICS)

    def test_a_sensor_kind_on_a_truth_topic_raises(self):
        path = Path(self.tmp.name) / "relabelled.jsonl"
        path.write_text(self.telemetry.read_text() +
                        '{"kind":"odometry","t":99.0,"topic":"/race_auv/stonefish/odometry","stamp":1099.0,'
                        '"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":1}\n')
        with self.assertRaises(sensors.TruthAccessError):
            sensors.read_telemetry(path)

    def test_truth_records_are_skipped_without_being_parsed(self):
        path = Path(self.tmp.name) / "broken-truth.jsonl"
        path.write_text(self.telemetry.read_text() + '{"kind":"gt_auv", this is not json\n'
                        '{"kind": "dock", neither is this\n')
        self.assertEqual(len(sensors.read_telemetry(path).odometry),
                         len(sensors.read_telemetry(self.telemetry).odometry))

    def test_the_builder_takes_only_a_sensor_stream(self):
        with self.assertRaises(TypeError):
            factors.build({"odometry": []}, factors.load_config())

    def test_no_estimator_module_imports_the_evaluation_side(self):
        for module in ESTIMATOR_MODULES:
            tree = ast.parse((PACKAGE / "race_tag_graph" / f"{module}.py").read_text())
            imported = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    imported.add((node.module or "").split(".")[-1])
                    imported.update(alias.name for alias in node.names)
                elif isinstance(node, ast.Import):
                    imported.update(alias.name.split(".")[-1] for alias in node.names)
            with self.subTest(module=module):
                self.assertFalse(imported & {"truth", "evaluate", "read_truth", "scoring"}, imported)


class RosbagConversionTests(unittest.TestCase):
    """The rosbag2 reader is UNTESTED against a real bag; these cover its message conversions only."""

    @staticmethod
    def header(sec, nanosec, frame):
        return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id=frame)

    def test_odometry(self):
        msg = SimpleNamespace(header=self.header(10, 500_000_000, "race_auv/odom"), pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=-3.0),
                                 orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
            covariance=[0.0] * 36))
        record = sensors.odometry_from_msg(msg)
        self.assertEqual((record.stamp, record.position, record.covariance), (10.5, (1.0, 2.0, -3.0), None))

    def test_detection3d_array(self):
        pose = SimpleNamespace(position=SimpleNamespace(x=0.1, y=0.2, z=3.0),
                               orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        detection = SimpleNamespace(id="tag36h11:146", results=[SimpleNamespace(pose=SimpleNamespace(pose=pose))])
        empty = SimpleNamespace(id="tag36h11:541", results=[])
        msg = SimpleNamespace(header=self.header(5, 0, "race_auv/cam_front"), detections=[detection, empty])
        array = sensors.detections_from_msg(msg, "cam_front")
        self.assertEqual((array.stamp, array.camera, array.frame_id), (5.0, "cam_front", "race_auv/cam_front"))
        self.assertEqual([(d.tag, d.position) for d in array.detections], [("tag36h11:146", (0.1, 0.2, 3.0))])

    def test_tf_static(self):
        transform = SimpleNamespace(header=self.header(0, 0, "race_auv/base_link"),
                                    child_frame_id="race_auv/nose_tip_link", transform=SimpleNamespace(
                                        translation=SimpleNamespace(x=0.7, y=0.0, z=0.0),
                                        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.assertEqual(sensors.tf_static_from_msg(SimpleNamespace(transforms=[transform])),
                         ({"parent": "race_auv/base_link", "child": "race_auv/nose_tip_link",
                           "xyz": [0.7, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]},))

    def test_a_corner_detection_topic_is_refused_not_misread(self):
        sensors.check_detection_type("/cam_front/apriltag_detection/detections3d", "vision_msgs/msg/Detection3DArray")
        with self.assertRaisesRegex(ValueError, "corner projection factors"):
            sensors.check_detection_type("/cam_front/apriltag_detection/detections3d",
                                         "apriltag_msgs/msg/AprilTagDetectionArray")

    def test_a_forbidden_bag_topic_is_refused_before_the_bag_is_opened(self):
        with self.assertRaises(sensors.TruthAccessError):
            sensors.read_rosbag2("no-such-bag", odometry_topic="/race_auv/stonefish/odometry")


if __name__ == "__main__":
    unittest.main()
