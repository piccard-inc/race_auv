"""The factor builder: the only place the formulation lives (#265; research #35, comments 18824480 and 18824538).

From a SensorStream (and nothing else) and the YAML config it builds, in time order:
- keyframes: every image with detections, and at least every keyframes.max_interval_s; each keyframe's initial value
  is the EKF pose interpolated at its stamp;
- a prior on the first keyframe, fixing the gauge at the EKF's first pose;
- one BetweenFactorPose3 per consecutive keyframe pair from the EKF's own increment, so the already-fused odometry
  enters once, as motion, and never as an absolute measurement;
- each tag a landmark (Point3 in the odom frame), initialised at its first sighting and constrained by every later
  3D detection through the camera extrinsic from /tf_static. The detection's noise is anisotropic: tight across the
  camera ray, looser along it. A robust kernel and a causal gate (against the tag's running median EKF-projected
  position) reject outliers; a gated detection is recorded, not added. No range-scale parameter is estimated: a
  range convention error shows in the residuals, it is not absorbed;
- optionally (orientation.enabled, off by default) an orientation landmark per tag (Rot3) constrained by the
  detection's tag orientation inside orientation.max_range_m. The collector's telemetry records no tag orientation,
  so this path needs a bag (or another source) that carries it.

The steps are what the solvers consume: the batch solver all at once, the ISAM2 replay one keyframe at a time.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from pathlib import Path

import gtsam
import numpy as np
import yaml
from gtsam.symbol_shorthand import L, O, X

from .geometry import PoseSeries, pose3, rot3, static_chain
from .sensors import SensorStream

CONFIG_SCHEMA = "piccard.race-tag-graph.config/v1"
LAYOUT_SCHEMA = "piccard.race-tag-graph.station-layout/v1"
DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "default.yaml"


class ConfigError(ValueError):
    """The config, the layout or the stream does not support the requested formulation."""


def load_config(path=DEFAULT_CONFIG) -> dict:
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or config.get("schema") != CONFIG_SCHEMA:
        raise ConfigError(f"{path}: schema must be {CONFIG_SCHEMA}")
    config["_dir"] = str(path.resolve().parent)
    return config


def load_layout(config: dict) -> dict:
    """{tag: {"position": array in the dock frame, "rotation": Rot3 tag in the dock frame}}."""
    path = Path(config.get("_dir", ".")) / config["dock"]["layout"]
    layout = yaml.safe_load(path.read_text())
    if layout.get("schema") != LAYOUT_SCHEMA:
        raise ConfigError(f"{path}: schema must be {LAYOUT_SCHEMA}")
    return {tag: {"position": np.asarray(item["position_m"], dtype=float), "size": float(item["size_m"]),
                  "rotation": gtsam.Rot3.RzRyRx(*item.get("rpy_rad", [0.0, 0.0, 0.0]))}
            for tag, item in layout["tags"].items()}


@dataclass
class Keyframe:
    index: int
    stamp: float
    ekf: gtsam.Pose3


@dataclass
class Observation:
    keyframe: int
    stamp: float
    camera: str
    tag: str
    measured: np.ndarray        # in the camera's optical frame
    base_point: np.ndarray      # the same point in base_link, through the camera extrinsic
    range_m: float
    accepted: bool
    reason: str | None = None   # why a detection was not added
    first: bool = False         # the landmark's first sighting
    range_used: bool = False    # its range constrains the graph (else bearing only)


@dataclass
class Step:
    keyframe: int
    factors: list = field(default_factory=list)
    new_landmarks: dict = field(default_factory=dict)      # tag -> base_point of its first sighting at this keyframe
    new_orientations: dict = field(default_factory=dict)   # tag -> its first accepted orientation, in base_link
    odometry: gtsam.Pose3 | None = None                    # EKF increment from the previous keyframe


@dataclass
class Problem:
    config: dict
    keyframes: list
    steps: list
    landmarks: dict             # tag -> landmark index (key L(index))
    observations: list
    extrinsics: dict            # camera -> base_link->camera Pose3
    counts: dict
    notes: tuple

    def graph(self) -> gtsam.NonlinearFactorGraph:
        graph = gtsam.NonlinearFactorGraph()
        for step in self.steps:
            for factor in step.factors:
                graph.add(factor)
        return graph

    def initial(self) -> gtsam.Values:
        """The EKF's poses, and each landmark at its first sighting projected through the EKF pose."""
        values = gtsam.Values()
        for keyframe in self.keyframes:
            values.insert(X(keyframe.index), keyframe.ekf)
        for step in self.steps:
            pose = self.keyframes[step.keyframe].ekf
            for tag, base_point in step.new_landmarks.items():
                values.insert(L(self.landmarks[tag]), pose.transformFrom(base_point))
            for tag, base_rotation in step.new_orientations.items():
                values.insert(O(self.landmarks[tag]), pose.rotation().compose(base_rotation))
        return values


# ----------------------------------------------------------------------------- noise models and factors
def _robust(config: dict, model):
    robust = config["detections"].get("robust") or {"kernel": "none"}
    kernel = robust.get("kernel", "none")
    if kernel == "none":
        return model
    estimators = {"huber": gtsam.noiseModel.mEstimator.Huber, "cauchy": gtsam.noiseModel.mEstimator.Cauchy}
    if kernel not in estimators:
        raise ConfigError(f"detections.robust.kernel must be huber, cauchy or none, not {kernel!r}")
    return gtsam.noiseModel.Robust.Create(estimators[kernel].Create(float(robust["k"])), model)


def range_sigma(config: dict, range_m: float, size_m: float) -> float:
    cfg = config["detections"]
    return cfg["range_sigma_m"] + cfg["range_px_sigma"] * range_m ** 2 / (cfg["focal_px"] * size_m)


def range_used(config: dict, range_m: float, size_m: float) -> bool:
    return range_sigma(config, range_m, size_m) <= config["detections"]["range_informative_sigma_m"]


def detection_noise(config: dict, measured: np.ndarray, camera_rotation: np.ndarray, size_m: float):
    """Anisotropic in the camera frame, expressed in base_link: across the ray a bearing error, along it a range
    error growing with range squared over the tag's size (a pose solved from the tag's apparent size). Where that
    range error exceeds detections.range_informative_sigma_m the range is not used (bearing only): far ranges carry
    a systematic error that averaging does not remove. No scale is estimated: a range convention error stays in the
    residuals."""
    cfg = config["detections"]
    range_m = float(np.linalg.norm(measured))
    ray = measured / range_m
    lateral = cfg["bearing_sigma_rad"] * range_m + cfg["lateral_floor_m"]
    along = (range_sigma(config, range_m, size_m) if range_used(config, range_m, size_m)
             else cfg["bearing_only_range_sigma_m"])   # beyond: the range is not used; the bearing is
    camera = lateral ** 2 * np.eye(3) + (along ** 2 - lateral ** 2) * np.outer(ray, ray)
    base = camera_rotation @ camera @ camera_rotation.T
    return _robust(config, gtsam.noiseModel.Gaussian.Covariance(base))


def landmark_factor(pose_key, landmark_key, base_point: np.ndarray, noise) -> gtsam.CustomFactor:
    """The landmark seen from the keyframe, in base_link, against the detection mapped into base_link."""
    measured = np.array(base_point, dtype=float)

    def error(this, values, jacobians):
        pose = values.atPose3(this.keys()[0])
        point = values.atPoint3(this.keys()[1])
        h_pose, h_point = np.zeros((3, 6), order="F"), np.zeros((3, 3), order="F")
        predicted = pose.transformTo(point, h_pose, h_point)
        if jacobians is not None:
            jacobians[0], jacobians[1] = h_pose, h_point
        return predicted - measured

    return gtsam.CustomFactor(noise, [pose_key, landmark_key], error)


def orientation_residual(pose: gtsam.Pose3, tag_rotation: gtsam.Rot3, camera: gtsam.Rot3,
                         measured: gtsam.Rot3) -> np.ndarray:
    predicted = camera.inverse().compose(pose.rotation().inverse()).compose(tag_rotation)
    return gtsam.Rot3.Logmap(measured.between(predicted))


def orientation_factor(pose_key, orientation_key, camera: gtsam.Rot3, measured: gtsam.Rot3,
                       noise) -> gtsam.CustomFactor:
    """Opt-in: the tag's orientation (odom frame) seen from the keyframe, against the detected orientation.
    Jacobians by central differences on the manifolds."""
    step = 1e-6

    def error(this, values, jacobians):
        pose, tag = values.atPose3(this.keys()[0]), values.atRot3(this.keys()[1])
        value = orientation_residual(pose, tag, camera, measured)
        if jacobians is not None:
            h_pose, h_tag = np.zeros((3, 6), order="F"), np.zeros((3, 3), order="F")
            for i in range(6):
                delta = np.zeros(6)
                delta[i] = step
                h_pose[:, i] = (orientation_residual(pose.retract(delta), tag, camera, measured) -
                                orientation_residual(pose.retract(-delta), tag, camera, measured)) / (2 * step)
            for i in range(3):
                delta = np.zeros(3)
                delta[i] = step
                h_tag[:, i] = (orientation_residual(pose, tag.retract(delta), camera, measured) -
                               orientation_residual(pose, tag.retract(-delta), camera, measured)) / (2 * step)
            jacobians[0], jacobians[1] = h_pose, h_tag
        return value

    return gtsam.CustomFactor(noise, [pose_key, orientation_key], error)


def odometry_noise(config: dict, increment: gtsam.Pose3, dt: float, covariances=None):
    cfg = config["odometry"]
    distance = float(np.linalg.norm(increment.translation()))
    turned = float(np.linalg.norm(gtsam.Rot3.Logmap(increment.rotation())))
    if cfg["noise"] == "ekf_covariance_difference":
        if covariances is None or any(c is None for c in covariances):
            raise ConfigError("odometry.noise ekf_covariance_difference needs recorded odometry covariance")
        difference = np.diag(np.asarray(covariances[1]).reshape(6, 6) - np.asarray(covariances[0]).reshape(6, 6))
        translation = math.sqrt(max(float(np.mean(difference[:3])), cfg["translation_floor_m"] ** 2))
        rotation = math.sqrt(max(float(np.mean(difference[3:])), cfg["rotation_floor_rad"] ** 2))
    elif cfg["noise"] == "model":
        # Random walks in time and in distance travelled (turned): the variances add, so the total over an interval
        # does not depend on how densely it is cut into keyframes.
        translation = math.sqrt(cfg["translation_rw_m_per_sqrt_s"] ** 2 * max(dt, 0.0) +
                                cfg["translation_rw_m_per_sqrt_m"] ** 2 * distance + cfg["translation_floor_m"] ** 2)
        rotation = math.sqrt(cfg["rotation_rw_rad_per_sqrt_s"] ** 2 * max(dt, 0.0) +
                             cfg["rotation_rw_rad_per_sqrt_rad"] ** 2 * turned + cfg["rotation_floor_rad"] ** 2)
    else:
        raise ConfigError(f"odometry.noise must be model or ekf_covariance_difference, not {cfg['noise']!r}")
    return gtsam.noiseModel.Diagonal.Sigmas(np.array([rotation] * 3 + [translation] * 3))


# ----------------------------------------------------------------------------- the builder
def _keyframe_stamps(odometry: PoseSeries, image_stamps: list, config: dict) -> list:
    """Every image stamp inside the odometry's span, plus a periodic stamp wherever no image stamp lies within
    keyframes.merge_s of it, so that keyframes are never further apart than keyframes.max_interval_s."""
    start, end = odometry.stamps[0], odometry.stamps[-1]
    interval, merge = config["keyframes"]["max_interval_s"], config["keyframes"]["merge_s"]
    images = sorted({s for s in image_stamps if start <= s <= end})
    periodic = [start + i * interval for i in range(int((end - start) / interval) + 1)] + [end]

    def near_image(stamp: float) -> bool:
        i = bisect.bisect_left(images, stamp)
        return any(abs(images[j] - stamp) <= merge for j in (i - 1, i) if 0 <= j < len(images))

    return sorted(set(images) | {s for s in periodic if not near_image(s)})


def _covariance_at(stream: SensorStream, stamp: float):
    stamps = [o.stamp for o in stream.odometry]
    i = min(bisect.bisect_left(stamps, stamp), len(stamps) - 1)
    return stream.odometry[i].covariance


def build(stream: SensorStream, config: dict, layout: dict | None = None) -> Problem:
    if not isinstance(stream, SensorStream):
        raise TypeError("the factor builder reads only a SensorStream from race_tag_graph.sensors")
    if len(stream.odometry) < 2:
        raise ConfigError("at least two odometry records are needed")
    layout = load_layout(config) if layout is None else layout
    det_cfg, ori_cfg = config["detections"], config["orientation"]
    wanted = set(layout) if det_cfg["tags"] == "layout" else set(det_cfg["tags"])
    if wanted - set(layout):
        raise ConfigError(f"tags not in the station layout (their size is needed): {sorted(wanted - set(layout))}")
    base = config["frames"]["base"]

    odometry = PoseSeries([o.stamp for o in stream.odometry],
                          [pose3(o.position, o.orientation) for o in stream.odometry])
    extrinsics = {}
    for array in stream.detections:
        if array.camera not in extrinsics:
            extrinsic = static_chain(list(stream.tf_static), base, array.frame_id)
            if extrinsic is None:
                raise ConfigError(f"no static transform {base} -> {array.frame_id} in /tf_static")
            extrinsics[array.camera] = extrinsic

    images, seen_images = [], set()
    for array in stream.detections:
        if odometry.at(array.stamp) is None or (array.camera, array.stamp) in seen_images:
            continue
        if any(d.tag in wanted for d in array.detections):
            seen_images.add((array.camera, array.stamp))
            images.append(array)
    stamps = _keyframe_stamps(odometry, [a.stamp for a in images], config)
    keyframes = [Keyframe(i, s, odometry.at(s)) for i, s in enumerate(stamps)]
    index_of = {s: i for i, s in enumerate(stamps)}

    if ori_cfg["enabled"] and any(d.orientation is None for a in images for d in a.detections if d.tag in wanted):
        raise ConfigError("orientation.enabled needs tag orientations in the detections; this stream records "
                          "positions only (the collector's telemetry does not record them)")

    prior = config["prior"]
    steps = [Step(keyframe=k.index) for k in keyframes]
    steps[0].factors.append(gtsam.PriorFactorPose3(
        X(0), keyframes[0].ekf, gtsam.noiseModel.Diagonal.Sigmas(np.array(
            [prior["rotation_sigma_rad"]] * 3 + [prior["translation_sigma_m"]] * 3))))
    for previous, current in zip(keyframes, keyframes[1:]):
        increment = previous.ekf.between(current.ekf)
        covariances = None
        if config["odometry"]["noise"] == "ekf_covariance_difference":
            covariances = (_covariance_at(stream, previous.stamp), _covariance_at(stream, current.stamp))
        noise = odometry_noise(config, increment, current.stamp - previous.stamp, covariances)
        steps[current.index].odometry = increment
        steps[current.index].factors.append(gtsam.BetweenFactorPose3(X(previous.index), X(current.index),
                                                                     increment, noise))

    landmarks, accepted_world, first_orientation = {}, {}, {}
    observations, counts = [], {"gated": 0, "out_of_range": 0, "unlisted": 0, "orientation": 0,
                                "orientation_gated": 0}
    for array in images:
        k = index_of[array.stamp]
        pose, extrinsic = keyframes[k].ekf, extrinsics[array.camera]
        camera_rotation = extrinsic.rotation().matrix()
        for detection in array.detections:
            measured = np.asarray(detection.position, dtype=float)
            range_m = float(np.linalg.norm(measured))
            base_point = extrinsic.transformFrom(measured)
            observation = Observation(k, array.stamp, array.camera, detection.tag, measured, base_point, range_m,
                                      accepted=False)
            observations.append(observation)
            if detection.tag not in wanted:
                observation.reason, counts["unlisted"] = "tag not in the configured list", counts["unlisted"] + 1
                continue
            if not 0.0 < range_m <= det_cfg["max_range_m"]:
                observation.reason = "out of range"
                counts["out_of_range"] += 1
                continue
            world = pose.transformFrom(base_point)
            history = accepted_world.setdefault(detection.tag, [])
            if history:
                median = np.median(np.asarray(history), axis=0)
                limit = det_cfg["gate"]["base_m"] + det_cfg["gate"]["per_m"] * range_m
                if float(np.linalg.norm(world - median)) > limit:
                    observation.reason = f"gated: {float(np.linalg.norm(world - median)):.3f} m > {limit:.3f} m"
                    counts["gated"] += 1
                    continue
            history.append(world)
            observation.accepted = True
            observation.range_used = range_used(config, range_m, layout[detection.tag]["size"])
            if detection.tag not in landmarks:
                landmarks[detection.tag] = len(landmarks)
                steps[k].new_landmarks[detection.tag] = base_point
                observation.first = True
            steps[k].factors.append(landmark_factor(X(k), L(landmarks[detection.tag]), base_point,
                                                    detection_noise(config, measured, camera_rotation,
                                                                    layout[detection.tag]["size"])))
            if ori_cfg["enabled"] and range_m <= ori_cfg["max_range_m"]:
                measured_rotation = rot3(detection.orientation)
                base_rotation = extrinsic.rotation().compose(measured_rotation)
                odom_rotation = pose.rotation().compose(base_rotation)
                reference = first_orientation.get(detection.tag)
                if reference is not None and float(np.linalg.norm(
                        gtsam.Rot3.Logmap(reference.between(odom_rotation)))) > ori_cfg["flip_gate_rad"]:
                    counts["orientation_gated"] += 1
                    continue
                if reference is None:
                    first_orientation[detection.tag] = odom_rotation
                    steps[k].new_orientations[detection.tag] = base_rotation
                steps[k].factors.append(orientation_factor(
                    X(k), O(landmarks[detection.tag]), extrinsic.rotation(), measured_rotation,
                    gtsam.noiseModel.Isotropic.Sigma(3, ori_cfg["sigma_rad"])))
                counts["orientation"] += 1
    counts.update(keyframes=len(keyframes), images=len(images), priors=1, odometry=len(keyframes) - 1,
                  landmarks=len(landmarks), detections=sum(o.accepted for o in observations),
                  observations=len(observations))
    return Problem(config=config, keyframes=keyframes, steps=steps, landmarks=landmarks, observations=observations,
                   extrinsics=extrinsics, counts=counts, notes=stream.notes)
