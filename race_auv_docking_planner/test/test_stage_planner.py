"""The planner's decisions, ported from piccard-physical-ai runtime/test_planner.py at 468d65f6 (protocol v1.5):
the stage planner's unit tests, unchanged but for their imports and for the parameters, which come from this package's
config/docking_planner_v1_5.yaml. Left out: the replays of recorded trials (their fixtures carry ground truth, which
belongs to the analysis, not to the vehicle's package) and the tests of the trial collector. No ROS."""
import copy
import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from race_auv_docking_planner import node as node_module  # noqa: E402
from race_auv_docking_planner import parameters as parameters_module  # noqa: E402
from race_auv_docking_planner import planner as planner_module  # noqa: E402

CONFIG = PACKAGE / "config/docking_planner_v1_5.yaml"
PARAMETERS, INITIAL_SETPOINT = parameters_module.load_yaml(CONFIG, planner_module.NODE)
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
AUV_DOCK_IN_BASE = (IDENTITY, [0.395, 0.0, -0.13])  # URDF: nose_tip_link 0.7 m ahead, dock point (-0.305, 0, -0.13)
CG_IN_BASE = ([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], [0.0, 0.0, 0.0])  # Rx(pi) at base_link
# The station dock point in world_ned: Stonefish world (3.59, 0.325, 3.91) with x_C = -y_W, y_C = x_W + 3.8. Its axes
# are base_link's when docked: x along the approach (+y_C, north), z up (-z_C).
STATION_IN_WORLD = (planner_module.level_heading([0.0, 1.0, 0.0]), [-0.325, 7.39, 3.91])
CLEARANCE = PARAMETERS["approach_clearance_m"]  # protocol v1.3: every stage's target this far above the dock point
GOAL_Z = 3.78 - CLEARANCE  # cg_link depth for a level AUV with its dock point on the target (z down)


def estimate(cg_position, yaw, station=STATION_IN_WORLD, stamp=0.0) -> dict:
    """The planner's inputs for a level AUV whose cg_link is at cg_position (world_ned) with heading yaw; the fused
    transform's stamp is stamp (a fixed stamp: one fuser sample, never frozen)."""
    base = (planner_module.level_heading([math.cos(yaw), math.sin(yaw), 0.0]), list(cg_position))
    return {"station_dock_in_base": planner_module.compose(planner_module.inverse(base), station),
            "station_stamp": stamp, "auv_dock_in_base": AUV_DOCK_IN_BASE, "cg_in_base": CG_IN_BASE,
            "base_in_world": base}


def goal_for(planner: planner_module.StagePlanner, inputs: dict) -> dict:
    """The planner's goal after it has seen these inputs (the first observation sets the filter)."""
    planner.observe(0.0, inputs)
    return planner.goal(inputs)


def at_goal(planner: planner_module.StagePlanner) -> dict:
    """An estimate with the AUV exactly where the current stage wants it (the true station, unfiltered)."""
    probe = planner_module.StagePlanner(planner.p)
    probe.stage = planner.stage
    goal = goal_for(probe, estimate([0.0, 1.0, 3.5], 1.2))
    return estimate([goal["x_m"], goal["y_m"], goal["z_m"]], goal["yaw_rad"])


def parameters(**changes) -> dict:
    return {**copy.deepcopy(PARAMETERS), **changes}




class ParameterTests(unittest.TestCase):
    def test_the_v1_5_file_is_valid(self):
        self.assertEqual(parameters_module.validate_parameters(copy.deepcopy(PARAMETERS)), PARAMETERS)
        self.assertEqual(INITIAL_SETPOINT, {"x_m": -0.325, "y_m": 0.0, "z_m": 3.78, "roll_rad": 0.0, "pitch_rad": 0.0,
                                            "yaw_rad": 1.571})

    def test_rejections(self):
        cases = {"increasing": ("standoffs_m", [0.3, 1.5, 0.0]), "not_at_dock": ("standoffs_m", [3.0, 0.3]),
                 "repeat": ("standoffs_m", [3.0, 3.0, 0.0]), "empty": ("standoffs_m", []),
                 "far": ("standoffs_m", [9.0, 0.0]), "nan_band": ("band_m", float("nan")), "bool": ("tick_hz", True),
                 "fast": ("tick_hz", 11), "no_cap": ("speed_cap_mps", 0.0),
                 "clearance_negative": ("approach_clearance_m", -0.01), "clearance_high": ("approach_clearance_m", 0.2),
                 "deadband_past_vertical_band": ("setpoint_deadband_m", 0.02),
                 "deadband_rad_past_band_rad": ("setpoint_deadband_rad", 0.06)}
        for name, (key, value) in cases.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                parameters_module.validate_parameters(parameters(**{key: value}))
        for name, change in {"extra": lambda p: p.update(gain=1.0), "missing": lambda p: p.pop("settle_s")}.items():
            raw = parameters()
            change(raw)
            with self.subTest(name=name), self.assertRaises(ValueError):
                parameters_module.validate_parameters(raw)
        for name, change in {"nan": {"z_m": float("nan")}, "yaw": {"yaw_rad": 4.0}, "extra": {"label": "dive"}}.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                parameters_module.validate_initial_setpoint({**INITIAL_SETPOINT, **change})
        with self.assertRaises(ValueError):
            parameters_module.validate_initial_setpoint({k: v for k, v in INITIAL_SETPOINT.items() if k != "z_m"})

    def test_ros_parameters_by_name(self):
        """What the node reads: every planner parameter and initial_setpoint.*, the vectors as arrays."""
        import array
        flat = {**{k: array.array("d", v) if isinstance(v, list) else v for k, v in PARAMETERS.items()},
                **{f"initial_setpoint.{k}": v for k, v in INITIAL_SETPOINT.items()}}
        self.assertEqual(parameters_module.from_ros_parameters(flat), (PARAMETERS, INITIAL_SETPOINT))
        with self.assertRaises(ValueError):
            parameters_module.from_ros_parameters({k: v for k, v in flat.items() if k != "initial_setpoint.yaw_rad"})
        with self.assertRaises(ValueError):
            parameters_module.from_ros_parameters({**flat, "ground_truth_topic": "/race_auv/stonefish/odometry"})


class StagePlannerTests(unittest.TestCase):
    def test_goals_are_the_m2_pose_derivation_for_every_stand_off(self):
        """With the fused station dock point where it truly is, the goal for stand-off s is the pose the M2 builder
        derives for s: cg_link 0.395 m behind and 0.13 m above the AUV dock point (x_C -0.325, y_C 6.995 - s, z_C
        3.78, heading north), wherever the AUV happens to be; from protocol v1.3 approach_clearance_m higher."""
        for stage, standoff in enumerate(PARAMETERS["standoffs_m"]):
            for position, yaw in (([0.0, 1.0, 3.5], 1.2), ([-0.4, 6.0, 3.9], 1.7)):
                planner = planner_module.StagePlanner(PARAMETERS)
                planner.stage = stage
                goal = goal_for(planner, estimate(position, yaw))
                with self.subTest(standoff=standoff, position=position):
                    self.assertAlmostEqual(goal["x_m"], -0.325, places=9)
                    self.assertAlmostEqual(goal["y_m"], 7.39 - standoff - 0.395, places=9)
                    self.assertAlmostEqual(goal["z_m"], GOAL_Z, places=9)
                    self.assertAlmostEqual(goal["yaw_rad"], math.pi / 2, places=9)
                    self.assertEqual((goal["roll_rad"], goal["pitch_rad"]), (0.0, 0.0))

    def test_the_goal_follows_the_fused_estimate_not_the_scenario(self):
        """A fused station 0.1 m to the left and 0.2 rad turned moves the goal with it."""
        planner = planner_module.StagePlanner(PARAMETERS)
        planner.stage = len(PARAMETERS["standoffs_m"]) - 1  # stand-off 0: the AUV dock point on the fused one
        turned = (planner_module.level_heading([-math.sin(0.2), math.cos(0.2), 0.0]), [-0.225, 7.39, 3.91])
        goal = goal_for(planner, estimate([0.0, 1.0, 3.5], 1.2, station=turned))
        self.assertAlmostEqual(goal["yaw_rad"], math.pi / 2 + 0.2, places=9)
        self.assertAlmostEqual(goal["x_m"], -0.225 + 0.395 * math.sin(0.2), places=9)
        self.assertAlmostEqual(goal["y_m"], 7.39 - 0.395 * math.cos(0.2), places=9)

    def test_errors_are_in_the_station_dock_frame(self):
        planner = planner_module.StagePlanner(PARAMETERS)
        planner.stage = 1  # 1.5 m
        goal = goal_for(planner, estimate([0.0, 1.0, 3.5], 1.2))
        self.assertEqual([round(v, 9) for v in planner.error(estimate([goal["x_m"], goal["y_m"], goal["z_m"]],
                                                                          goal["yaw_rad"]))], [0.0, 0.0, 0.0, 0.0])
        # 0.1 m farther back, 0.05 m to the left (+x_C), 0.02 m deeper
        shifted = estimate([goal["x_m"] + 0.05, goal["y_m"] - 0.1, goal["z_m"] + 0.02], goal["yaw_rad"])
        self.assertEqual([round(v, 9) for v in planner.error(shifted)[:3]], [-0.1, 0.05, -0.02])
        # yaw_C +0.1 (world z down: a turn to the right) puts the station's axis 0.1 rad to the AUV's left (base_link)
        turned = estimate([goal["x_m"], goal["y_m"], goal["z_m"]], goal["yaw_rad"] + 0.1)
        self.assertAlmostEqual(planner.error(turned)[3], 0.1, places=9)

    def test_a_rotation_error_about_the_tag_pivot_moves_neither_the_dock_point_nor_the_goal(self):
        """The fuser's collinear-tag solve pins the tag centres and gets the rotation wrong about them (M3-0: yaw sd
        35-44 deg at 1.5 m). The re-anchored dock point, placed with the planner's own heading, does not move."""
        pivot = PARAMETERS["tag_pivot_m"]
        rotation, position = STATION_IN_WORLD
        anchor = [position[i] + v for i, v in enumerate(planner_module.apply(rotation, pivot))]
        for yaw, pitch in ((math.radians(30), 0.0), (math.radians(-40), math.radians(10)), (0.0, math.radians(15))):
            error = planner_module.matmul(
                [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]],
                [[math.cos(pitch), 0.0, math.sin(pitch)], [0.0, 1.0, 0.0], [-math.sin(pitch), 0.0, math.cos(pitch)]])
            wrong = planner_module.matmul(rotation, error)  # the solve's rotation, about the pivot
            fused = (wrong, [anchor[i] - v for i, v in enumerate(planner_module.apply(wrong, pivot))])
            planner = planner_module.StagePlanner(PARAMETERS)
            planner.observe(0.0, estimate([0.0, 1.0, 3.5], 1.2))  # the heading from a good sample (the first stage)
            planner.hold_heading()
            planner.observe(0.25, estimate([0.0, 1.0, 3.5], 1.2, station=fused))
            goal = planner.goal(estimate([0.0, 1.0, 3.5], 1.2, station=fused))
            with self.subTest(yaw=yaw, pitch=pitch):
                self.assertEqual([round(v, 9) for v in planner.station[0]], [round(v, 9) for v in position])
                self.assertAlmostEqual(planner.station[1], math.pi / 2, places=9)
                self.assertAlmostEqual(goal["y_m"], 7.39 - 3.0 - 0.395, places=9)
                self.assertAlmostEqual(goal["z_m"], GOAL_Z, places=9)
                self.assertGreater(math.dist(fused[1], position), 0.02)  # the fused dock point itself did move

    def test_the_heading_is_the_circular_median_of_the_window(self):
        """While the first stage runs: the median over heading_window_s rejects a minority of wrong solves and
        forgets samples older than the window."""
        planner = planner_module.StagePlanner(parameters(estimate_filter_s=0))
        rotation, position = STATION_IN_WORLD
        turned = lambda angle: (planner_module.matmul(
            [[math.cos(angle), -math.sin(angle), 0.0], [math.sin(angle), math.cos(angle), 0.0], [0.0, 0.0, 1.0]],
            rotation), position)
        for k in range(70):  # 10 s at 7 Hz; every third sample is a collinear-solve outlier of +-40 deg
            angle = (math.radians(40) if k % 6 == 2 else math.radians(-40)) if k % 3 == 2 else 0.0
            planner.observe(k / 7, estimate([0.0, 1.0, 3.5], 1.2, station=turned(angle)))
        self.assertAlmostEqual(planner.station[1], math.pi / 2, places=9)
        for k in range(70, 70 + 7 * 31):  # then 31 s at +0.1 rad: the old samples leave the 30 s window
            planner.observe(k / 7, estimate([0.0, 1.0, 3.5], 1.2, station=turned(0.1)))
        self.assertAlmostEqual(planner.station[1], math.pi / 2 + 0.1, places=9)

    def test_the_heading_is_held_from_the_second_stage(self):
        planner = planner_module.StagePlanner(parameters(settle_s=0, estimate_filter_s=0))
        first = planner.step(0.0, at_goal(planner))  # inside at once: settle_s 0 ends the first stage
        self.assertEqual((planner.stage, first["heading_held"]), (1, planner.station[1]))
        rotation, position = STATION_IN_WORLD
        pivot = PARAMETERS["tag_pivot_m"]
        anchor = [position[i] + v for i, v in enumerate(planner_module.apply(rotation, pivot))]
        wrong = planner_module.matmul([[math.cos(0.6), -math.sin(0.6), 0.0], [math.sin(0.6), math.cos(0.6), 0.0],
                                       [0.0, 0.0, 1.0]], rotation)
        fused = (wrong, [anchor[i] - v for i, v in enumerate(planner_module.apply(wrong, pivot))])
        records = [planner.step(0.25 * k, estimate([0.0, 1.0, 3.5], 1.2, station=fused)) for k in range(1, 20)]
        self.assertTrue(all(r["heading_held"] == first["heading_held"] and r["station"][3] == first["heading_held"]
                            for r in records))
        self.assertAlmostEqual(records[-1]["station"][1], position[1], places=9)

    def test_the_station_estimate_is_low_passed(self):
        planner = planner_module.StagePlanner(PARAMETERS)  # estimate_filter_s 2
        planner.step(0.0, estimate([0.0, 1.0, 3.5], 1.2))
        moved = (STATION_IN_WORLD[0], [-0.225, 7.39, 3.91])  # the fused dock point jumps 0.1 m to the left
        record = planner.step(0.25, estimate([0.0, 1.0, 3.5], 1.2, station=moved))
        step = 1 - math.exp(-0.25 / PARAMETERS["estimate_filter_s"])
        self.assertAlmostEqual(record["station"][0], -0.325 + 0.1 * step, places=9)
        self.assertAlmostEqual(record["goal"]["x_m"], -0.325 + 0.1 * step, places=9)
        unfiltered = planner_module.StagePlanner({**PARAMETERS, "estimate_filter_s": 0})
        unfiltered.step(0.0, estimate([0.0, 1.0, 3.5], 1.2))
        self.assertAlmostEqual(unfiltered.step(0.25, estimate([0.0, 1.0, 3.5], 1.2, station=moved))["station"][0],
                               -0.225, places=9)

    def test_set_points_step_only_at_a_stage_start_or_after_settle_s_at_the_set_point(self):
        """mvp_control zeroes an axis's integral whenever its set point changes: the set point is re-sent unchanged
        however the estimate moves, until the vehicle has been at it for settle_s with a fresh error still outside the
        band; then only the axes that error is outside on step, if their goal moved past the deadband."""
        planner = planner_module.StagePlanner(parameters(estimate_filter_s=0, settle_s=2))
        first = planner.step(0.0, estimate([0.0, 1.0, 3.5], 1.2))["command"]
        rotation, position = STATION_IN_WORLD
        moved = (rotation, [position[0] + 0.03, position[1] - 0.1, position[2] + 0.01])  # x +3 cm, y -10 cm, z +1 cm
        self.assertEqual(planner.step(0.25, estimate([0.0, 1.0, 3.5], 1.2, station=moved))["command"], first)
        there = estimate([first["x_m"], first["y_m"], first["z_m"]], first["yaw_rad"], station=moved)  # arrived
        self.assertTrue(all(planner.step(0.5 + k / 4, there)["command"] == first for k in range(8)))  # 0.5..2.25 s
        stepped = planner.step(2.5, there)  # settle_s at the set point, 10 cm short: along (and lateral) step
        self.assertEqual({k: stepped["command"][k] == first[k] for k in first},
                         {"x_m": False, "y_m": False, "z_m": True, "roll_rad": True, "pitch_rad": True, "yaw_rad": True})
        self.assertAlmostEqual(stepped["command"]["x_m"], first["x_m"] + 0.03, places=9)
        self.assertEqual(stepped["setpoint_updates"], {"x_m": 1, "y_m": 1, "z_m": 0, "yaw_rad": 0})
        held = stepped["command"]
        further = (rotation, [position[0] + 0.03, position[1] - 0.1, position[2] + 0.1])  # 10 cm deeper than the held depth
        again = estimate([held["x_m"], held["y_m"], held["z_m"]], held["yaw_rad"], station=further)
        self.assertTrue(all(planner.step(2.75 + k / 4, again)["command"] == held for k in range(8)))  # 2.75..4.5 s
        deeper = planner.step(4.75, again)
        self.assertAlmostEqual(deeper["command"]["z_m"], held["z_m"] + 0.1, places=9)
        self.assertEqual(deeper["setpoint_updates"], {"x_m": 1, "y_m": 1, "z_m": 1, "yaw_rad": 0})
        self.assertEqual(deeper["stage"], 0)

    def test_depth_steps_when_the_vertical_error_is_outside_the_band_and_never_in_the_final_stage(self):
        dive = {"label": "dive", "x_m": -0.325, "y_m": 0.0, "z_m": 3.70, "roll_rad": 0.0, "pitch_rad": 0.0,
                "yaw_rad": math.pi / 2, "dwell_s": 40}  # 6 cm shallower than the station-derived goal (GOAL_Z)
        planner = planner_module.StagePlanner({**PARAMETERS, "estimate_filter_s": 0, "standoffs_m": [0.3, 0.0],
                                               "settle_s": 0}, initial=dive)
        far = estimate([-0.325, 0.0, 3.70], math.pi / 2)
        commands = [planner.step(0.2 * k, far)["command"] for k in range(400)]  # the walk to y 6.695 lands at tick 334
        self.assertEqual({c["z_m"] for c in commands}, {3.70})  # depth kept, however long the vehicle is away
        start = commands[-1]
        self.assertEqual(round(start["y_m"], 9), round(7.39 - 0.3 - 0.395, 9))  # y re-targeted, walked
        arrived = planner.step(80.0, estimate([start["x_m"], start["y_m"], 3.70], math.pi / 2))["command"]
        self.assertAlmostEqual(arrived["z_m"], GOAL_Z, places=9)  # at the set point (settle_s 0), 6 cm shallow
        self.assertEqual(planner.stage, 0)  # 6 cm deep of the target: not inside the band yet
        planner.step(80.2, at_goal(planner))  # inside: settle_s 0 starts the final stage
        self.assertEqual(planner.stage, 1)
        rotation, position = STATION_IN_WORLD
        deeper = (rotation, [position[0], position[1], position[2] + 0.05])  # the estimate moves 5 cm in the final stage
        ramped = [planner.step(80.2 + 0.2 * k, estimate([-0.325, 6.7, 3.78], math.pi / 2, station=deeper))["command"]
                  for k in range(1, 28)]
        self.assertEqual({c["z_m"] for c in ramped}, {arrived["z_m"]})
        self.assertEqual(planner.updates["z_m"], 1)
        self.assertAlmostEqual(ramped[-1]["y_m"], 7.39 - 0.395, places=9)  # the final goal fixed at the stage start

    def test_the_handover_keeps_the_fallback_set_point(self):
        """Started from the dive's set point, the planner leaves every axis whose goal is within the deadband, and
        depth, which a stage start never changes."""
        dive = {"label": "dive", "x_m": -0.325, "y_m": 7.39 - 3.0 - 0.395 + 0.004, "z_m": GOAL_Z - 0.015,
                "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": math.pi / 2 + 0.01, "dwell_s": 40}
        planner = planner_module.StagePlanner(PARAMETERS, initial=dive)
        self.assertEqual(planner.step(0.0, None)["command"], {k: dive[k] for k in planner_module.COMMAND_FIELDS})
        command = planner.step(0.25, estimate([0.0, 1.0, 3.5], 1.2))["command"]
        self.assertEqual(command, {k: dive[k] for k in planner_module.COMMAND_FIELDS})

    def test_nothing_is_commanded_before_an_estimate(self):
        planner = planner_module.StagePlanner(PARAMETERS)
        record = planner.step(0.0, None)
        self.assertEqual((record["state"], record["command"], record["event"]), ("stale", None, "stage_start"))

    def test_a_stage_ends_after_settle_s_inside_the_band(self):
        planner = planner_module.StagePlanner(PARAMETERS)
        steps = [planner.step(k / 4, at_goal(planner)) for k in range(0, 41)]  # 0..10 s (exact binary times)
        self.assertEqual([s["stage"] for s in steps[:40]], [0] * 40)
        self.assertEqual((steps[40]["stage"], steps[40]["event"], steps[40]["standoff_m"]), (1, "stage_start", 1.5))
        self.assertEqual([t["standoff_m"] for t in planner.transitions], [3.0, 1.5])

    def test_the_lateral_band_widens_with_the_stand_off(self):
        """band_m along, band_m + s * band_rad lateral: at 3 m, 0.05 + 3 * 0.05 = 0.20 m; at the dock, band_m. Vertical:
        vertical_band_m before the final stage (v1.2), band_m in it. The lateral target error is s times the station
        heading's estimate error."""
        for stage, limits in ((0, [0.05, 0.2, 0.01, 0.05]), (2, [0.05, 0.065, 0.01, 0.05]), (3, [0.05, 0.05, 0.05, 0.05])):
            planner = planner_module.StagePlanner(PARAMETERS)
            planner.stage = stage
            with self.subTest(standoff=PARAMETERS["standoffs_m"][stage]):
                self.assertEqual([round(v, 9) for v in planner.band()], limits)
        goal = goal_for(planner_module.StagePlanner(PARAMETERS), estimate([0.0, 1.0, 3.5], 1.2))  # stage 0, 3 m
        for dx, dy, inside in ((0.19, 0.0, True), (0.21, 0.0, False), (0.0, -0.06, False)):  # +x_C: left; -y_C: short
            probe = planner_module.StagePlanner(parameters(estimate_filter_s=0))
            record = probe.step(0.0, estimate([goal["x_m"] + dx, goal["y_m"] + dy, goal["z_m"]], goal["yaw_rad"]))
            with self.subTest(dx=dx, dy=dy):
                self.assertEqual(record["inside_s"] is not None, inside)

    def test_leaving_the_band_restarts_the_settle_clock(self):
        planner = planner_module.StagePlanner(PARAMETERS)
        for k in range(0, 21):  # 0..5 s inside
            planner.step(k / 4, at_goal(planner))
        goal = planner.goal(at_goal(planner))  # the filter has converged on the (fixed) station
        outside = estimate([goal["x_m"], goal["y_m"] - 0.06, goal["z_m"]], goal["yaw_rad"])  # 6 cm short
        planner.step(5.25, outside)
        stages = [planner.step(5.5 + k / 4, at_goal(planner))["stage"] for k in range(0, 41)]  # inside from 5.5 s
        self.assertEqual(stages.index(1), 40)  # at 15.5 s, settle_s after re-entering

    def test_a_stale_tick_is_missing_data_not_a_restart(self):
        """Stale ticks hold the set point; settling counts from the first fresh inside tick and ends on a fresh one."""
        planner = planner_module.StagePlanner(PARAMETERS)
        records = []
        for k in range(0, 49):  # 0..12 s; every third tick stale, and stale from 9.75 to 10.5 s
            t = k / 4
            stale = k % 3 == 2 or 9.75 <= t <= 10.5
            records.append(planner.step(t, None if stale else at_goal(planner)))
        held = [r for r in records if r["state"] == "stale" and r["stage"] == 0 and r["command"] is not None]
        self.assertTrue(held and all(r["command"] == records[0]["command"] for r in held))
        first = next(i for i, r in enumerate(records) if r["stage"] == 1)
        self.assertEqual(first / 4, 10.75)  # the first fresh tick at or after settle_s
        self.assertEqual(records[first]["event"], "stage_start")

    def test_a_frozen_estimate_is_stale_like_an_old_one(self):
        """D bit-identical under advancing stamps for longer than max_frozen_estimate_s: the tick is stale ("frozen"),
        holds the set point and leaves the settle clock alone; the same sample re-read under its own stamp is not."""
        planner = planner_module.StagePlanner(parameters(max_frozen_estimate_s=2.0))
        rotation, position = STATION_IN_WORLD
        goal = goal_for(planner_module.StagePlanner(PARAMETERS), estimate([0.0, 1.0, 3.5], 1.2))  # stage 0's

        def live(t, frozen=False):  # the AUV on stage 0's goal; the fuser's sample at t has 0.1-0.2 mm noise
            jitter = 0.0 if frozen else 1e-4 * (1 + int(t * 4) % 2)
            return estimate([goal["x_m"], goal["y_m"], goal["z_m"]], goal["yaw_rad"], stamp=t,
                            station=(rotation, [position[0] + jitter, *position[1:]]))

        records = [planner.step(k / 4, live(k / 4, frozen=1.0 <= k / 4 < 6.0)) for k in range(0, 49)]  # 0..12 s
        states = {k / 4: (r["state"], r["stale_reason"]) for k, r in enumerate(records)}
        self.assertEqual(states[3.0], ("tracking", None))  # frozen since 1 s: 2 s is not more than the limit
        self.assertEqual(states[3.25], ("stale", "frozen"))
        self.assertEqual(states[5.75], ("stale", "frozen"))
        self.assertEqual(states[6.0], ("tracking", None))  # the pose moved: a new run
        stale = [r for r in records if r["state"] == "stale"]
        self.assertTrue(all(r["command"] == records[0]["command"] and r["goal"] is None for r in stale))
        first = next(i for i, r in enumerate(records) if r["stage"] == 1)
        self.assertEqual(first / 4, 10.0)  # settling counted from 0 s through the frozen ticks
        repeated = planner_module.StagePlanner(parameters(max_frozen_estimate_s=2.0))
        same = [repeated.step(k / 4, {**at_goal(repeated), "station_stamp": 0.0})["state"] for k in range(0, 17)]
        self.assertEqual(set(same), {"tracking"})  # one sample, re-read every tick: old, not frozen

    def test_the_final_stage_rate_limits_the_set_point(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.3, 0.0], settle_s=0))
        first = planner.step(0.0, at_goal(planner))  # inside at once: the final stage starts
        self.assertEqual(planner.stage, 1)
        start = first["command"]
        commands = [planner.step(0.2 * (k + 1), at_goal(planner))["command"] for k in range(20)]
        step = PARAMETERS["speed_cap_mps"] / PARAMETERS["tick_hz"]
        previous = start
        for command in commands:
            moved = math.dist([command[k] for k in ("x_m", "y_m", "z_m")], [previous[k] for k in ("x_m", "y_m", "z_m")])
            self.assertLessEqual(moved, step + 1e-9)
            previous = command
        self.assertAlmostEqual(commands[13]["y_m"] - start["y_m"], 14 * step, places=9)  # still ramping at 0.02 m/tick
        # landed after 0.3 m / 0.1 m/s = 3 s; the vehicle is on the goal from the stage start, so the along axis arrives
        # at once and its re-issue, equal to the held value, is nudged by ARRIVAL_NUDGE_M (protocol v1.5)
        self.assertAlmostEqual(commands[-1]["y_m"], 7.39 - 0.395 + planner_module.ARRIVAL_NUDGE_M, places=9)

    def test_without_a_held_set_point_the_first_goal_is_commanded_at_once(self):
        planner = planner_module.StagePlanner(PARAMETERS)
        far = estimate([-0.325, 0.0, 3.78], math.pi / 2)  # at the dive, 7 m out
        command = planner.step(0.0, far)["command"]
        self.assertAlmostEqual(command["y_m"], 7.39 - 3.0 - 0.395, places=9)

    def test_complete_after_final_stage_s_then_hold(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.0], final_stage_s=4))
        records = [planner.step(k / 4, at_goal(planner)) for k in range(0, 17)]
        self.assertEqual(records[15]["state"], "tracking")
        self.assertEqual((records[16]["state"], records[16]["event"]), ("complete", "complete"))
        held = planner.step(5.0, estimate([0.0, 1.0, 3.5], 1.2))
        self.assertEqual((held["state"], held["event"], held["command"]), ("complete", None, records[16]["command"]))


DIVE = {"label": "dive", "x_m": -0.125, "y_m": 0.0, "z_m": 3.70, "roll_rad": 0.0, "pitch_rad": 0.0,
        "yaw_rad": math.pi / 2 + 0.1, "dwell_s": 40}  # 20 cm off the approach line, 8 cm shallow, 0.1 rad off
HORIZONTAL = ("x_m", "y_m")


def horizontal_move(a: dict, b: dict) -> float:
    return math.hypot(*(b[k] - a[k] for k in HORIZONTAL))


class WalkTests(unittest.TestCase):
    """v1.1 (piccard-experiments #88): the set point walks to every stage's goal at speed_cap_mps."""
    STEP = PARAMETERS["speed_cap_mps"] / PARAMETERS["tick_hz"]

    def walk(self, planner, inputs, ticks, t0=0.0):
        return [planner.step(t0 + k / PARAMETERS["tick_hz"], inputs) for k in range(ticks)]

    def test_a_stage_start_walks_x_y_at_the_speed_cap_with_yaw_set_and_depth_kept(self):
        planner = planner_module.StagePlanner(PARAMETERS, initial=DIVE)
        far = estimate([-0.125, 0.0, 3.70], math.pi / 2 + 0.1)  # at the dive, 7 m out
        records = self.walk(planner, far, 260)
        commands = [r["command"] for r in records]
        held = planner.held
        distance = math.hypot(held["x_m"] - DIVE["x_m"], held["y_m"] - DIVE["y_m"])  # 4.1 m
        landing = math.ceil(distance / self.STEP) - 1  # the tick index that lands
        self.assertAlmostEqual(horizontal_move(DIVE, commands[0]), self.STEP, places=9)
        moves = [horizontal_move(a, b) for a, b in zip(commands, commands[1:])]
        self.assertTrue(all(abs(m - self.STEP) < 1e-9 for m in moves[:landing - 1]))  # 0.1 m/s at 5 Hz
        self.assertLessEqual(moves[landing - 1], self.STEP + 1e-9)
        self.assertEqual(commands[landing], held)  # landed exactly
        self.assertNotEqual(commands[landing - 1], held)
        self.assertEqual(commands[-1], held)
        self.assertEqual({c["z_m"] for c in commands}, {DIVE["z_m"]})  # depth: the held depth throughout
        self.assertEqual({c["yaw_rad"] for c in commands}, {held["yaw_rad"]})  # yaw: the goal's from the first tick
        self.assertAlmostEqual(held["yaw_rad"], math.pi / 2, places=9)  # the station's heading
        self.assertEqual(records[-1]["setpoint_updates"], {"x_m": 1, "y_m": 1, "z_m": 0, "yaw_rad": 1})
        self.assertEqual([r["stage"] for r in records], [0] * len(records))

    def test_the_final_stage_walk_is_the_same_walk(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.0]), initial=DIVE)
        commands = [r["command"] for r in self.walk(planner, estimate([-0.125, 0.0, 3.70], math.pi / 2), 400)]
        moves = [horizontal_move(a, b) for a, b in zip([DIVE] + commands, commands)]
        self.assertLessEqual(max(moves), self.STEP + 1e-9)
        self.assertEqual(commands[-1], planner.held)
        self.assertEqual({c["z_m"] for c in commands}, {DIVE["z_m"]})

    def test_a_stale_tick_pauses_the_walk(self):
        planner = planner_module.StagePlanner(PARAMETERS, initial=DIVE)
        far = estimate([-0.125, 0.0, 3.70], math.pi / 2)
        before = self.walk(planner, far, 10)[-1]["command"]
        paused = planner.step(2.0, None)
        self.assertEqual((paused["state"], paused["command"]), ("stale", before))
        after = planner.step(2.2, far)["command"]
        self.assertAlmostEqual(horizontal_move(before, after), self.STEP, places=9)

    def test_refinement_waits_for_the_landing_and_is_one_step(self):
        planner = planner_module.StagePlanner(parameters(estimate_filter_s=0, settle_s=2), initial=DIVE)
        far = estimate([-0.125, 0.0, 3.70], math.pi / 2)
        planner.step(0.0, far)
        held = dict(planner.held)
        rotation, position = STATION_IN_WORLD
        moved = (rotation, [position[0] + 0.03, position[1] - 0.1, position[2]])  # the goal moves 3 cm and -10 cm
        there = estimate([held["x_m"], held["y_m"], held["z_m"]], held["yaw_rad"], station=moved)
        # the vehicle sits at the held goal with the error outside the band while the set point still walks: no step
        walking = [planner.step(0.2 * k, there) for k in range(1, 60)]  # 11.8 s, settle_s 2
        self.assertTrue(planner.walking)
        self.assertEqual({tuple(sorted(r["setpoint_updates"].items())) for r in walking},
                         {(("x_m", 1), ("y_m", 1), ("yaw_rad", 1), ("z_m", 0))})
        self.assertEqual(planner.held, held)
        k = 60
        while planner.walking:
            planner.step(0.2 * k, there)
            k += 1
        landed = dict(planner.command)
        self.assertEqual(landed, held)
        records = [planner.step(0.2 * (k + j), there) for j in range(12)]  # settle_s at the landed set point
        stepped = next(r for r in records if r["setpoint_updates"]["x_m"] == 2)
        self.assertAlmostEqual(stepped["command"]["x_m"], held["x_m"] + 0.03, places=9)  # one step, not walked
        self.assertAlmostEqual(stepped["command"]["y_m"], held["y_m"] - 0.1, places=9)
        self.assertAlmostEqual(stepped["command"]["z_m"], GOAL_Z, places=9)  # the 6 cm vertical error: depth too
        self.assertEqual(stepped["setpoint_updates"], {"x_m": 2, "y_m": 2, "z_m": 1, "yaw_rad": 1})
        self.assertFalse(planner.walking)

    def test_a_stage_that_ends_mid_walk_walks_on_from_the_command(self):
        planner = planner_module.StagePlanner(parameters(settle_s=1), initial=DIVE)
        records, k = [], 0
        while planner.stage == 0:  # the vehicle at the 3 m target at once: the stage ends after settle_s
            records.append(planner.step(0.2 * k, at_goal(planner)))
            k += 1
        self.assertEqual((len(records), planner.walking), (6, True))  # 1 s in, 0.2 m of the 4.1 m walk
        far = estimate([-0.125, 0.0, 3.70], math.pi / 2)
        records += [planner.step(0.2 * (k + j), far) for j in range(400)]
        commands = [r["command"] for r in records]
        moves = [horizontal_move(a, b) for a, b in zip([DIVE] + commands, commands)]
        self.assertLessEqual(max(moves), self.STEP + 1e-9)  # no jump at the stage change
        self.assertEqual(commands[-1], planner.held)
        self.assertAlmostEqual(planner.held["y_m"], 7.39 - 1.5 - 0.395, places=9)  # the 1.5 m goal


class WaitingTests(unittest.TestCase):
    """v1.1: the waiting ticks (the fallback pose's dwell) observe, so the handover goal has the window's heading."""

    def flip(self, k: int) -> float:
        """A single-tag solve's two headings: 3 of 5 samples 3 deg off, 2 of 5 13 deg off the other way."""
        return math.pi / 2 + math.radians(3.0 if k % 5 < 3 else -10.0)

    def station_at(self, heading: float):
        rotation, position = STATION_IN_WORLD
        return planner_module.level_heading([math.cos(heading), math.sin(heading), 0.0]), position

    def test_waiting_ticks_fill_the_window_and_the_filter_and_nothing_else(self):
        planner = planner_module.StagePlanner(PARAMETERS, initial=DIVE)
        command, held = dict(planner.command), dict(planner.held)
        far = estimate([-0.125, 0.0, 3.70], math.pi / 2)
        for k in range(50):  # 10 s at 5 Hz
            planner.wait(k / 5, far)
        self.assertEqual(len(planner.headings), 50)
        self.assertIsNotNone(planner.station)
        self.assertEqual((planner.command, planner.held), (command, held))
        self.assertEqual((planner.stage, planner.stage_started, planner.inside_since, planner.arrived_since),
                         (0, None, None, None))
        self.assertEqual((planner.updates, planner.transitions, planner.walking),
                         ({"x_m": 0, "y_m": 0, "z_m": 0, "yaw_rad": 0}, [], False))

    def test_the_window_keeps_heading_window_s(self):
        planner = planner_module.StagePlanner(PARAMETERS, initial=DIVE)
        far = estimate([-0.125, 0.0, 3.70], math.pi / 2)
        for k in range(240):  # 60 s at 4 Hz (exact binary times)
            planner.wait(k / 4, far)
        self.assertEqual(len(planner.headings), PARAMETERS["heading_window_s"] * 4)

    def test_stale_or_frozen_waiting_ticks_are_not_observed(self):
        planner = planner_module.StagePlanner(PARAMETERS, initial=DIVE)
        planner.wait(0.0, None)
        self.assertEqual((planner.headings, planner.station), ([], None))
        frozen = estimate([-0.125, 0.0, 3.70], math.pi / 2)  # one transform under advancing stamps
        for k in range(20):  # 4 s; max_frozen_estimate_s 2
            planner.wait(k / 5, {**frozen, "station_stamp": k / 5})
        self.assertEqual(len(planner.headings), 11)  # 0..2.0 s

    def test_the_handover_goal_uses_the_window_median_not_the_first_sample(self):
        for waited in (False, True):
            planner = planner_module.StagePlanner(parameters(estimate_filter_s=0), initial=DIVE)
            for k in range(150):  # 30 s of the flip
                inputs = estimate([-0.125, 0.0, 3.70], math.pi / 2, station=self.station_at(self.flip(k)), stamp=k / 5)
                if waited:
                    planner.wait(k / 5, inputs)
            flipped = self.station_at(math.pi / 2 - math.radians(10))  # the handover's own sample is a flipped one
            handover = estimate([-0.125, 0.0, 3.70], math.pi / 2, station=flipped, stamp=30.0)
            record = planner.step(30.0, handover)
            error = math.degrees(record["command"]["yaw_rad"] - math.pi / 2)
            if waited:
                self.assertAlmostEqual(error, 3.0, places=6)  # the median: the majority solve
            else:
                self.assertAlmostEqual(error, -10.0, places=6)  # v1.0: the one sample

    def test_the_node_observes_while_it_waits_on_the_planner_clock(self):
        """The ROS node (not importable without ROS): its waiting ticks call wait(), and wait() and step() share one
        clock, so the window spans the handover."""
        source = (PACKAGE / "race_auv_docking_planner/node.py").read_text()
        tick = source[source.index("        def tick(self):"):source.index("    return Planner, rclpy")]
        waiting = tick[tick.index("if self.started is None:"):tick.index("self.ticks += 1")]
        self.assertIn("self.planner.wait(now - self.created, self.estimate())", waiting)
        self.assertNotIn("self.setpoints.publish", waiting)
        self.assertIn("self.planner.step(now - self.created, self.estimate())", tick)
        self.assertIn('"heading_samples": len(self.planner.headings)', waiting)

    def test_waiting_starts_no_clock(self):
        """The vehicle already inside the band through the dwell: the stage still needs settle_s after the
        handover."""
        planner = planner_module.StagePlanner(parameters(settle_s=2), initial=None)
        for k in range(100):
            planner.wait(k / 5, at_goal(planner))
        records = [planner.step(20.0 + k / 5, at_goal(planner)) for k in range(11)]
        self.assertEqual([r["stage"] for r in records], [0] * 10 + [1])  # 0..1.8 s in stage 0, then settle_s 2


class FinalStageFollowTests(unittest.TestCase):
    """v1.2 (piccard-experiments #88): in the final stage x/y follow the live goal past final_deadband_m, walked. From
    v1.5 the along axis (y here) does so only in hold: these vehicles start on the goal, so it arrives on the first
    tick (one re-issue, nudged by ARRIVAL_NUDGE_M as the goal equals the held value) and holds."""
    STEP = PARAMETERS["speed_cap_mps"] / PARAMETERS["tick_hz"]

    def drifting(self, seconds: float, drift_mps: float, until_s: float | None = None):
        """A final stage from its start, the EKF frame drifting laterally (x_C, across the approach) at drift_mps
        until until_s: the fused relative pose is exact, so the station estimate in world_ned moves with the drift."""
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.0]), initial=None)
        start = at_goal(planner)  # the vehicle at the dock goal
        rotation, position = STATION_IN_WORLD
        records = []
        for k in range(int(seconds * PARAMETERS["tick_hz"]) + 1):
            t = k / PARAMETERS["tick_hz"]
            d = drift_mps * min(t, until_s if until_s is not None else t)
            station = (rotation, [position[0] + d, position[1], position[2]])
            cg = start["base_in_world"][1]
            inputs = estimate([cg[0] + d, cg[1], cg[2]], math.pi / 2, station=station)
            records.append((t, d, planner.step(t, inputs)))
        return planner, records

    def test_a_lateral_drift_is_followed_within_the_deadband_and_walked(self):
        planner, records = self.drifting(60.0, 0.001)
        deadband = PARAMETERS["final_deadband_m"]
        commands = [r["command"] for _, _, r in records]
        first = records[0][2]["command"]
        for (t, d, record), previous in zip(records[1:], commands):
            goal, command = record["goal"], record["command"]
            self.assertLessEqual(math.hypot(goal["x_m"] - command["x_m"], goal["y_m"] - command["y_m"]),
                                 deadband + self.STEP + 1e-9)  # the filtered goal
            self.assertLessEqual(abs(command["x_m"] - (first["x_m"] + d)), deadband + self.STEP + 1e-9)  # the drift
            self.assertLessEqual(math.hypot(command["x_m"] - previous["x_m"], command["y_m"] - previous["y_m"]),
                                 self.STEP + 1e-9)  # walked
            self.assertEqual((command["z_m"], command["yaw_rad"]), (first["z_m"], first["yaw_rad"]))  # depth, yaw held
        self.assertGreaterEqual(planner.updates["x_m"], 10)  # 6 cm of drift in steps of about 5 mm
        self.assertLessEqual(planner.updates["x_m"], 13)
        self.assertEqual((planner.updates["z_m"], planner.updates["yaw_rad"]), (0, 0))
        self.assertEqual({r["stage"] for _, _, r in records}, {0})

    def test_a_lateral_only_drift_never_touches_y(self):
        """v1.4: each world axis re-targets on its own; across the approach (x at this heading) the along axis y and
        its integral are left alone, but for v1.5's arrival re-issue on the first tick."""
        planner, records = self.drifting(60.0, 0.001)
        arrivals = planner.along_log["arrivals"]
        # the first tick sets the held value (no set point before the planner), the second arrives
        self.assertEqual([(a["t"], a["change_m"]) for a in arrivals], [(0.2, planner_module.ARRIVAL_NUDGE_M)])
        self.assertEqual(planner.updates["y_m"], 1)
        self.assertEqual({r["command"]["y_m"] for _, _, r in records[1:]}, {records[1][2]["command"]["y_m"]})

    def test_the_deadband_is_per_axis_not_euclidean(self):
        """4 mm on each axis (5.7 mm together): v1.2-v1.3 re-targeted both; v1.4 neither."""
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.0], estimate_filter_s=0))
        start = at_goal(planner)
        planner.step(0.0, start)
        rotation, position = STATION_IN_WORLD
        moved = (rotation, [position[0] + 0.004, position[1] + 0.004, position[2]])
        cg = start["base_in_world"][1]
        for k in range(1, 50):
            planner.step(k / 5, estimate([cg[0] + 0.004, cg[1] + 0.004, cg[2]], math.pi / 2, station=moved))
        self.assertEqual(planner.updates, {"x_m": 0, "y_m": 1, "z_m": 0, "yaw_rad": 0})  # y: the arrival re-issue
        self.assertEqual([a["t"] for a in planner.along_log["arrivals"]], [0.2])  # after the first tick's held value

    def test_no_re_target_below_the_deadband(self):
        planner, records = self.drifting(30.0, 0.001, until_s=4.0)  # 4 mm, then still
        commands = {tuple(sorted(r["command"].items())) for _, _, r in records[1:]}  # after the arrival re-issue
        self.assertEqual(len(commands), 1)
        self.assertEqual(planner.updates, {"x_m": 0, "y_m": 1, "z_m": 0, "yaw_rad": 0})

    def test_a_stale_tick_pauses_the_following(self):
        planner, records = self.drifting(20.0, 0.001)
        held = dict(planner.command)
        paused = planner.step(20.2, None)
        self.assertEqual((paused["state"], paused["command"]), ("stale", held))

    def test_final_stage_s_still_counts_from_the_stage_start(self):
        planner, records = self.drifting(PARAMETERS["final_stage_s"] + 1.0, 0.001)
        states = [(t, r["state"]) for t, _, r in records]
        self.assertEqual(next(t for t, state in states if state == "complete"), PARAMETERS["final_stage_s"])


class FinalAlongPhaseTests(unittest.TestCase):
    """v1.5 (piccard-experiments #88): in the final stage the along axis (world y at this heading) approaches from the
    stage start, re-targeted only on a stale command (|goal - held| past final_along_deadband_m, final_along_interval_s
    after its last change); arrives once the fused along error is within final_arrival_m of the goal on the approach's
    side, re-issuing the along set point at the goal; then holds, following the goal past final_deadband_m; and
    approaches again, from its side, after final_reapproach_s past final_along_deadband_m."""

    def run_final(self, seconds: float, vehicle, drift=lambda t: 0.0, stale=lambda t: False, **changes):
        """A final stage from its start: the station (and so the EKF-frame goal) drifting along y by drift(t); the
        vehicle's cg_link along y by vehicle(t, planner), in metres from the start goal; stale(t) drops the tick."""
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.0], estimate_filter_s=0, **changes))
        start = at_goal(planner)["base_in_world"][1]
        rotation, position = STATION_IN_WORLD
        records = []
        for k in range(int(seconds * PARAMETERS["tick_hz"]) + 1):
            t = k / PARAMETERS["tick_hz"]
            station = (rotation, [position[0], position[1] + drift(t), position[2]])
            inputs = estimate([start[0], start[1] + vehicle(t, planner), start[2]], math.pi / 2, station=station)
            records.append((t, planner.step(t, None if stale(t) else inputs)))
        return planner, records

    @staticmethod
    def changes(records, key="y_m"):
        return [t for (t, r), (_, before) in zip(records[1:], records) if r["held"][key] != before["held"][key]]

    def test_the_approach_re_targets_only_a_stale_command(self):
        """The vehicle stuck 10 cm short while the goal drifts 1 mm/s along: |goal - held| passes 2 cm at 20 s, but y
        first changes at final_along_interval_s (60 s), then at 120 s; it never arrives; x never changes."""
        interval = PARAMETERS["final_along_interval_s"]
        planner, records = self.run_final(150.0, lambda t, p: -0.10, drift=lambda t: 0.001 * t)
        self.assertEqual(self.changes(records), [interval, 2 * interval])
        self.assertEqual(planner.updates["x_m"], 0)
        self.assertEqual((planner.along_phase, planner.along_log["arrivals"]), ("approach", []))

    def test_a_drift_inside_the_along_deadband_never_re_targets_the_approach(self):
        planner, records = self.run_final(150.0, lambda t, p: -0.10, drift=lambda t: 0.001 * min(t, 15.0))
        self.assertEqual(self.changes(records), [])

    def test_arrival_re_issues_the_along_set_point_once_at_the_goal(self):
        """The goal drifts 1 cm (inside the approach's 2 cm) while the vehicle closes at 2 mm/s from 10 cm short; the
        first tick within final_arrival_m of the goal re-issues y at the goal, 1 cm from the held value, and holds."""
        planner, records = self.run_final(80.0, lambda t, p: min(-0.10 + 0.002 * t, 0.008),
                                          drift=lambda t: 0.001 * min(t, 10.0))
        arrivals = planner.along_log["arrivals"]
        self.assertEqual(len(arrivals), 1)
        arrival = arrivals[0]
        self.assertEqual(self.changes(records), [arrival["t"]])
        self.assertEqual(arrival["side"], -1)
        self.assertGreaterEqual(arrival["error_m"], -PARAMETERS["final_arrival_m"])
        self.assertLess(arrival["error_m"] - 0.002 / PARAMETERS["tick_hz"], -PARAMETERS["final_arrival_m"])  # the first
        self.assertAlmostEqual(arrival["change_m"], 0.01, places=6)
        record = dict(records)[arrival["t"]]
        self.assertAlmostEqual(record["held"]["y_m"], record["goal"]["y_m"], places=9)
        self.assertEqual(record["final_along"]["phase"], "hold")
        self.assertEqual(planner.along_log["reapproaches"], [])

    def test_an_arrival_on_the_held_value_is_still_a_set_point_change(self):
        """mvp_control zeroes an integral only on a change: with the goal equal to the held value, the re-issue is
        nudged by ARRIVAL_NUDGE_M."""
        planner, records = self.run_final(60.0, lambda t, p: min(-0.10 + 0.002 * t, 0.0))
        (arrival,) = planner.along_log["arrivals"]
        self.assertEqual(arrival["change_m"], planner_module.ARRIVAL_NUDGE_M)
        self.assertEqual(self.changes(records), [arrival["t"]])

    def test_in_hold_the_along_axis_follows_the_goal(self):
        """Arrived at once, then the goal drifts 1 mm/s along with the vehicle on its command: y follows in steps of
        about final_deadband_m, and the error never leaves final_along_deadband_m, so no new approach."""
        planner, records = self.run_final(
            60.0, lambda t, p: (p.command["y_m"] - at_goal(p)["base_in_world"][1][1]) if p.command else 0.0,
            drift=lambda t: 0.001 * t)
        self.assertEqual(len(planner.along_log["arrivals"]), 1)
        follows = self.changes(records)[1:]
        self.assertGreaterEqual(len(follows), 10)  # 6 cm in steps of about 5 mm
        self.assertLessEqual(len(follows), 12)
        self.assertEqual((planner.along_phase, planner.along_log["reapproaches"]), ("hold", []))

    def test_a_new_approach_from_past_the_goal(self):
        """Arrived at once; pushed 3 cm past at 10 s and left there, with stale ticks from 12 to 15 s (they neither
        restart nor end the clock): a new approach, from past (side +1), at 10 + final_reapproach_s; back to 3 mm past
        at 40 s: a second arrival, from past."""
        wait = PARAMETERS["final_reapproach_s"]
        planner, records = self.run_final(
            50.0, lambda t, p: 0.0 if t < 10.0 else (0.03 if t < 40.0 else 0.003), stale=lambda t: 12.0 <= t < 15.0)
        (reapproach,) = planner.along_log["reapproaches"]
        self.assertEqual((reapproach["t"], reapproach["side"]), (10.0 + wait, 1))
        self.assertAlmostEqual(reapproach["error_m"], 0.03, places=9)
        second = planner.along_log["arrivals"][1]
        self.assertEqual((second["t"], second["side"]), (40.0, 1))
        self.assertEqual(planner.along_phase, "hold")
        self.assertEqual(len(planner.along_log["arrivals"]), 2)

    def test_coming_back_inside_restarts_the_reapproach_clock(self):
        planner, records = self.run_final(40.0, lambda t, p: 0.03 if 5.0 <= t < 12.0 or t >= 13.0 else 0.0)
        (reapproach,) = planner.along_log["reapproaches"]
        self.assertEqual(reapproach["t"], 13.0 + PARAMETERS["final_reapproach_s"])

    def test_the_arrival_must_lie_inside_the_along_deadband(self):
        planner = parameters(final_arrival_m=PARAMETERS["final_along_deadband_m"])
        with self.assertRaisesRegex(ValueError, "final_arrival_m must be less than final_along_deadband_m"):
            parameters_module.validate_parameters(planner)


PIVOT = PARAMETERS["tag_pivot_m"]


def station_about_pivot(heading: float):
    """The station dock frame as a fused solve at this heading would place it: rotated about the tag pivot, whose
    image the tag centres pin (a single-tag flip turns the solve about the tag)."""
    rotation0, position0 = STATION_IN_WORLD
    pivot_world = [position0[i] + v for i, v in enumerate(planner_module.apply(rotation0, PIVOT))]
    rotation = planner_module.level_heading([math.cos(heading), math.sin(heading), 0.0])
    return rotation, [pivot_world[i] - v for i, v in enumerate(planner_module.apply(rotation, PIVOT))]


def fused_at(planner, fused_heading: float, heading: float = math.pi / 2) -> dict:
    """The vehicle exactly at the current stage's goal for the station at heading, seen through a fused solve at
    fused_heading (turned about the tag pivot, so the re-anchored dock point does not move)."""
    probe = planner_module.StagePlanner(planner.p)
    probe.stage = planner.stage
    goal = goal_for(probe, estimate([0.0, 1.0, 3.5], 1.2, station=station_about_pivot(heading)))
    return estimate([goal["x_m"], goal["y_m"], goal["z_m"]], goal["yaw_rad"],
                    station=station_about_pivot(fused_heading))


class FinalHeadingReholdTests(unittest.TestCase):
    """v1.2: at the final stage's first fresh tick the held heading is replaced once by the circular median of the
    fresh fused headings of the stage before it within heading_window_s; fewer than REHOLD_MIN_SAMPLES keep it."""
    H0 = math.pi / 2

    def test_once_at_the_final_stage_start_from_the_stage_before_it(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[1.5, 0.3, 0.0], settle_s=1, estimate_filter_s=0))
        h1, h2 = self.H0 + math.radians(2.0), self.H0 + math.radians(4.0)
        for k in range(50):  # 10 s of the dive at H0: inside the window, but not the stage before the final one
            planner.wait(k / 5, fused_at(planner, self.H0))
        records, t = [], 10.0
        for fused, stage in ((self.H0, 0), (h1, 1)):
            for _ in range(100):
                if planner.stage != stage:
                    break
                records.append(planner.step(t, fused_at(planner, fused)))
                t += 0.2
        self.assertEqual(planner.stage, 2)
        self.assertAlmostEqual(planner.heading_held, self.H0, places=9)  # stage 1 kept the first stage's heading
        self.assertIsNone(planner.rehold)
        yaw_before = planner.updates["yaw_rad"]
        final = [planner.step(t + k / 5, fused_at(planner, h2)) for k in range(50)]
        rehold = final[0]["heading_rehold"]
        self.assertTrue(rehold["applied"])
        self.assertAlmostEqual(rehold["heading"], h1, places=9)  # the 0.3 m stage's samples only
        self.assertAlmostEqual(rehold["previous"], self.H0, places=9)
        self.assertTrue(all(r["heading_rehold"] == rehold and r["heading_held"] == h1 for r in final))  # h2 never
        self.assertEqual(planner.updates["yaw_rad"], yaw_before + 1)  # one yaw step, at the stage start
        self.assertEqual({round(r["command"]["yaw_rad"], 9) for r in final}, {round(h1, 9)})
        self.assertTrue(all(r["heading_rehold"] is None for r in records))

    def test_too_few_samples_keep_the_held_heading(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.3, 0.0], settle_s=0, estimate_filter_s=0))
        first = planner.step(0.0, fused_at(planner, self.H0))  # inside: stage 0 ends at once
        self.assertEqual((first["stage"], planner.heading_held), (1, self.H0))
        record = planner.step(0.2, fused_at(planner, self.H0 + math.radians(3.0)))
        rehold = record["heading_rehold"]
        self.assertEqual((rehold["applied"], rehold["samples"]), (False, 2))
        self.assertAlmostEqual(planner.heading_held, self.H0, places=9)
        self.assertLess(rehold["samples"], planner_module.REHOLD_MIN_SAMPLES)

    def test_a_single_tag_flip_on_a_fifth_of_the_samples_keeps_the_median_within_a_degree(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.3, 0.0], settle_s=8))
        truth = self.H0 + math.radians(0.2)
        for k in range(200):
            if planner.stage != 0:
                break
            fused = truth - math.radians(10.0) if k % 5 == 0 else truth  # a fifth of the solves flipped
            planner.step(k / 5, fused_at(planner, fused, truth))
        self.assertEqual(planner.stage, 1)
        record = planner.step(k / 5, fused_at(planner, truth, truth))
        rehold = record["heading_rehold"]
        self.assertTrue(rehold["applied"])
        self.assertGreaterEqual(rehold["samples"], 40)
        self.assertLess(abs(math.degrees(rehold["heading"] - truth)), 1.0)


class VerticalBandTests(unittest.TestCase):
    """vertical_band_m before the final stage (v1.2). From v1.3 a refinement step moves a flagged axis only past its
    deadband, and only once the vehicle has been at its set point for settle_s with z within vertical_band_m."""

    def run_stage(self, bias_m: float, tail_m=lambda t: 0.0, seconds: float = 12.0, settle_s: float = 1,
                  yaw_bias: float = 0.0, yaw_tail=lambda t: 0.0):
        """Stage 0 held at its goal but bias_m shallow (yaw_bias turned); the vehicle at its set point plus a
        depth tail_m(t) (+: deeper) and a yaw swing yaw_tail(t)."""
        params = parameters(estimate_filter_s=0, settle_s=settle_s)
        probe = planner_module.StagePlanner(params)
        goal = goal_for(probe, estimate([0.0, 1.0, 3.5], 1.2))
        initial = {**{k: goal[k] for k in planner_module.COMMAND_FIELDS}, "z_m": goal["z_m"] - bias_m,
                   "yaw_rad": goal["yaw_rad"] + yaw_bias, "dwell_s": 40, "label": "dive"}
        planner = planner_module.StagePlanner(params, initial=initial)
        records = []
        for k in range(round(seconds * 5)):
            held, t = planner.held, k / 5
            records.append(planner.step(t, estimate([held["x_m"], held["y_m"], held["z_m"] + tail_m(t)],
                                                    held["yaw_rad"] + yaw_tail(t))))
            if planner.stage == 1:
                break
        return planner, records, goal

    def test_a_2_2_cm_vertical_bias_takes_one_z_step_then_the_stage_advances(self):
        planner, records, goal = self.run_stage(0.022)
        self.assertEqual(planner.stage, 1)
        self.assertEqual(planner.updates["z_m"], 1)
        steps = [r for r in records if r["setpoint_updates"]["z_m"] == 1]
        self.assertAlmostEqual(steps[0]["command"]["z_m"], goal["z_m"], places=9)  # one discrete step, to the goal
        advance = records[-1]
        self.assertEqual(advance["event"], "stage_start")
        before = records[-2]["error"]
        self.assertLessEqual(abs(before[2]), PARAMETERS["vertical_band_m"])
        first = records[0]["error"]
        self.assertAlmostEqual(first[2], 0.022, places=6)  # the AUV 2.2 cm shallow: +z up in L

    def test_a_1_5_cm_bias_steps_once(self):
        """1.5 cm is outside vertical_band_m (1 cm) and past setpoint_deadband_m (0.5 cm): one step, to the goal."""
        self.assertTrue(PARAMETERS["setpoint_deadband_m"] < PARAMETERS["vertical_band_m"] < 0.015)
        planner, records, goal = self.run_stage(0.015)
        self.assertEqual((planner.stage, planner.updates["z_m"]), (1, 1))
        self.assertAlmostEqual(planner.held["z_m"], goal["z_m"], places=9)

    def test_a_sub_millimetre_goal_move_flagged_by_the_controller_tail_does_not_step(self):
        """The M3-0 v1.2 cycle: the held depth 0.8 mm shallow of the goal, the vehicle 0.95 cm shallow of its set point
        (inside the arrival gate), so the vertical error is 1.03 cm and flagged. v1.2 stepped z by 0.8 mm; v1.3 does
        not, and the stage advances once the tail decays."""
        tail = lambda t: -0.0095 if t < 30 else 0.0
        planner, records, goal = self.run_stage(0.0008, tail_m=tail, seconds=60, settle_s=10)
        flagged = [r for r in records if r["error"] and abs(r["error"][2]) > PARAMETERS["vertical_band_m"]]
        self.assertGreater(len(flagged), 100)  # flagged for the whole tail, the gate open from 10 s
        self.assertEqual(planner.updates["z_m"], 0)
        self.assertEqual((planner.stage, records[-1]["event"]), (1, "stage_start"))
        self.assertGreaterEqual((len(records) - 1) / 5, 30 + 10)  # after the tail, then settle_s inside the band

    def test_a_0_62_deg_heading_move_flagged_by_the_yaw_swing_does_not_step(self):
        """The M3-0 v1.2 yaw step at m 795.75: the heading estimate 0.62 deg from the held yaw, the vehicle swinging
        +4 deg: the heading error (3.4 deg) is outside band_rad and flagged, the goal is inside setpoint_deadband_rad
        (1.15 deg). v1.2 stepped yaw; v1.3 does not."""
        yaw_bias = math.radians(-0.62)
        swing = lambda t: math.radians(4.0) - yaw_bias if t < 30 else -yaw_bias
        self.assertLess(abs(yaw_bias), PARAMETERS["setpoint_deadband_rad"])
        planner, records, goal = self.run_stage(0.0, seconds=60, settle_s=10, yaw_bias=yaw_bias,
                                                yaw_tail=lambda t: swing(t) + yaw_bias)
        flagged = [r for r in records if r["error"] and abs(r["error"][3]) > PARAMETERS["band_rad"]]
        self.assertGreater(len(flagged), 100)
        self.assertEqual(planner.updates["yaw_rad"], 0)
        self.assertEqual(planner.stage, 1)

    def test_the_arrival_gate_stays_closed_during_a_1_5_cm_tail(self):
        """A goal 1.2 cm deeper than the held depth (past the deadband, outside the band) and the vehicle 1.5 cm deep of
        its set point for 60 s: the gate (z within vertical_band_m of the set point) keeps the refinement check
        closed, so nothing steps while the tail lasts; once it decays, exactly one z step after settle_s, then the
        stage advances. v1.2's gate (band_m, 5 cm) would have stepped at 10 s."""
        tail = lambda t: 0.015 if t < 60 else 0.0
        planner, records, goal = self.run_stage(-0.012, tail_m=tail, seconds=120, settle_s=10)
        steps = [(k / 5, r) for k, r in enumerate(records) if r["setpoint_updates"]["z_m"] == 1]
        self.assertEqual(planner.updates["z_m"], 1)
        self.assertGreaterEqual(steps[0][0], 60 + 10)  # after the tail and settle_s at the set point
        self.assertAlmostEqual(planner.held["z_m"], goal["z_m"], places=9)
        self.assertEqual(planner.stage, 1)

    def test_inside_the_vertical_band_nothing_steps(self):
        planner, records, goal = self.run_stage(0.008)
        self.assertEqual((planner.stage, planner.updates["z_m"]), (1, 0))

    def test_the_final_stage_keeps_band_m_vertical_and_never_steps_depth(self):
        planner = planner_module.StagePlanner(parameters(standoffs_m=[0.0]))
        self.assertEqual(planner.band()[2], PARAMETERS["band_m"])


class ClearanceTests(unittest.TestCase):
    """v1.3: every stage's vertical target places the AUV dock point approach_clearance_m above the station dock
    point, and the vertical error is around it."""

    def test_the_clearance_shifts_every_stage_vertical_target_by_exactly_approach_clearance_m(self):
        clearance = 0.02
        for stage, standoff in enumerate(PARAMETERS["standoffs_m"]):
            vehicle = estimate([-0.3, 3.0, 3.8], 1.5)
            level, raised = (planner_module.StagePlanner(parameters(approach_clearance_m=c)) for c in (0.0, clearance))
            for planner in (level, raised):
                planner.stage = stage
            flat, up = goal_for(level, vehicle), goal_for(raised, vehicle)
            with self.subTest(standoff=standoff):
                for key in ("x_m", "y_m", "roll_rad", "pitch_rad", "yaw_rad"):
                    self.assertAlmostEqual(up[key], flat[key], places=12)
                self.assertAlmostEqual(flat["z_m"] - up["z_m"], clearance, places=12)  # z down: shallower
                self.assertAlmostEqual(level.error(vehicle)[2] - raised.error(vehicle)[2], clearance, places=12)
                self.assertEqual(level.error(vehicle)[:2], raised.error(vehicle)[:2])

    def test_the_vehicle_on_the_clearance_target_is_inside_the_band_and_the_record_states_it(self):
        planner = planner_module.StagePlanner(parameters(settle_s=0, estimate_filter_s=0))
        record = planner.step(0.0, at_goal(planner))
        self.assertEqual([round(v, 9) for v in record["error"]], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(record["goal"]["approach_clearance_m"], CLEARANCE)
        self.assertNotIn("approach_clearance_m", record["command"])
        level = estimate([record["goal"]["x_m"], record["goal"]["y_m"], record["goal"]["z_m"] + CLEARANCE],
                         record["goal"]["yaw_rad"])  # the dock points level: CLEARANCE below the target
        self.assertAlmostEqual(planner.error(level)[2], -CLEARANCE, places=9)


class SetpointStepTests(unittest.TestCase):
    def test_the_state_record_carries_the_held_set_point(self):
        planner = planner_module.StagePlanner(PARAMETERS)
        record = planner.step(0.0, estimate([0.0, 1.0, 3.5], 1.2))
        self.assertEqual(record["held"], planner.held)


class NodeInputTests(unittest.TestCase):
    """The node's gather(): the four edges, the static ones once, and nothing while the odometry is old."""

    def recording(self, missing=()):
        calls = []

        def lookup(parent, child, max_age=None):
            calls.append((parent, child, max_age))
            return None if (parent, child) in missing else (([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.0, 0, 0]), 7.0)
        return calls, lookup

    def test_the_four_edges_and_the_station_age(self):
        calls, lookup = self.recording()
        static = {}
        inputs = node_module.gather(lookup, static, 0.1, PARAMETERS["max_estimate_age_s"])
        self.assertEqual({(p, c) for p, c, _ in calls}, set(planner_module.PLANNER_TF_LOOKUPS))
        self.assertEqual([a for p, c, a in calls if c == planner_module.STATION_DOCK],
                         [PARAMETERS["max_estimate_age_s"]])
        self.assertEqual(set(inputs), {"auv_dock_in_base", "cg_in_base", "station_dock_in_base", "station_stamp",
                                       "base_in_world"})
        calls.clear()
        node_module.gather(lookup, static, 0.1, PARAMETERS["max_estimate_age_s"])
        self.assertEqual({(p, c) for p, c, _ in calls}, {(planner_module.BASE_LINK, planner_module.STATION_DOCK),
                                                        (planner_module.WORLD_LINK, planner_module.BASE_LINK)})

    def test_old_or_missing_inputs_give_none(self):
        calls, lookup = self.recording()
        self.assertIsNone(node_module.gather(lookup, {}, None, 1.0))
        self.assertIsNone(node_module.gather(lookup, {}, 1.5, 1.0))
        self.assertEqual(calls, [])
        _, lookup = self.recording(missing={(planner_module.BASE_LINK, planner_module.STATION_DOCK)})
        self.assertIsNone(node_module.gather(lookup, {}, 0.1, 1.0))


if __name__ == "__main__":
    unittest.main()
