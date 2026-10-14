#!/usr/bin/env python3
"""M3 docking planner (piccard-physical-ai #109, protocol piccard-experiments #88): a staged approach on the
AprilTag-fused station dock point, commanding bhv_direct_control pose set points.

Inputs, and nothing else: the TF tree on /tf and /tf_static, looked up for four edges only (PLANNER_TF_LOOKUPS: the
fuser's race_auv/base_link -> race_station/dock_point, race_auv/world_ned -> race_auv/base_link through the EKF's odom,
and the static base_link -> auv_dock_point and base_link -> cg_link), and /race_auv/odometry/filtered, whose age gates
every tick. The collector's consumption audit checks the node's subscriptions and these lookups (planner_inputs); the
TF tree itself is audited free of ground truth. No station pose from the scenario or from ground truth is read here.

Output: mvp_msgs/ControlProcess set points (cg_link in race_auv/world_ned) on the helm's desired_setpoints, and one
JSON state record per tick on /piccard/planner/state for the collector. The planner waits (1 Hz state, no set points)
until the collector calls /piccard_planner/start (std_srvs/Trigger) after the fallback pose's dwell.

Each tick (tick_hz):
1. The estimate: the fused station dock frame D in base_link, and the EKF odometry, each at most max_estimate_age_s
   old; otherwise the tick is stale (stale_reason "age") and the last set point is held. It is stale ("frozen") too
   when D's stamp has advanced but its transform has stayed bit-identical for longer than max_frozen_estimate_s, on
   the fuser's stamps: a simulator whose render froze kept the fuser publishing one pose under fresh stamps. The
   fuser repeats its pose between camera frames, so live data has such runs too (up to 7.7 s in the dev loop, whose
   camera renders at about 0.75 Hz). A stale tick is missing data: it neither ends a stage nor restarts the settle
   or at-set-point clock (in the CPU-rendered dev loop the fuser runs at 1.3-2.8 Hz with gaps of several seconds; on
   the L40S at 7 Hz).
2. The station estimate: a level station at a heading, and its dock point, both in world_ned (through the EKF's TF).
   The fuser cannot give the heading at short range. On the approach it sees only the forward camera's three tags,
   all on the station's vertical centreline; with three it solves on their centres, which leaves yaw about that line
   unconstrained (M3-0 on the L40S: fused yaw sd 35-44 deg with them; 2.7-4.2 deg from the per-tag fallback with one
   or two tags at 3 m, piccard-experiments #88). So:
   - heading: the circular median of D's heading over the last heading_window_s, from the fallback pose's dwell
     (the waiting ticks observe too) until the first stage ends, then held for the rest of the trial. The median
     rejects the minority of three-tag solves at 3 m, and at the handover (7.45 m on the M3-0 rerun, tag 146 alone)
     the single-tag solve's flip between two headings about 13 deg apart: a first sample alone set that rerun's stage-0
     goal 11 deg off, the window's median would have been 2.5 deg off. At the final stage's first fresh tick
     (protocol v1.2) the held heading is replaced once by the circular median of the fresh fused headings of the
     stage before it within heading_window_s: at 0.3 m tags 541 and 558 both solve, without the three-tag degeneracy
     of 1.5 m (fused yaw sd 0.05-0.24 deg under 1 m on the v1.1 run, whose held heading was 1.88 deg off, 1.5 cm
     across at the dock over the 0.457 m pivot). With fewer than REHOLD_MIN_SAMPLES it keeps the held heading. Either
     way it is held from then on: single-tag flips near the dock (558 leaves at about 18 cm) never reach the command.
     Each state record carries heading_rehold {t, samples, applied, previous, heading};
   - dock point: re-anchored on the tag pivot. The fused pose places the pivot tag_pivot_m (the forward camera's tag
     centroid in D, derived from the station URDF and checked at trial start) well even when its rotation is wrong;
     the dock point is that point minus the pivot rotated by the level station at the heading. It is low-passed with
     time constant estimate_filter_s (0: none). Replayed on M3-0's first 180 s at 1.5 m (test_planner.py): the
     dock point's error against truth fell from along +9 +- 19, lateral +- 7.5 and vertical -2.6 +- 4.5 cm as fused to
     +2.7 +- 1.7, +0.45 +- 0.42 and -0.04 +- 0.25 cm, with the held heading 0.6 deg off.
   Only the heading of D's orientation enters the level frame L (x along the heading, z up): the fused pitch is noisy
   at range (sd 5-20 deg at 4-8 m in the arm64 dev loop).
3. The stage target for the AUV dock point: stand-off s behind the station estimate along L's x axis and
   approach_clearance_m above it on L's z axis (protocol v1.3; docked, D's axes are base_link's). The clearance keeps
   the AUV's legs above the station frame's rails on the approach. On the M3-0 v1.2 run, with the dock points level,
   the legs met the frame at the rail entrance, 0.81 m before the dock point: swept along the approach they
   interfere 4.2-6.6 mm from 0.68 to 0.13 m of stand-off. At 2.2 cm above, v1.1's depth, they clear by 15.7-22.7 mm.
   The controller cannot descend onto the dock without a depth step (see 6), so the clearance holds to the end.
   vertical_band_m is the stage-advance test around the clearance, not a clearance guarantee: on the M3-0 v1.3 run
   the vehicle held +2.19 cm (2.16-2.25) in the final stage and the legs' tightest point, at 29 cm of stand-off,
   cleared by 8.9 mm, which at the band's lower edge (+1 cm) would be about -1 mm.
4. The stage error: the AUV dock point relative to the target in L (along, lateral, vertical) and the AUV's heading
   relative to L's (positive: the station's axis lies to the AUV's left). The vertical error is around the clearance
   target. The band: band_m along, vertical_band_m
   vertical in every stage but the final one (protocol v1.2; band_m in the final stage), band_rad in heading, and
   band_m + s * band_rad lateral at stand-off s. v1.1 held band_m (5 cm) vertical too, and the M3-0 v1.1 run
   reached the dock 2.2 cm shallow: inside that band, so never corrected. The lateral target error is s times the
   error of the estimated station heading (filtered sd 1.1-3.2 deg at 3 m in the dev loop: 6-17 cm), and a heading
   error of band_rad, already accepted, moves the dock point's track by about s * band_rad; at the dock the band is
   band_m. A stage other than the last ends, on a fresh tick, once every fresh error for settle_s has been within the
   band; an error outside it restarts the settle clock.
5. The goal: cg_link in world_ned with the AUV dock point on the target, level, heading along L's x axis.
6. The held goal: the set point moves in discrete steps, as M2's pose missions did. mvp_control 6cfea2d zeroes an
   axis's integral whenever its set point changes (mvp_control_ros.cpp f_amend_set_point). The depth integral carries
   the vehicle's buoyancy (about 14 at 3.8 m; a reset leaves the AUV about 0.9 m shallow until it rebuilds) and the
   along-axis integral a few more; a goal that followed the filtered estimate every tick changed each axis every 1-2
   s at 3 m in the dev loop and held the AUV 0.9 m shallow. So x, y and yaw are re-targeted when a stage starts.
   Then, in a stage other than the last, once the vehicle has been at its set point for settle_s on fresh ticks
   (protocol v1.3: its cg_link within band_m of it on x and y and within vertical_band_m on z) and a fresh error is
   still outside the band, the axes that error is outside on are stepped to the goal: x and y for the along or
   lateral error, z for the vertical, yaw for the heading. An axis changes only if its goal differs from the held
   value by more than setpoint_deadband_m (setpoint_deadband_rad for yaw), at a stage start and in a refinement step
   alike (protocol v1.3); mission.py holds each deadband to at most its band (setpoint_deadband_m to vertical_band_m
   and band_m, setpoint_deadband_rad to band_rad), so with the vehicle on its set point a vertical or heading error
   outside its band always has a goal past the deadband (an along or lateral one too while setpoint_deadband_m is at
   most band_m / sqrt(2), as the error splits over world x and y). The at-set-point clock restarts after a step, so a
   set point steps at most once per settle_s. v1.2 stepped a flagged axis whatever the difference, and its arrival
   gate held z to band_m (5 cm): on its M3-0 run a z step's integral reset raised the AUV 50 cm and it took about 150
   s to settle inside 1 cm; the gate reopened at about +100 s, measured the controller's own tail (1.1-1.9 cm either
   way) as station error and stepped z by 0.1-1.1 mm, 13 times in 1450 s. The planner starts from the fallback set
   point the controller already holds, so the handover changes no axis that need not change. In the final stage
   (protocol v1.2) x and y follow the live goal, with no settle wait; yaw keeps the stage start's (one step to the
   re-held heading, past setpoint_deadband_rad, see 2 above) and depth is kept. v1.1 fixed the final goal at the
   stage start, and on its M3-0 run the EKF frame drifted 13.2 cm across the approach in the 120 s final stage: the
   planner saw its lateral error grow to 12 cm and did not act. From protocol v1.4 each world axis re-targets on its
   own, when its goal differs from its held value by more than final_deadband_m (v1.2-v1.3: the Euclidean x/y
   difference, and both axes together); the lateral axis keeps that rule. mvp_control zeroes only the changed axis's
   integral: on the M3-0 v1.3 run the final stage re-targeted x and y together 47 times, a median of 2.2 s apart, so the
   along integral (gain about 0.2 per m s against P 10: about 50 s) never built, and the vehicle crept on P alone at 0.3
   mm/s and ended 4.7 cm short of a set point within 0.2 cm of the true dock point. v1.4 gated the along axis (the world
   axis closer to L's x; y at the RACE station's heading) on the fused along error past final_along_deadband_m and
   final_along_interval_s since its last change. On both M3-0 v1.4 runs the vehicle moved at most 0.5 mm/s below about
   0.6 of x/y effort, a 6 cm error on P alone: what closes the last centimetres is the integral wound on the stage
   start's approach (+1.06 on the upstream run), and it also carried the vehicle 2 cm past the dock. The error gate then
   held a command the EKF frame had drifted 6.8 cm off the true dock point, until the vehicle had been pulled out of the
   2 cm window. From protocol v1.5 the along axis has phases. Approach, from the stage start: re-targeted only when its
   command is stale (the goal past final_along_deadband_m from the held value, and final_along_interval_s since the last
   along change). Arrival, on the first fresh tick whose fused along error is within final_arrival_m of the goal on the
   approach's side: the along set point is re-issued at the goal (ARRIVAL_NUDGE_M off the held value if they are equal),
   a set-point change that zeroes the approach's windup where the vehicle is, not past the dock; on the v1.4 run a late
   re-target zeroed +1.04 at 1.3 mm/s and the vehicle coasted 0.3 cm and stopped. Hold: the along axis follows the goal
   past final_deadband_m like the lateral one, so its command stays on the dock and its integral near zero while the
   vehicle, inside its small-effort band, stays. A new approach, from whichever side the vehicle is on, on a fresh tick
   once the fused along error has been past final_along_deadband_m for final_reapproach_s (a stale tick neither restarts
   nor ends that clock, as with settle_s). Each state record's final_along gives the phase, and each arrival and re-
   approach on the planner's clock. Under a controller that keeps the x/y integrals (the M3-C arm, retired) no set-point
   change clears the windup: every stage of its M3-0 run overshot, the 1.5 m stage by 19 cm.
7. The command walks to a stage's held goal (protocol v1.1): after each stage start, the x/y set point moves from the
   last command towards the held goal by at most speed_cap_mps / tick_hz per fresh tick (a set-point rate limit: it
   caps the commanded speed, not the vehicle's) and lands on it exactly; yaw is the held goal's from the stage start,
   and depth is the held depth (a stage start never changes it). v1.0 stepped straight to the goal in every stage
   but the last: on the M3-0 rerun the 4.1 m handover step held sway_stern at its ceiling from 42 to 252 s of
   mission time and the AUV turned 95 deg off the station, losing the tags for 197 s. The walk changes the x/y set
   point every tick, so mvp_control zeroes those integrals while it lasts (the anti-windup wanted there); depth is
   untouched, so the buoyancy integral is kept. The final stage's x/y re-targets of 6 are walked the same way. The
   refinement steps of 6 are not walked, and wait until the walk has landed. A stale tick pauses the walk. The final
   stage lasts final_stage_s from its start; then the planner reports complete and holds its set point.
Deterministic; no learning. ROS imports are lazy so the offline tests run without ROS.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from mission import is_planner_mission, parameters_sha256, validate_mission

NODE = "piccard_planner"
EKF_ODOMETRY = "/race_auv/odometry/filtered"
SETPOINT_TOPIC = "/race_auv/mvp_helm/bhv_direct_control/desired_setpoints"
STATE_TOPIC = "/piccard/planner/state"
START_SERVICE = f"/{NODE}/start"
WORLD_LINK, BASE_LINK, CG_LINK = "race_auv/world_ned", "race_auv/base_link", "race_auv/cg_link"
AUV_DOCK, STATION_DOCK = "race_auv/auv_dock_point", "race_station/dock_point"
PLANNER_TF_LOOKUPS = ((WORLD_LINK, BASE_LINK), (BASE_LINK, STATION_DOCK), (BASE_LINK, AUV_DOCK), (BASE_LINK, CG_LINK))
WAITING_STATE_PERIOD_S = 1.0
REHOLD_MIN_SAMPLES = 5  # fresh fused headings the final stage's re-hold needs (see 2 above)
ARRIVAL_NUDGE_M = 1e-4  # the arrival re-issue's change when the goal equals the held value exactly (see 6 above)
COMMAND_FIELDS = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")


# ----------------------------------------------------------------------------- rigid transforms: (rotation, position)
def quat_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def apply(rotation, vector):
    return [sum(rotation[i][k] * vector[k] for k in range(3)) for i in range(3)]


def compose(a, b):
    """a * b for transforms (rotation, position): b expressed in a's child frame."""
    return matmul(a[0], b[0]), [a[1][i] + v for i, v in enumerate(apply(a[0], b[1]))]


def inverse(a):
    rotation = transpose(a[0])
    return rotation, [-v for v in apply(rotation, a[1])]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def circular_median(angles) -> float:
    """The median of angles about their circular mean (deterministic; robust to a minority of outliers)."""
    reference = math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))
    return wrap(reference + statistics.median(wrap(a - reference) for a in angles))


def level_heading(x_axis_world):
    """A level FLU frame in world_ned (z down) whose x axis is x_axis_world projected on the horizontal plane."""
    hx, hy = x_axis_world[0], x_axis_world[1]
    norm = math.hypot(hx, hy)
    x = [hx / norm, hy / norm, 0.0]
    z = [0.0, 0.0, -1.0]  # FLU up in a z-down world
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [[x[i], y[i], z[i]] for i in range(3)]


# ----------------------------------------------------------------------------- the planner
class StagePlanner:
    """The planner's decisions, free of ROS: step() takes the time and the looked-up transforms, returns the record."""

    def __init__(self, parameters: dict, initial: dict | None = None):
        """initial: the set point the controller holds at the start (the fallback pose), if any."""
        self.p = parameters
        self.standoffs = list(parameters["standoffs_m"])
        self.stage = 0
        self.stage_started = None
        self.inside_since = None
        self.command = None
        self.complete = False
        self.transitions = []
        self.station = None  # the filtered station dock point: (world_ned position, heading of its x axis)
        self.observed_at = None
        self.held = {k: initial[k] for k in COMMAND_FIELDS} if initial else None
        self.command = dict(self.held) if self.held else None
        self.updates = {k: 0 for k in ("x_m", "y_m", "z_m", "yaw_rad")}
        self.arrived_since = None  # since when the vehicle has been at its held set point (fresh ticks)
        self.retarget_pending = False  # a stage has started and its x, y, yaw re-target waits for a fresh tick
        self.fused = None  # (D's transform, the stamp it first had): its run of bit-identical samples
        self.headings = []  # (time, D's heading in world_ned) of the last heading_window_s, while not held
        self.heading_held = None  # the heading from the second stage on
        self.previous_stage_started = None  # when the stage before the current one started
        self.rehold = None  # the final stage's heading re-hold, once: {t, samples, applied, previous, heading}
        self.walking = False  # a stage start's walk to its held goal has not landed yet (see 7 above)
        self.along_changed = None  # the final stage: when its along axis last changed (the stage start counts)
        self.along_phase = None  # the final stage's along axis: "approach" or "hold" (protocol v1.5, see 6 above)
        self.along_side = None  # the approach's side of the goal: -1 short (the stage start), +1 past
        self.outside_since = None  # in hold: since when the fused along error has been past final_along_deadband_m
        self.along_log = {"arrivals": [], "reapproaches": []}

    @property
    def final(self) -> bool:
        return self.stage == len(self.standoffs) - 1

    def observe(self, now: float, estimate) -> None:
        """The station estimate in world_ned (see 2 above): the heading, then the dock point re-anchored on the pivot
        and low-passed with time constant estimate_filter_s."""
        rotation, position = compose(estimate["base_in_world"], estimate["station_dock_in_base"])
        self.headings = [(t, h) for t, h in self.headings if now - t < self.p["heading_window_s"]]
        self.headings.append((now, math.atan2(rotation[1][0], rotation[0][0])))  # the final stage's re-hold uses it too
        heading = circular_median([h for _, h in self.headings]) if self.heading_held is None else self.heading_held
        pivot = self.p["tag_pivot_m"]
        anchor = [position[i] + v for i, v in enumerate(apply(rotation, pivot))]
        dock = [anchor[i] - v for i, v in enumerate(apply(level_heading([math.cos(heading), math.sin(heading), 0.0]),
                                                           pivot))]
        if self.station is None:
            self.station = (dock, heading)
        else:
            tau = self.p["estimate_filter_s"]
            alpha = 1.0 if tau <= 0 else 1.0 - math.exp(-(now - self.observed_at) / tau)
            old = self.station[0]
            self.station = ([old[i] + alpha * (dock[i] - old[i]) for i in range(3)], heading)
        self.observed_at = now

    def fresh(self, estimate) -> str | None:
        """None for a fresh estimate, else the stale_reason: "age" (none within max_estimate_age_s) or "frozen"."""
        if estimate is None:
            return "age"
        if self.frozen_for(estimate) > self.p["max_frozen_estimate_s"]:
            return "frozen"
        return None

    def wait(self, now: float, estimate) -> None:
        """A tick before the handover: observe a fresh estimate (the heading window and the dock-point filter), and
        nothing else: no command, no stage, no clock."""
        if self.stage_started is None and self.fresh(estimate) is None:
            self.observe(now, estimate)

    def hold_heading(self) -> None:
        """From the second stage on the heading is the first stage's (see 2 above)."""
        self.heading_held = self.station[1]
        self.station = (self.station[0], self.heading_held)

    def rehold_heading(self, now: float) -> None:
        """Once, at the final stage's first fresh tick: the held heading from the fresh fused headings of the stage
        before it within heading_window_s, or the previous one if fewer than REHOLD_MIN_SAMPLES (see 2 above)."""
        since = self.previous_stage_started if self.previous_stage_started is not None else -math.inf
        window = [h for t, h in self.headings if t >= since and now - t < self.p["heading_window_s"]]
        previous = self.heading_held if self.heading_held is not None else self.station[1]
        applied = len(window) >= REHOLD_MIN_SAMPLES
        self.heading_held = circular_median(window) if applied else previous
        self.station = (self.station[0], self.heading_held)
        self.rehold = {"t": now, "samples": len(window), "applied": applied, "previous": previous,
                       "heading": self.heading_held}

    def frozen_for(self, estimate) -> float:
        """How long, on the fuser's stamps, D's transform has stayed bit-identical."""
        transform, stamp = estimate["station_dock_in_base"], estimate["station_stamp"]
        if self.fused is None or transform != self.fused[0]:
            self.fused = (transform, stamp)
        return stamp - self.fused[1]

    def frame(self):
        """L: level, x along the filtered heading, z up (world_ned is z down), at the filtered position."""
        position, heading = self.station
        return level_heading([math.cos(heading), math.sin(heading), 0.0]), position

    def target(self) -> list[float]:
        """The stage target for the AUV dock point: stand-off s behind the station estimate on L's x axis, and
        approach_clearance_m above it on L's z axis (see 3 above)."""
        rotation, position = self.frame()
        s, c = self.standoffs[self.stage], self.p["approach_clearance_m"]
        return [position[i] - s * rotation[i][0] + c * rotation[i][2] for i in range(3)]

    def band(self) -> list[float]:
        """[along, lateral, vertical, heading] limits of the stage error (see 4 above)."""
        m, rad = self.p["band_m"], self.p["band_rad"]
        return [m, m + self.standoffs[self.stage] * rad, m if self.final else self.p["vertical_band_m"], rad]

    def error(self, estimate) -> list[float]:
        """[along, lateral, vertical]: the AUV dock point from the stage target in L; then the heading error."""
        rotation, _ = self.frame()
        target = self.target()
        auv = compose(estimate["base_in_world"], estimate["auv_dock_in_base"])
        relative = apply(transpose(rotation), [auv[1][i] - target[i] for i in range(3)])
        return relative + [wrap(math.atan2(auv[0][1][0], auv[0][0][0]) - self.station[1])]

    def goal(self, estimate) -> dict:
        """cg_link in world_ned with the AUV dock point on the stage target, level, heading along L's x axis."""
        dock_goal = (self.frame()[0], self.target())
        dock_to_cg = compose(inverse(estimate["auv_dock_in_base"]), estimate["cg_in_base"])
        rotation, position = compose(dock_goal, dock_to_cg)
        return {"x_m": position[0], "y_m": position[1], "z_m": position[2], "roll_rad": 0.0, "pitch_rad": 0.0,
                "yaw_rad": math.atan2(rotation[1][0], rotation[0][0])}

    def retarget(self, goal: dict, keys) -> list:
        """The held axes in keys to the goal where it differs by more than setpoint_deadband_m (_rad for yaw), at a
        stage start and in a refinement step alike (see 6 above); returns the axes that changed."""
        changed = []
        for key in keys:
            limit = self.p["setpoint_deadband_rad" if key == "yaw_rad" else "setpoint_deadband_m"]
            difference = wrap(goal[key] - self.held[key]) if key == "yaw_rad" else goal[key] - self.held[key]
            if abs(difference) > limit:
                self.held = {**self.held, key: goal[key]}
                self.updates[key] += 1
                changed.append(key)
        return changed

    def hold(self, now: float, goal: dict, estimate, error, stage_started: bool) -> dict:
        """The set point in discrete steps: x, y, yaw at a stage start; then the axes a fresh error is outside the band
        on, once the vehicle has been at its set point for settle_s; in the final stage the lateral axis follows the
        live goal past final_deadband_m, and the along axis approaches, arrives and holds (see 6 above)."""
        if self.final and (stage_started or self.along_changed is None):  # the stage start counts as an along change
            self.along_changed = now
        if self.held is None:  # no set point held before the planner: its first goal is one
            self.held = dict(goal)
            return self.held
        if stage_started:
            self.retarget(goal, ("x_m", "y_m", "yaw_rad"))
            self.arrived_since, self.walking = None, True
        if self.final:  # per world axis, walked; yaw and depth held; no settle wait
            along = self.along_key()
            if stage_started or self.along_phase is None:  # the stage start begins the approach, from short
                self.along_phase, self.along_side, self.outside_since = "approach", -1 if error[0] < 0 else 1, None
            self.final_along(now, goal, error[0], along)
            for key in ("x_m", "y_m"):  # the lateral axis, and the along axis in hold: follow past final_deadband_m
                if (key != along or self.along_phase == "hold") \
                        and abs(goal[key] - self.held[key]) > self.p["final_deadband_m"]:
                    self.set_axis(key, goal[key], now)
            return self.held
        if self.walking:  # no refinement before the walk has landed
            return self.held
        if self.arrived(estimate):
            self.arrived_since = self.arrived_since if self.arrived_since is not None else now
        else:
            self.arrived_since = None
        if self.arrived_since is not None and now - self.arrived_since >= self.p["settle_s"]:
            outside = [abs(v) > limit for v, limit in zip(error, self.band())]
            keys = (("x_m", "y_m") if outside[0] or outside[1] else ()) + (("z_m",) if outside[2] else ()) \
                + (("yaw_rad",) if outside[3] else ())
            if self.retarget(goal, keys):
                self.arrived_since = None
        return self.held

    def set_axis(self, key: str, value: float, now: float) -> None:
        """One final-stage x or y change, walked (see 7 above); an along change restarts the along interval."""
        self.held = {**self.held, key: value}
        self.updates[key] += 1
        self.walking = True
        if key == self.along_key():
            self.along_changed = now

    def final_along(self, now: float, goal: dict, along_error: float, along: str) -> None:
        """The final stage's along axis (protocol v1.5, see 6 above). Approach: re-target only a stale command (goal
        past final_along_deadband_m from the held value, final_along_interval_s since the last along change). Arrival:
        the fused along error within final_arrival_m of the goal on the approach's side re-issues the along set point
        at the goal, once, and holds. Hold: the along axis follows the goal like the lateral one (in hold()); an error
        past final_along_deadband_m for final_reapproach_s starts a new approach from its side, on a fresh tick (a
        stale tick neither restarts nor ends that clock, as with settle_s)."""
        if self.along_phase == "approach":
            if self.along_side * along_error <= self.p["final_arrival_m"]:
                nudged = goal[along] == self.held[along]
                change = ARRIVAL_NUDGE_M if nudged else goal[along] - self.held[along]
                self.set_axis(along, self.held[along] + ARRIVAL_NUDGE_M if nudged else goal[along], now)
                self.along_phase, self.outside_since = "hold", None
                self.along_log["arrivals"].append({"t": now, "error_m": along_error, "side": self.along_side,
                                                   "change_m": change})
            elif abs(goal[along] - self.held[along]) > self.p["final_along_deadband_m"] \
                    and now - self.along_changed >= self.p["final_along_interval_s"]:
                self.set_axis(along, goal[along], now)
            return
        if abs(along_error) <= self.p["final_along_deadband_m"]:
            self.outside_since = None
            return
        self.outside_since = self.outside_since if self.outside_since is not None else now
        if now - self.outside_since >= self.p["final_reapproach_s"]:
            self.along_phase, self.along_side, self.outside_since = "approach", -1 if along_error < 0 else 1, None
            self.along_log["reapproaches"].append({"t": now, "error_m": along_error, "side": self.along_side})

    def along_key(self) -> str:
        """The world axis closer to L's x, the approach (see 6 above): y_m at the RACE station's heading (about 90
        deg in north-east-down)."""
        heading = self.station[1]
        return "y_m" if abs(math.sin(heading)) >= abs(math.cos(heading)) else "x_m"

    def arrived(self, estimate) -> bool:
        """The vehicle at its held set point: cg_link within band_m of it on x and y and within vertical_band_m on z
        (see 6 above), so a depth step's integral tail keeps the refinement check closed."""
        cg = compose(estimate["base_in_world"], estimate["cg_in_base"])[1]
        limits = (self.p["band_m"], self.p["band_m"], self.p["vertical_band_m"])
        return all(abs(cg[i] - self.held[k]) <= limits[i] for i, k in enumerate(("x_m", "y_m", "z_m")))

    def walk(self, held: dict) -> dict:
        """The command (see 7 above): while a stage start's walk has not landed, the x/y set point moves from the last
        command towards held by at most speed_cap_mps / tick_hz and lands on it exactly; every other axis is held's.
        Otherwise held itself, so a refinement step is one step."""
        if self.command is None or not self.walking:
            self.walking = False
            return dict(held)
        step = self.p["speed_cap_mps"] / self.p["tick_hz"]
        delta = [held[k] - self.command[k] for k in ("x_m", "y_m")]
        norm = math.hypot(*delta)
        if norm <= step:
            self.walking = False
            return dict(held)
        return {**held, **{k: self.command[k] + step / norm * d for k, d in zip(("x_m", "y_m"), delta)}}

    def step(self, now: float, estimate) -> dict:
        event = None
        if self.stage_started is None:
            self.stage_started, event, self.retarget_pending = now, "stage_start", True
            self.transitions.append({"t": now, "stage": 0, "standoff_m": self.standoffs[0]})
        record = {"stage": self.stage, "standoff_m": self.standoffs[self.stage], "error": None, "goal": None,
                  "station": None, "stale_reason": None}
        if self.complete:
            return {**record, "state": "complete", "event": None, "command": self.command, "inside_s": None,
                    "setpoint_updates": dict(self.updates), "heading_held": self.heading_held, "held": self.held,
                    "heading_samples": len(self.headings), "heading_rehold": self.rehold,
                    "final_along": self.final_along_record()}
        stale = self.fresh(estimate)
        if stale is not None:
            state, record["stale_reason"] = "stale", stale
        else:
            state = "tracking"
            self.observe(now, estimate)
            if self.final and self.rehold is None:
                self.rehold_heading(now)
            record["station"] = self.station[0] + [self.station[1]]
            error = self.error(estimate)
            record["error"] = error
            inside = all(abs(v) <= limit for v, limit in zip(error, self.band()))
            self.inside_since = (self.inside_since if self.inside_since is not None else now) if inside else None
            goal = self.goal(estimate)
            record["goal"] = {**goal, "approach_clearance_m": self.p["approach_clearance_m"]}
            held = self.hold(now, goal, estimate, error, self.retarget_pending)
            self.retarget_pending = False
            self.command = self.walk(held)
            if (not self.final and self.inside_since is not None
                    and now - self.inside_since >= self.p["settle_s"]):
                self.stage += 1
                self.previous_stage_started = self.stage_started
                self.stage_started, self.inside_since, event = now, None, "stage_start"
                if self.heading_held is None:  # the first stage has ended
                    self.hold_heading()
                self.retarget_pending = True
                self.transitions.append({"t": now, "stage": self.stage, "standoff_m": self.standoffs[self.stage]})
        if self.final and now - self.stage_started >= self.p["final_stage_s"]:
            self.complete, state, event = True, "complete", "complete"
            self.transitions.append({"t": now, "stage": self.stage, "complete": True})
        return {**record, "stage": self.stage, "standoff_m": self.standoffs[self.stage], "state": state, "event": event,
                "command": self.command, "setpoint_updates": dict(self.updates), "heading_held": self.heading_held,
                "held": self.held, "heading_samples": len(self.headings), "heading_rehold": self.rehold,
                "final_along": self.final_along_record(),
                "inside_s": now - self.inside_since if self.inside_since is not None else None}

    def final_along_record(self) -> dict | None:
        """The final stage's along phase and its arrivals and re-approaches (planner clock), for the collector."""
        if self.along_phase is None:
            return None
        return {"phase": self.along_phase, "side": self.along_side, "arrivals": list(self.along_log["arrivals"]),
                "reapproaches": list(self.along_log["reapproaches"])}


# ----------------------------------------------------------------------------- ROS node
def ros_node_class():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener
    from mvp_msgs.msg import ControlProcess

    reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

    class Planner(Node):
        def __init__(self, mission: dict):
            super().__init__(NODE)
            self.parameters = mission["planner"]
            self.parameters_sha256 = parameters_sha256(self.parameters)
            self.planner = StagePlanner(self.parameters, initial=mission["fallback_pose"])
            self.created = time.monotonic()  # the planner's clock, waiting ticks included
            self.started = None
            self.ticks = 0
            self.last_waiting = 0.0
            self.odometry_received = None
            self.static = {}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Odometry, EKF_ODOMETRY, self.odometry, qos_profile_sensor_data)
            self.setpoints = self.create_publisher(ControlProcess, SETPOINT_TOPIC, 10)
            self.state = self.create_publisher(String, STATE_TOPIC, reliable)
            self.create_service(Trigger, START_SERVICE, self.start)
            self.create_timer(1.0 / self.parameters["tick_hz"], self.tick)

        def odometry(self, _msg):
            self.odometry_received = time.monotonic()

        def start(self, _request, response):
            if self.started is None:
                self.started = time.monotonic()
            response.success, response.message = True, self.parameters_sha256
            return response

        def lookup(self, parent, child, max_age=None):
            """((rotation, position), stamp) of the latest transform, or None if missing or older than max_age."""
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, Time())
            except Exception:
                return None
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if max_age is not None and self.get_clock().now().nanoseconds / 1e9 - stamp > max_age:
                return None
            q, v = transform.transform.rotation, transform.transform.translation
            return (quat_matrix(q.x, q.y, q.z, q.w), [v.x, v.y, v.z]), stamp

        def estimate(self):
            age = self.parameters["max_estimate_age_s"]
            if self.odometry_received is None or time.monotonic() - self.odometry_received > age:
                return None
            for child, key in ((AUV_DOCK, "auv_dock_in_base"), (CG_LINK, "cg_in_base")):  # static: look up once
                if key not in self.static:
                    found = self.lookup(BASE_LINK, child)
                    if found is None:
                        return None
                    self.static[key] = found[0]
            station = self.lookup(BASE_LINK, STATION_DOCK, age)
            world = self.lookup(WORLD_LINK, BASE_LINK)
            if station is None or world is None:
                return None
            return {**self.static, "station_dock_in_base": station[0], "station_stamp": station[1],
                    "base_in_world": world[0]}

        def publish_state(self, record):
            msg = String()
            msg.data = json.dumps({**record, "tick": self.ticks, "parameters_sha256": self.parameters_sha256,
                                   "tf_lookups": [f"{p}->{c}" for p, c in PLANNER_TF_LOOKUPS]},
                                  separators=(",", ":"), allow_nan=False)
            self.state.publish(msg)

        def tick(self):
            now = time.monotonic()
            if self.started is None:  # the fallback pose's dwell: observe only (see 2 above)
                self.planner.wait(now - self.created, self.estimate())
                if now - self.last_waiting >= WAITING_STATE_PERIOD_S:
                    self.last_waiting = now
                    self.publish_state({"state": "waiting", "event": None, "stage": None, "command": None,
                                        "heading_samples": len(self.planner.headings)})
                return
            self.ticks += 1
            record = self.planner.step(now - self.created, self.estimate())
            command = record["command"]
            if command is not None:
                msg = ControlProcess()
                msg.header.frame_id, msg.child_frame_id, msg.control_mode = WORLD_LINK, CG_LINK, "docking"
                msg.position.x, msg.position.y, msg.position.z = (float(command[k]) for k in ("x_m", "y_m", "z_m"))
                msg.orientation.x, msg.orientation.y, msg.orientation.z = (
                    float(command[k]) for k in ("roll_rad", "pitch_rad", "yaw_rad"))
                self.setpoints.publish(msg)
            self.publish_state({**record, "t": now - self.started})

    return Planner, rclpy


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mission-profile-json", type=Path, required=True)
    args = parser.parse_args(argv)
    mission = validate_mission(json.loads(args.mission_profile_json.read_text()))
    if not is_planner_mission(mission):
        parser.error("the planner runs only for a piccard.race-auv.planner-mission/v1 mission")
    Planner, rclpy = ros_node_class()
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = Planner(mission)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
