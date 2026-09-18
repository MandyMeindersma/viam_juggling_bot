import asyncio
import json
import math
import sys

from grpclib.exceptions import GRPCError

from viam.components.arm import Arm
from viam.proto.common import Pose
from viam.proto.component.arm import JointPositions, MoveOptions, MoveThroughJointPositionsRequest

import connect

# Substring of the error the xArm controller raises when a single move covers too much ground
# for its ServoJ streaming mode -- a mechanical execution limit, not a reachability problem.
# Seen even on moves Viam's own motion planner already validated as collision-free.
SERVO_SPEED_FAULT = "Linear speed exceeds limit in ServoJ mode"

# Joints orbited to trace the circle: wrist and gripper_rot.
COS_JOINT = 4
SIN_JOINT = 5
# Headroom kept off each travel limit; the controller rejects out-of-range targets outright.
LIMIT_MARGIN = 1.0

# Flat, outstretched home pose (mm / orientation vector / degrees).
ZERO = Pose(
    x=734.3404649781442,
    y=0.7709977991432595,
    z=246.41067435410184,
    o_x=0.9998925877347096,
    o_y=0,
    o_z=0,
    theta=174.8205202973959,
)

# Staging offset from ZERO. ZERO sits inside wall-front's box (the gripper's own body extends
# beyond the nominal tip point along the tool axis), so the staging pose must retract past the
# wall's near face, not just off the tip's exact position. Measured minimum retract to clear
# it is ~200mm; this keeps 50mm of margin.
RETRACT = 250.0
RAISE = 50.0
# Height of the throw stroke above MIDDLE.
STROKE = 200.0
# The staging point and the throw are swung this far about the base's vertical axis
# (right-hand rule about world Z). Negative turns the arm to its right when facing +x. The
# hand stays level and the stroke stays vertical; only where the arm points changes.
YAW_DEG = -30.0
# Largest joint change allowed between MIDDLE and TOP. The stroke moves each joint a few tens
# of degrees; anything near this means IK landed on a different arm configuration, and
# snapping between two of those at throw speed would swing the whole arm.
MAX_STROKE_DEG = 70.0


def shifted(pose, dx=0.0, dy=0.0, dz=0.0):
    out = Pose()
    out.CopyFrom(pose)
    out.x += dx
    out.y += dy
    out.z += dz
    return out


def yawed(pose, deg):
    """Pose swung about the world Z axis through the base (right-hand rule): position and
    tool axis both rotate, so the hand keeps facing along the arm. Theta is unchanged --
    Viam's orientation-vector theta is defined relative to world Z, so a yaw leaves it."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    out = Pose()
    out.CopyFrom(pose)
    out.x, out.y = pose.x * c - pose.y * s, pose.x * s + pose.y * c
    out.o_x, out.o_y = pose.o_x * c - pose.o_y * s, pose.o_x * s + pose.o_y * c
    return out


def blend(a, b, t):
    """Pose partway (t in 0..1) from a to b: position lerped, tool axis slerped along the
    great circle, theta lerped the short way round. Antiparallel axes have no unique great
    circle, so the tool swings through +z (up) rather than through the base."""
    a_axis = _unit((a.o_x, a.o_y, a.o_z))
    b_axis = _unit((b.o_x, b.o_y, b.o_z))
    omega = math.acos(max(-1.0, min(1.0, sum(i * j for i, j in zip(a_axis, b_axis)))))
    if omega > math.radians(170.0):
        # Near-antiparallel slerp is ill-conditioned (weights blow up as sin(omega) -> 0), so
        # swing through +z (or +y if the axis already is z) in two well-behaved halves.
        mid = (0.0, 1.0, 0.0) if abs(a_axis[2]) > 0.9 else (0.0, 0.0, 1.0)
        axis = _slerp(a_axis, mid, t * 2) if t < 0.5 else _slerp(mid, b_axis, t * 2 - 1)
    else:
        axis = _slerp(a_axis, b_axis, t)

    dtheta = (b.theta - a.theta + 180.0) % 360.0 - 180.0
    theta = (a.theta + dtheta * t + 180.0) % 360.0 - 180.0
    return Pose(
        x=a.x + (b.x - a.x) * t,
        y=a.y + (b.y - a.y) * t,
        z=a.z + (b.z - a.z) * t,
        o_x=axis[0], o_y=axis[1], o_z=axis[2],
        theta=theta,
    )


def _unit(v):
    n = math.sqrt(sum(i * i for i in v))
    return tuple(i / n for i in v)


def _slerp(a, b, t):
    omega = math.acos(max(-1.0, min(1.0, sum(i * j for i, j in zip(a, b)))))
    if omega < 1e-6:
        return b
    wa, wb = math.sin((1 - t) * omega) / math.sin(omega), math.sin(t * omega) / math.sin(omega)
    return _unit(tuple(wa * i + wb * j for i, j in zip(a, b)))


def axis_angle_deg(a, b):
    """Angle between two poses' tool axes, in degrees."""
    dot = a.o_x * b.o_x + a.o_y * b.o_y + a.o_z * b.o_z
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


MIDDLE = yawed(shifted(ZERO, dx=-RETRACT, dz=RAISE), YAW_DEG)
TOP = shifted(MIDDLE, dz=STROKE)

# A single move_to_position/move_to_joint_positions call this far can trip the controller's
# per-move speed ceiling ("xArm: Linear speed exceeds limit in ServoJ mode") -- seen whenever
# the arm starts far from the target. Big repositioning moves go through intermediate waypoints
# no farther apart than this instead; a step that still faults backs off toward this floor.
APPROACH_STEP_MM = 150.0
MIN_APPROACH_STEP_MM = 5.0
# Likewise for the tool's rotation between waypoints, so a big reorientation is spread over
# the path instead of forced on the first step.
APPROACH_STEP_DEG = 20.0


class Juggler:
    def __init__(self, robot):
        self.robot = robot
        self.arm = Arm.from_robot(self.robot, "arm")

    async def zero(self):
        """Return the arm to the flat, outstretched home pose."""
        await self._approach(ZERO)

    async def middle(self):
        """Park at the staging pose the throw launches from and returns to."""
        await self._approach(MIDDLE)

    # Measured ceiling: the driver silently clamps joint speed to 180deg/s and acceleration to
    # ~4000deg/s^2 (xArm6 hardware limits). Passing more changes nothing -- timed identical
    # strokes from 180 to 900deg/s. More launch velocity has to come from geometry: a longer
    # lever arm (less RETRACT) moves the tip further per degree at the same joint speed.
    async def throw(self, vel=180.0, acc=4000.0):
        """Toss whatever rests on top of the gripper: stage at MIDDLE, snap straight up by
        STROKE and straight back down to MIDDLE. A slow rehearsal lets the arm's own IK resolve
        the joint configuration at each end of the stroke; the up-and-return then runs as one
        fast joint trajectory between them (over a lift this short the joint path is straight
        to within a few mm), and the arm never pauses at the top -- the object keeps rising as
        the gripper decelerates out from under it."""
        await self._approach(MIDDLE)
        middle_joints = await self._joints()
        await self._approach(TOP)
        top_joints = await self._joints()
        await self.arm.move_to_joint_positions(JointPositions(values=middle_joints))

        swing = max(abs(t - m) for t, m in zip(top_joints, middle_joints))
        if swing > MAX_STROKE_DEG:
            raise RuntimeError(
                f"IK put TOP {swing:.0f}deg from MIDDLE on one joint; refusing to throw"
            )

        # The joint that moves most sets the timing, so its share of the stroke converts joint
        # rates to tip rates. The object leaves at the tip's peak velocity, which is the
        # ceiling only if the joint can reach it before it must start decelerating; over a
        # short swing the peak is acceleration-limited (triangular profile) instead. It only
        # separates at all if the gripper then decelerates faster than gravity.
        mm_per_deg = STROKE / swing
        peak_deg_s = min(vel, math.sqrt(acc * swing))
        launch_ms = mm_per_deg * peak_deg_s / 1000
        tip_decel = mm_per_deg * acc / 1000
        print(
            f"throw: {STROKE:.0f}mm stroke yawed {YAW_DEG:+.0f}deg, {swing:.1f}deg on the widest joint, "
            f"launch ~{launch_ms:.1f} m/s (~{launch_ms**2 / 19.6 * 1000:.0f}mm high)"
            + (f", {'vel' if peak_deg_s == vel else 'acc'}-limited")
            + (", tip decel under 9.8 m/s^2 -- object will not separate" if tip_decel < 9.8 else "")
        )
        await self._move_through([top_joints, middle_joints], vel, acc)
        # Settle on MIDDLE explicitly so the throw always ends there, whatever the fast
        # trajectory's final blend left behind.
        await self.arm.move_to_joint_positions(JointPositions(values=middle_joints))

    async def circle(self, amplitude=10.0, steps=12):
        """Orbit two joints to trace a circle, working in joint space rather than re-solving
        Cartesian IK (which can land on an alternate, self-colliding joint solution even for a
        pose the arm is already sitting in). The circle is shrunk and recentered to fit the
        joints' travel limits, so the first waypoint repositions the arm somewhere the whole
        sweep is reachable."""
        limits = await self._joint_limits()
        base = list((await self.arm.get_joint_positions()).values)

        cos_lo, cos_hi = limits[COS_JOINT]
        sin_lo, sin_hi = limits[SIN_JOINT]
        cos_lo, cos_hi = cos_lo + LIMIT_MARGIN, cos_hi - LIMIT_MARGIN
        sin_lo, sin_hi = sin_lo + LIMIT_MARGIN, sin_hi - LIMIT_MARGIN

        amplitude = min(amplitude, (cos_hi - cos_lo) / 2, (sin_hi - sin_lo) / 2)
        cos_center = min(max(base[COS_JOINT], cos_lo + amplitude), cos_hi - amplitude)
        sin_center = min(max(base[SIN_JOINT], sin_lo + amplitude), sin_hi - amplitude)

        moved = (cos_center, sin_center) != (base[COS_JOINT], base[SIN_JOINT])
        print(
            f"circle: amplitude {amplitude:.1f}deg, center joint{COS_JOINT}={cos_center:.1f} "
            f"joint{SIN_JOINT}={sin_center:.1f}"
            + (" (repositioned to fit limits)" if moved else "")
        )

        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            joints = list(base)
            joints[COS_JOINT] = cos_center + amplitude * math.cos(angle)
            joints[SIN_JOINT] = sin_center + amplitude * math.sin(angle)
            await self.arm.move_to_joint_positions(JointPositions(values=joints))

    async def _move_through(self, waypoints, vel, acc):
        """Run waypoints as one continuous trajectory under velocity/acceleration ceilings.
        SDK 0.80 ships no wrapper for this RPC, so it goes through the stub directly."""
        request = MoveThroughJointPositionsRequest(
            name=self.arm.name,
            positions=[JointPositions(values=w) for w in waypoints],
            options=MoveOptions(max_vel_degs_per_sec=vel, max_acc_degs_per_sec2=acc),
        )
        await self.arm.client.MoveThroughJointPositions(request, metadata=self.arm.Metadata().proto)

    async def _approach(self, target):
        """Move to a Cartesian pose through waypoints blended from wherever the arm currently
        sits: position and tool orientation advance together, so no intermediate pose asks for
        the target's orientation at a position that can't support it. Steps are bounded in both
        travel and rotation. If a step trips the controller's ServoJ speed fault, that step
        alone is retried at half the size -- it's a mechanical limit on how much ground one call
        can cover, not a sign the step's destination is bad. Any other error (self-collision,
        unreachable) means the target itself is the problem and is raised immediately."""
        start = await self.arm.get_end_position()
        travel = math.dist((start.x, start.y, start.z), (target.x, target.y, target.z))
        turn = axis_angle_deg(start, target)
        if travel < 1.0 and turn < 0.5:
            await self.arm.move_to_position(target)
            return

        # Fraction of the whole path one step may cover, from whichever of distance or
        # rotation is the tighter bound.
        step = 1.0 / max(1, math.ceil(travel / APPROACH_STEP_MM), math.ceil(turn / APPROACH_STEP_DEG))
        floor = step * MIN_APPROACH_STEP_MM / APPROACH_STEP_MM
        t = 0.0
        while t < 1.0 - 1e-9:
            t_next = min(1.0, t + step)
            try:
                await self.arm.move_to_position(target if t_next >= 1.0 else blend(start, target, t_next))
            except GRPCError as e:
                if SERVO_SPEED_FAULT in str(e) and step > floor:
                    step = max(step / 2, floor)
                    continue
                raise
            t = t_next

    async def _joints(self):
        return list((await self.arm.get_joint_positions()).values)

    async def _joint_limits(self):
        """Per-joint (min, max) in degrees, read from the arm's own kinematics config."""
        _, kinematics = (await self.arm.get_kinematics())[:2]
        joints = json.loads(kinematics.decode("utf-8"))["joints"]
        return [(j["min"], j["max"]) for j in joints]


# verb -> method. One entry per capability.
STEPS = {
    "throw": Juggler.throw,
    "zero": Juggler.zero,
    "middle": Juggler.middle,
    "circle": Juggler.circle,
}


async def main(verb):
    robot = await connect.connect()
    juggler = Juggler(robot)
    try:
        step = STEPS.get(verb)
        if step is None:
            print(f"unknown step '{verb}'. steps: {', '.join(STEPS)}")
            return
        await step(juggler)
    finally:
        await robot.close()


if __name__ == "__main__":
    verb = sys.argv[1] if len(sys.argv) > 1 else "zero"
    asyncio.run(main(verb))
