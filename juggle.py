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
STROKE = 90.0
# Largest joint change allowed between MIDDLE and TOP. A 90mm lift moves each joint a few
# degrees; anything near this means IK landed on a different arm configuration, and snapping
# between two of those at throw speed would swing the whole arm.
MAX_STROKE_DEG = 45.0


def shifted(pose, dx=0.0, dy=0.0, dz=0.0):
    out = Pose()
    out.CopyFrom(pose)
    out.x += dx
    out.y += dy
    out.z += dz
    return out


MIDDLE = shifted(ZERO, dx=-RETRACT, dz=RAISE)
TOP = shifted(MIDDLE, dz=STROKE)

# A single move_to_position/move_to_joint_positions call this far can trip the controller's
# per-move speed ceiling ("xArm: Linear speed exceeds limit in ServoJ mode") -- seen whenever
# the arm starts far from the target. Big repositioning moves go through intermediate waypoints
# no farther apart than this instead; a step that still faults backs off toward this floor.
APPROACH_STEP_MM = 150.0
MIN_APPROACH_STEP_MM = 5.0


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

    async def throw(self, vel=180.0, acc=4000.0):
        """Toss whatever rests on top of the gripper: stage at MIDDLE, snap straight up by
        STROKE and straight back down to MIDDLE. A slow rehearsal lets the arm's own IK resolve
        the joint configuration at each end of the stroke; the up-and-return then runs as one
        fast joint trajectory between them (over a lift this short the joint path is straight
        to within a few mm), and the arm never pauses at the top -- the object keeps rising as
        the gripper decelerates out from under it."""
        await self._approach(MIDDLE)
        middle_joints = await self._joints()
        await self.arm.move_to_position(TOP)
        top_joints = await self._joints()
        await self.arm.move_to_joint_positions(JointPositions(values=middle_joints))

        swing = max(abs(t - m) for t, m in zip(top_joints, middle_joints))
        if swing > MAX_STROKE_DEG:
            raise RuntimeError(
                f"IK put TOP {swing:.0f}deg from MIDDLE on one joint; refusing to throw"
            )

        # The object only separates if the gripper decelerates faster than gravity. The joint
        # that moves most sets the timing, so its share of the stroke gives the tip's rate.
        tip_decel = STROKE / swing * acc / 1000
        print(
            f"throw: {STROKE:.0f}mm stroke, {swing:.1f}deg on the widest joint, "
            f"tip decel ~{tip_decel:.1f} m/s^2"
            + (" -- under 9.8, object will not separate" if tip_decel < 9.8 else "")
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

    async def _approach(self, target, step_mm=APPROACH_STEP_MM):
        """Move to a Cartesian pose in position increments along the straight line from wherever
        the arm currently sits, holding the target's orientation throughout. If a given step
        trips the controller's ServoJ speed fault, that step alone is retried at half the size
        rather than failing the whole approach -- it's a mechanical limit on how much ground one
        call can cover, not a sign the step's destination is bad. Any other error (self-collision,
        unreachable) means the target itself is the problem and is raised immediately."""
        current = await self.arm.get_end_position()
        total = math.dist((current.x, current.y, current.z), (target.x, target.y, target.z))
        if total < 1.0:
            await self.arm.move_to_position(target)
            return

        ux = (target.x - current.x) / total
        uy = (target.y - current.y) / total
        uz = (target.z - current.z) / total
        pos = [current.x, current.y, current.z]
        traveled = 0.0

        while traveled < total - 1e-6:
            this_step = min(step_mm, total - traveled)
            candidate = [pos[0] + ux * this_step, pos[1] + uy * this_step, pos[2] + uz * this_step]
            waypoint = shifted(target, dx=candidate[0] - target.x, dy=candidate[1] - target.y, dz=candidate[2] - target.z)
            try:
                await self.arm.move_to_position(waypoint)
            except GRPCError as e:
                if SERVO_SPEED_FAULT in str(e) and step_mm > MIN_APPROACH_STEP_MM:
                    step_mm = max(step_mm / 2, MIN_APPROACH_STEP_MM)
                    continue
                raise
            pos, traveled = candidate, traveled + this_step

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
