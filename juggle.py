import asyncio
import json
import math
import sys

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions, MoveOptions, MoveThroughJointPositionsRequest

import connect

# Joints orbited to trace the circle: wrist and gripper_rot.
COS_JOINT = 4
SIN_JOINT = 5
# Headroom kept off each travel limit; the controller rejects out-of-range targets outright.
LIMIT_MARGIN = 1.0

# Shoulder, elbow and wrist rotate about parallel axes, so the tool's pitch tracks their sum:
# any change that nets to zero across the three leaves the gripper's top face level.
SHOULDER, ELBOW, WRIST = 1, 2, 4

ZERO = [5, -5.6, 5.9, 2.0, -92.4, 0.4]
# Degrees the elbow opens to reach the middle position, measured to put the tip roughly
# 110mm further out and 45mm higher than ZERO, with room above for the throw stroke.
EXTEND = 20.0


def level_adjust(pose, joint, delta):
    """Rotate one pitch joint and give the wrist the opposite delta, so the tool stays level."""
    out = list(pose)
    out[joint] += delta
    out[WRIST] -= delta
    return out


MIDDLE = level_adjust(ZERO, ELBOW, -EXTEND)


class Juggler:
    def __init__(self, robot):
        self.robot = robot
        self.arm = Arm.from_robot(self.robot, "arm")

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        await self.arm.move_to_joint_positions(JointPositions(values=ZERO))

    async def middle(self):
        """Park at the staging pose the throw launches from and returns to."""
        await self.arm.move_to_joint_positions(JointPositions(values=MIDDLE))

    # The object only separates if the gripper decelerates faster than gravity. The stroke
    # covers ~3.6mm of tip travel per degree, so acc must clear ~2700deg/s^2 to beat 9.8m/s^2;
    # the default sits about 1.5x above that.
    async def throw(self, lift=25.0, vel=180.0, acc=4000.0):
        """Toss whatever is resting on top of the gripper: stage at the middle position, then
        snap the tip up and straight back down to it. The up-and-return is sent as a single
        trajectory so the arm never pauses at the top -- the object keeps rising as the gripper
        decelerates out from under it. The lift comes from the shoulder with the wrist
        compensating, which keeps the gripper's top face level through the whole stroke."""
        top = level_adjust(MIDDLE, SHOULDER, -lift)
        self._check_limits(await self._joint_limits(), {"middle": MIDDLE, "top": top})

        await self.arm.move_to_joint_positions(JointPositions(values=MIDDLE))
        print(f"throw: lift {lift:.0f}deg at {vel:.0f}deg/s, {acc:.0f}deg/s^2")
        await self._move_through([top, MIDDLE], vel, acc)

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

    def _check_limits(self, limits, poses):
        for label, pose in poses.items():
            for index, (value, (low, high)) in enumerate(zip(pose, limits)):
                if not low + LIMIT_MARGIN <= value <= high - LIMIT_MARGIN:
                    raise ValueError(
                        f"{label} pose puts joint {index} at {value:.1f}, outside [{low}, {high}]"
                    )

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
