import asyncio
import sys

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions

import connect



class Juggler:
    def __init__(self, robot):
        self.robot = robot

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        arm = Arm.from_robot(self.robot, "arm")
        await arm.move_to_joint_positions(
            JointPositions(values=[5, -5.6, 5.9, 2.0, -92.4, 0.4])
        )

    async def throw(self):
        """Throw the ball"""
        arm = Arm.from_robot(self.robot, "arm-1")




    


# verb -> method. One entry per capability.
STEPS = {
    "throw": Juggler.throw,
    "zero": Juggler.zero,
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
