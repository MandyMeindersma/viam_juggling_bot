import asyncio
import math
import sys

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
from viam.services.motion import MotionClient
from viam.proto.service.motion import MotionConfiguration

from viam.proto.component.arm import MoveOptions
from viam.proto.service.motion import MoveRequest
from viam.proto.common import PoseInFrame, Pose

import connect

from google.protobuf.json_format import MessageToJson



class Juggler:
    def __init__(self, robot):
        self.robot = robot
        self.arm = Arm.from_robot(self.robot, "arm")
        self.motion = MotionClient.from_robot(self.robot, "builtin")

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        await self.arm.move_to_joint_positions(JointPositions(values=[43.91, -90.07, -215.38, 5.55, 32.72, -11.52]))


        

    async def throw(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        await self.arm.move_to_joint_positions(JointPositions(values=[43.91, -70, -215.38, 5.55, 32.72, -11.52]))

        

    # async def throw(self):
    #     """Throw the ball"""
    #     await self.arm.move_to_joint_positions(
    #         JointPositions(values=[5.0, -38.4, 4.3, 0.7, -64.5, 0.4])
    #     )




    


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
