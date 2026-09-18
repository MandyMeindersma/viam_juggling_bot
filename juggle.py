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
        zero = Pose(x=-608.8575681802733, y=-14.280484044140682, z=-27.480223424094945, o_x=-0.9953762680748285, o_y=-0.001203186670059822, o_z=0.09604497537749951, theta=176.1478741307629)

        req = MoveRequest(
            name="builtin",
            component_name="arm",
            destination=PoseInFrame(reference_frame="world", pose=zero),
            world_state=None,
            constraints=None,
        )
        resp = await self.motion.do_command({"plan":MessageToJson(req)})
        trajectory = resp["plan"]

        positions = [
            JointPositions(values=[math.degrees(v) for v in step["arm"]])
            for step in trajectory
            if step.get("arm")
        ]
        options = MoveOptions(
            max_acc_degs_per_sec2=90,
            max_vel_degs_per_sec=90
        )

        await self.arm.move_through_joint_positions(positions, options)

        

    async def throw(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        # pose = PoseInFrame(reference_frame="world", pose=Pose(x=-620.8801241253041, y=13.065989629810097, z=564.3014151250942, o_x=-0.9994030742493492, o_y=0.016881295816552816, o_z=0.03014161628884865, theta=174.6887362033622))
        pose = PoseInFrame(reference_frame="world", pose=Pose(x=-690.7447881801012,y=11.93337068601625,z=431.2269891028385,o_x=-0.9997518095214151,o_y=.00631436298081086,o_z=.021364647874532516,theta=175.78125152270124))

        req = MoveRequest(
            name="builtin",
            component_name="arm",
            destination=pose,
            world_state=None,
            constraints=None
        )
        resp = await self.motion.do_command({"plan":MessageToJson(req)})
        trajectory = resp["plan"]

        positions = [
            JointPositions(values=[math.degrees(v) for v in step["arm"]])
            for step in trajectory
            if step.get("arm")
        ]
        
        await self.arm.move_through_joint_positions(
            positions,
            options= MoveOptions(
                max_acc_degs_per_sec2=1100,
                max_vel_degs_per_sec=180
            )
        )

        

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
