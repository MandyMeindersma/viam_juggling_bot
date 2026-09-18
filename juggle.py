import asyncio
import sys

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
from viam.services.motion import MotionClient
from viam.proto.service.motion import MotionConfiguration

from viam.proto.common import PoseInFrame, Pose

import connect



class Juggler:
    def __init__(self, robot):
        self.robot = robot
        self.arm = Arm.from_robot(self.robot, "arm")

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        motion_service = MotionClient.from_robot(self.robot, "builtin")
        home = PoseInFrame(reference_frame="world", pose=Pose(x=-734.3404649781442, y=0.7709977991432595, z=246.41067435410184, o_x=-0.9998925877347096, o_y=0, o_z=0, theta=174.8205202973959))

        
        # Define your speed constraints
        speed_config = MotionConfiguration(
            linear_m_per_sec=0.2,       # Sets the maximum linear velocity
            angular_degs_per_sec=90   # Sets the maximum turning/joint velocity
        )

        # Move the arm using the constraints
        await motion_service.move(
            component_name="arm",
            destination=home,
            motion_configuration=speed_config
        )

    async def high(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        motion_service = MotionClient.from_robot(self.robot, "builtin")
        home = PoseInFrame(reference_frame="world", pose=Pose(x=-620.8801241253041, y=13.065989629810097, z=564.3014151250942, o_x=-0.9994030742493492, o_y=0.016881295816552816, o_z=0.03014161628884865, theta=174.6887362033622))

        
        # Define your speed constraints
        speed_config = MotionConfiguration(
            linear_m_per_sec=0.2,       # Sets the maximum linear velocity
            angular_degs_per_sec=90   # Sets the maximum turning/joint velocity
        )

        # Move the arm using the constraints
        await motion_service.move(
            component_name="arm",
            destination=home,
            motion_configuration=speed_config
        )

    async def throw(self):
        """Throw the ball"""
        await self.arm.move_to_joint_positions(
            JointPositions(values=[5.0, -38.4, 4.3, 0.7, -64.5, 0.4])
        )




    


# verb -> method. One entry per capability.
STEPS = {
    "throw": Juggler.throw,
    "zero": Juggler.zero,
    "high": Juggler.high,
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
