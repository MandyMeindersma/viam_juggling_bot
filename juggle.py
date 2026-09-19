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

import time


class Juggler:
    def __init__(self, robot):
        self.robot = robot
        self.arm = Arm.from_robot(self.robot, "arm")
        self.motion = MotionClient.from_robot(self.robot, "builtin")

    async def zero(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        print((await self.arm.get_joint_positions()).values)


        #task_a = asyncio.create_task(self.log_joint_speeds())
        #task_b = asyncio.create_task(self.arm.move_to_joint_positions(JointPositions(values=[43.91, -105.07, -215.38, 5.55, 12.72, -11.52])))
        #await task_a
        #await task_b
        slow = MoveOptions(max_vel_degs_per_sec=10, max_acc_degs_per_sec2=1100)
        await self.arm.move_through_joint_positions([JointPositions(values=[41.48, -112.58, -211.52, -3.52, 2.50, -11.30])], options= slow)

        

    async def throw(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        print((await self.arm.get_joint_positions()).values)

        #task_a = asyncio.create_task(self.log_joint_speeds())
        task_b = asyncio.create_task(self.arm.move_to_joint_positions(JointPositions(values=[43.91, -95.07, -170.38, 5.55, 12.72, -11.52])))
        #await task_a
        await task_b


    async def launch(self):
        """Zero out all the joints, returning the arm to a flat, outstretched position."""
        print((await self.arm.get_joint_positions()).values)

        #task_a = asyncio.create_task(self.log_joint_speeds())
        task_b = asyncio.create_task(self.arm.move_to_joint_positions(JointPositions(values=[41.77, -46.93, -171.17, 5.55, 10.82, -11.52])))
        #await task_a
        await task_b

    async def log_joint_speeds(self, duration_sec: int = 2, sample_rate_hz: int = 20):
        delay = 1.0 / sample_rate_hz
        print("Starting joint speed logging...")
        
        # Get initial positions and timestamp
        prev_positions = (await self.arm.get_joint_positions()).values
        prev_time = time.time()
        
        start_time = prev_time
        while time.time() - start_time < duration_sec:
            await asyncio.sleep(delay)
            
            # Capture current state
            current_time = time.time()
            current_positions = (await self.arm.get_joint_positions()).values
            dt = current_time - prev_time
            
            if dt <= 0:
                continue
                
            # Calculate and log speed for each joint
            speeds = []
            for i, (curr_pos, prev_pos) in enumerate(zip(current_positions, prev_positions)):
                # Computes speed in degrees per second (or radians depending on your driver config)
                speed = abs(curr_pos - prev_pos) / dt
                speeds.append(f"Joint {i}: {speed:.2f} deg/s")
                
            print(f"[{current_time:.2f}] " + " | ".join(speeds))
            
            # Update references for next iteration
            prev_positions = current_positions
            prev_time = current_time




    


# verb -> method. One entry per capability.
STEPS = {
    "throw": Juggler.throw,
    "zero": Juggler.zero,
    "launch": Juggler.launch,
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
