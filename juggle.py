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

        self.top_throw = JointPositions(values=[41.48, -72.58, -181.52, -3.52, -17.50, 3.70])
        self.mid_throw = JointPositions(values=[41.48, -92.6, -211.52, -3.52, -10, 3.70])
        self.three_throw = JointPositions(values=[41.48, -97.6, -211.52, -3.52, -5, 3.70])
        self.two_throw = JointPositions(values=[41.48, -102.6, -211.52, -3.52, 0, 3.70])
        self.one_throw = JointPositions(values=[41.48, -107.6, -211.52, -3.52, 2.50, 3.70])
        self.zero_place = JointPositions(values=[41.48, -112.58, -211.52, -3.52, 2.50, 3.70])


    async def zero(self):
        await self.arm.move_to_joint_positions(self.zero_place)

    async def test(self):
        await self.arm.move_to_joint_positions(self.mid_throw)

    async def hand(self):
        await self.arm.move_to_joint_positions(JointPositions(values=[41.48, -112.58, -211.52, -3.52, 57.50, 3.70]))
        

    async def throw(self):
        await self.arm.move_through_joint_positions([self.one_throw, self.two_throw, self.three_throw, self.mid_throw, self.top_throw])

    async def full(self):
        sleep_time = 0.2
        await self.arm.move_through_joint_positions([self.one_throw, self.two_throw, self.three_throw, self.mid_throw, self.top_throw])
        await asyncio.sleep(sleep_time)
        await self.arm.move_to_joint_positions(self.zero_place)
        await asyncio.sleep(sleep_time)
        await self.arm.move_through_joint_positions([self.one_throw, self.two_throw, self.three_throw, self.mid_throw, self.top_throw])
        await asyncio.sleep(sleep_time)    
        await self.arm.move_to_joint_positions(self.zero_place)


    async def launch(self):
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
    "hand": Juggler.hand,
    "test": Juggler.test,
    "full": Juggler.full,
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
