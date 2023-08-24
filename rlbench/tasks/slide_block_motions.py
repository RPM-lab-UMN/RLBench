from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
import numpy as np

class SlideBlockMotions(Task):

    def init_task(self) -> None:
        self.block = Shape('block')
        self.directions = ['in front of', 'behind', 'left of', 'right of']
        self.sensors = [ProximitySensor('sensor0'), ProximitySensor('sensor1'),
                        ProximitySensor('sensor2'), ProximitySensor('sensor3')]
        self.tip = Dummy('Panda_tip')
        self.waypoint0 = Dummy('waypoint0')

    def init_episode(self, index: int, seed = None) -> List[str]:
        direction = self.directions[index]
        # select the correct success sensor
        positions = []
        for i in range(4):
            positions.append(self.sensors[i].get_position())
        if direction == 'in front of':
            # register sensor with smallest x value
            self.sensor = self.sensors[np.argmin([pos[0] for pos in positions])]
            self.register_success_conditions([DetectedCondition(self.tip, self.sensor)])
            # put waypoint 0 at that sensor
            self.waypoint0.set_position(self.sensor.get_position())
        elif direction == 'behind':
            # register sensor with largest x value
            self.sensor = self.sensors[np.argmax([pos[0] for pos in positions])]
            self.register_success_conditions([DetectedCondition(self.tip, self.sensor)])
        elif direction == 'left of':
            # register sensor with largest y value
            self.sensor = self.sensors[np.argmax([pos[1] for pos in positions])]
            self.register_success_conditions([DetectedCondition(self.tip, self.sensor)])
        elif direction == 'right of':
            # register sensor with smallest y value
            self.sensor = self.sensors[np.argmin([pos[1] for pos in positions])]
            self.register_success_conditions([DetectedCondition(self.tip, self.sensor)])

        # move waypoint 0 to the correct pose
        self.waypoint0.set_pose(self.sensor.get_pose())

        # close the gripper
        while self.robot.gripper.get_open_amount()[0] > 0.01:
            self.robot.gripper.actuate(0, velocity=0.1)
            self.pyrep.step()

        text = ['0', '1', '2', '3', '4', '5']
        if direction == 'in front of':
            text[0] = 'move in front of the block'
            text[1] = 'go to the front side of the cube'
            text[2] = 'approach the block from the front'
            text[3] = 'move in front of the cube'
            text[4] = 'approach the front of the block'
            text[5] = 'move near the front of the block'
        elif direction == 'behind':
            text[0] = 'move behind the block'
            text[1] = 'go to the back side of the cube'
            text[2] = 'approach the block from the back'
            text[3] = 'move behind the cube'
            text[4] = 'approach the back of the block'
            text[5] = 'move near the back of the block'
        elif direction == 'left of':
            text[0] = 'move left of the block'
            text[1] = 'go to the left side of the cube'
            text[2] = 'approach the block from the left'
            text[3] = 'move to the left of the cube'
            text[4] = 'approach the left side of the block'
            text[5] = 'move near the left side of the block'
        elif direction == 'right of':
            text[0] = 'move right of the block'
            text[1] = 'go to the right side of the cube'
            text[2] = 'approach the block from the right'
            text[3] = 'move to the right of the cube'
            text[4] = 'approach the right side of the block'
            text[5] = 'move near the right side of the block'

        return text

    def variation_count(self) -> int:
        return 4
