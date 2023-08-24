from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task


class PutItemInDrawerMotions(Task):

    def init_task(self) -> None:
        self._item = Shape('item')
        self.register_graspable_objects([self._item])
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index, seed = None) -> List[str]:
        text = ['0', '1', '2', '3', '4', '5']
        text[0] = 'move above the block'
        text[1] = 'approach the block'
        text[2] = 'go over the block'
        text[3] = 'point the gripper at the block from above'
        text[4] = 'go above the block'
        text[5] = 'move above the cube'
        
        return text

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, - np.pi / 8], [0, 0, np.pi / 8]
