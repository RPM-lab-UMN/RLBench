from typing import List, Tuple
import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition

class OpenDrawerMotions(Task):

    def init_task(self) -> None:
        self._options = ['bottom', 'middle', 'top']
        self._anchors = [Dummy('waypoint_anchor_%s' % opt)
                         for opt in self._options]
        self._joints = [Joint('drawer_joint_%s' % opt)
                        for opt in self._options]
        self._waypoint0 = Dummy('waypoint0')
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])
        
    def init_episode(self, index: int, seed = None) -> List[str]:
        option = self._options[index]
        self._waypoint0.set_position(self._anchors[index].get_position())
        # language augmentation
        text = ['0', '1', '2', '3', '4', '5']
        if option == 'bottom':
            text[0] = 'move in front of the bottom handle'
            text[1] = 'approach the lowest drawer'
            text[2] = 'go near the lower handle'
            text[3] = 'point the gripper at the bottom handle'
            # unseen combo
            text[4] = 'move in front of the lowest handle'
            # unseen word
            text[5] = 'move in front of the bottommost handle'
        elif option == 'middle':
            text[0] = 'move in front of the middle handle'
            text[1] = 'approach the center drawer'
            text[2] = 'go near the middle handle'
            text[3] = 'point the gripper at the middle handle'
            text[4] = 'move in front of the center handle'
            text[5] = 'move in front of the handle second from the top'
        elif option == 'top':
            text[0] = 'move in front of the top handle'
            text[1] = 'approach the highest drawer'
            text[2] = 'go near the upper handle'
            text[3] = 'point the gripper at the top handle'
            text[4] = 'move in front of the highest handle'
            text[5] = 'move in front of the topmost handle'
        return text

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, - np.pi / 8], [0, 0, np.pi / 8]
