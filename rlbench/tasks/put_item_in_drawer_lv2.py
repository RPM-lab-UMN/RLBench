from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task


class PutItemInDrawerLv2(Task):

    def init_task(self) -> None:
        self._options = ['bottom', 'middle', 'top']
        self._anchors = [Dummy('waypoint_anchor_%s' % opt)
                         for opt in self._options]
        self._joints = [Joint('drawer_joint_%s' % opt)
                        for opt in self._options]
        self._waypoint_handle = Dummy('waypoint_handle')
        self._item = Shape('item')
        self.register_graspable_objects([self._item])

    def init_episode(self, index, seed = None) -> List[str]:
        anchor = self._anchors[index]
        self._waypoint_handle.set_position(anchor.get_position())
        # open the proper drawer
        for joint in self._joints:
            joint.set_joint_position(0.0)
        amount = np.random.uniform(0.15, 0.3)
        self._joints[index].set_joint_position(amount)

        option = self._options[index]
        success_sensor = ProximitySensor('success_' + option)
        self.register_success_conditions(
            [DetectedCondition(self._item, success_sensor)])
        return ['put the block in the %s drawer' % option,
                'put the block away in the %s drawer' % option,
                'open the %s drawer and place the block inside of it' % option,
                'leave the block in the %s drawer' % option]

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, - np.pi / 8], [0, 0, np.pi / 8]
