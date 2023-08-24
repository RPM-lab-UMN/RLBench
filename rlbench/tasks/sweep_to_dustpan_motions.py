from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from pyrep.objects.object import Object
from pyrep.objects.dummy import Dummy
from rlbench.backend.conditions import DetectedCondition

DIRT_NUM = 5


class SweepToDustpanMotions(Task):

    def init_task(self) -> None:
        self._dustpan_sizes = ['tall', 'short']

        broom = Shape('broom')
        self.register_graspable_objects([broom])

        self._waypoint_paths = {
            0: [Dummy('point1a'),
                Dummy('point1b'),
                Dummy('point1c')],

            1: [Dummy('point2a'),
                Dummy('point2b'),
                Dummy('point2c')]
        }
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed = None) -> List[str]:
        self.register_stop_at_waypoint(1)

        text = ['0', '1', '2', '3', '4', '5']
        text[0] = 'move in front of the broom'
        text[1] = 'approach the broom handle'
        text[2] = 'go near the broom'
        text[3] = 'point the gripper at the broom'
        text[4] = 'move in front of the broom handle'
        text[5] = 'move next to the broom handle'

        return text

    def variation_count(self) -> int:
        return 1

    # def boundary_root(self) -> Object:
    #     return Shape('boundary_root')
