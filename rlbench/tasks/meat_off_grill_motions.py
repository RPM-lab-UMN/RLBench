from re import M
from typing import List
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.task import Task

MEAT = ['chicken', 'steak']


class MeatOffGrillMotions(Task):

    def init_task(self) -> None:
        self._steak = Shape('steak')
        self._chicken = Shape('chicken')
        self._success_sensor = ProximitySensor('success')
        self.register_graspable_objects([self._chicken, self._steak])
        self._w1 = Dummy('waypoint1')
        self._w1z= self._w1.get_position()[2]
        self.register_stop_at_waypoint(1)
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self._success_sensor)
        ])

    def init_episode(self, index: int, seed = None) -> List[str]:
        if index == 0:
            x, y, _ = self._chicken.get_position()
            self._w1.set_position([x, y, self._w1z])

        else:
            x, y, _ = self._steak.get_position()
            self._w1.set_position([x, y, self._w1z])

        text = ['0', '1', '2', '3', '4', '5']
        if MEAT[index] == 'chicken':
            text[0] = 'move above the chicken'
            text[1] = 'approach the chicken'
            text[2] = 'go over the chicken'
            text[3] = 'point the gripper at the chicken from above'
            text[4] = 'go above the chicken'
            text[5] = 'move above the drumstick'
        elif MEAT[index] == 'steak':
            text[0] = 'move above the steak'
            text[1] = 'approach the steak'
            text[2] = 'go over the steak'
            text[3] = 'point the gripper at the steak from above'
            text[4] = 'go above the steak'
            text[5] = 'move above the red meat'

        return text

    def variation_count(self) -> int:
        return 2
