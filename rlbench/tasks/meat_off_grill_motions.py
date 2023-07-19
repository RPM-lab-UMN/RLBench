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

    def init_episode(self, index: int, seed = None, interactive=False) -> List[str]:
        if index == 0:
            x, y, _ = self._chicken.get_position()
            self._w1.set_position([x, y, self._w1z])

        else:
            x, y, _ = self._steak.get_position()
            self._w1.set_position([x, y, self._w1z])

        return ['move above the %s' % MEAT[index]]

    def variation_count(self) -> int:
        return 2
