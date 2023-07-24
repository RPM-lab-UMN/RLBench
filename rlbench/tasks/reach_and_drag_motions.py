from typing import List
from rlbench.backend.task import Task
from rlbench.const import colors
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.dummy import Dummy


class ReachAndDragMotions(Task):

    def init_task(self) -> None:
        self.stick = Shape('stick')
        self.register_graspable_objects([self.stick])
        self.cube = Shape('cube')
        self.target = Shape('target0')
        self.distractor1 = Shape('distractor1')
        self.distractor2 = Shape('distractor2')
        self.distractor3 = Shape('distractor3')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed=None) -> List[str]:
        color_name, color_rgb = colors[index]
        self.target.set_color(color_rgb)

        _, distractor1_rgb = colors[(index + 5) % len(colors)]
        self.distractor1.set_color(distractor1_rgb)

        _, distractor2_rgb = colors[(index + 6) % len(colors)]
        self.distractor2.set_color(distractor2_rgb)

        _, distractor3_rgb = colors[(index + 7) % len(colors)]
        self.distractor3.set_color(distractor3_rgb)

        return ['move above the stick']

    def variation_count(self) -> int:
        return len(colors)
