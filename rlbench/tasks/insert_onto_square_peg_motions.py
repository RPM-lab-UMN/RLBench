from typing import List
import numpy as np
from pyrep.objects import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition, ConditionSet
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from rlbench.const import colors


class InsertOntoSquarePegMotions(Task):

    def init_task(self) -> None:
        self._square_ring = Shape('square_ring')
        self._success_centre = Dummy('success_centre')
        success_detectors = [ProximitySensor(
            'success_detector%d' % i) for i in range(4)]
        self.register_graspable_objects([self._square_ring])

        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])
        self.w0 = Dummy('waypoint0')

    def init_episode(self, index: int, seed=None) -> List[str]:
        if index == 0:
            text = 'move above the square ring'
            ring = True
        else:
            index -= 1
            ring = False

        color_name, color_rgb = colors[index]
        spokes = [Shape('pillar0'), Shape('pillar1'), Shape('pillar2')]
        chosen_pillar = np.random.choice(spokes)
        chosen_pillar.set_color(color_rgb)
        _, _, z = self._success_centre.get_position()
        x, y, _ = chosen_pillar.get_position()
        self._success_centre.set_position([x, y, z])

        color_choices = np.random.choice(
            list(range(index)) + list(range(index + 1, len(colors))),
            size=2, replace=False)
        spokes.remove(chosen_pillar)
        for spoke, i in zip(spokes, color_choices):
            name, rgb = colors[i]
            spoke.set_color(rgb)
        b = SpawnBoundary([Shape('boundary0')])
        b.sample(self._square_ring)

        if ring:
            # move the sensor above the ring
            w_ring = Dummy('waypoint_ring')
            # set waypoint 0 to waypoint_ring pose
            self.w0.set_pose(w_ring.get_pose())
        else:
            text = 'move above the %s peg' % color_name
            # move the sensor above the peg
            w_peg = Dummy('waypoint3')
            # set waypoint 0 to waypoint_peg pose
            self.w0.set_pose(w_peg.get_pose())

        return [text]

    def variation_count(self) -> int:
        return len(colors) + 1 # +1 for the ring
