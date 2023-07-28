from typing import List
from rlbench.backend.task import Task
from rlbench.const import colors
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.dummy import Dummy


class LightBulbInMotions(Task):

    def init_task(self) -> None:
        self.bulbs_visual = [Shape('light_bulb%d' % i) for i in range(2)]
        self.bulb_glass_visual = [Shape('bulb%d' % i) for i in range(2)]
        self.holders = [Shape('bulb_holder%d' % i) for i in range(2)]
        self.bulbs = [Shape('bulb_phys%d' % i) for i in range(2)]
        self.conditions = [NothingGrasped(self.robot.gripper)]
        self.register_graspable_objects(self.bulbs)
        self.boundary = Shape('spawn_boundary')
        self.w0 = Dummy('waypoint0')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed=None) -> List[str]:
        if index >= len(colors):
            lamp = True
            text = 'move above the lamp'
            # gripper should be closed above the lamp
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            index = index - len(colors)
        else:
            lamp = False
            # gripper should be open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()

        self._variation_index = index
        b = SpawnBoundary([self.boundary])
        for holder in self.holders:
            b.sample(holder, min_distance=0.01)
        self.w1 = Dummy('waypoint1')
        self.w1.set_position([0, 0, +10*10**(-3)],
                             relative_to=self.bulb_glass_visual[index % 2],
                             reset_dynamics=False)
        target_color_name, target_color_rgb = colors[index]
        color_choice = np.random.choice(
            list(range(index)) + list(
                range(index + 1, len(colors))),
            size=1, replace=False)[0]
        _, distractor_color_rgb = colors[color_choice]
        self.holders[index % 2].set_color(target_color_rgb)
        other_index = {0: 1, 1: 0}
        self.holders[other_index[index % 2]].set_color(distractor_color_rgb)

        if lamp:
            # set waypoint0 above the lamp
            base = Shape('lamp_base')
            target_pos = base.get_position()
            self.w0.set_position([target_pos[0], target_pos[1], target_pos[2] + 0.4])
        else:
            text = 'move above the %s bulb' % target_color_name
            # set waypoint0 above the target bulb
            target_pos = self.bulb_glass_visual[index % 2].get_position()
            self.w0.set_position([target_pos[0], target_pos[1], target_pos[2] + 0.1])

        return [text]

    def variation_count(self) -> int:
        return len(colors) * 2 # move above the lamp for each color

    def step(self) -> None:
        if DetectedCondition(self.bulbs[self._variation_index % 2],
                                  ProximitySensor('sensor')).condition_met() \
                == (True, True):
            self.bulb_glass_visual[self._variation_index % 2].set_color(
                [1.0, 1.0, 0.0])

    def cleanup(self) -> None:
        if self.bulb_glass_visual:
            [self.bulb_glass_visual[i].set_color([1.0, 1.0, 1.0])
             for i in range(2)]