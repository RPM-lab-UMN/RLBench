from typing import List, Tuple
import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition, NothingGrasped, \
    OrConditions
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class PlaceCupsMotions(Task):

    def init_task(self) -> None:
        self._cups = [Shape('mug%d' % i) for i in range(3)]
        self._spokes = [Shape('place_cups_holder_spoke%d' % i) for i in
                        range(3)]
        self._cups_boundary = Shape('mug_boundary')
        self._w1 = Dummy('waypoint1')
        self._w5 = Dummy('waypoint5')
        success_detectors = [
            ProximitySensor('success_detector%d' % i) for i in range(3)]
        self._on_peg_conditions = [OrConditions([
            DetectedCondition(self._cups[ci], success_detectors[sdi]) for sdi in
            range(3)]) for ci in range(3)]
        self.register_graspable_objects(self._cups)
        self._initial_relative_cup = self._w1.get_pose(self._cups[0])
        self._initial_relative_spoke = self._w5.get_pose(self._spokes[0])
        self.w0 = Dummy('waypoint0')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor),
        ])

    def init_episode(self, index: int, seed=None) -> List[str]:
        self._cups_placed = 0
        self._index = index
        b = SpawnBoundary([self._cups_boundary])
        [b.sample(c, min_distance=0.10) for c in self._cups]

        if index == 0:
            # get max mug y position index
            mug_y_positions = [self._cups[i].get_position()[1] for i in range(3)]
            max_mug_y_position_index = np.argmax(mug_y_positions)
            # set waypoint 1 parent
            self._w1.set_parent(self._cups[max_mug_y_position_index])
            self._w1.set_pose(
                self._initial_relative_cup,
                relative_to=self._cups[max_mug_y_position_index])
            # move waypoint 0 to waypoint mug
            w_mug = Dummy('waypoint_mug')
            self.w0.set_pose(w_mug.get_pose())
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            return ['move above the left mug']
        else:
            # move waypoint 0 to waypoint3
            w3 = Dummy('waypoint3')
            self.w0.set_pose(w3.get_pose())
            # gripper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            return ['move in front of the rack']

    def variation_count(self) -> int:
        # 1 move above the left mug
        # 2 move in front of the rack
        return 2

    def _move_above_next_target(self, waypoint):
        self._w1.set_parent(self._cups[self._cups_placed])
        self._w5.set_pose(
            self._initial_relative_spoke,
            relative_to=self._spokes[self._cups_placed])
        self._w1.set_pose(
            self._initial_relative_cup,
            relative_to=self._cups[self._cups_placed])
        self._cups_placed += 1

    def _repeat(self):
        return self._cups_placed < self._index + 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -np.pi / 2], [0.0, 0.0, np.pi / 2]
