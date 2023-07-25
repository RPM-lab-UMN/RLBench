from typing import List, Tuple

import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from pyrep.objects.dummy import Dummy
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PlaceWineAtRackMotions(Task):

    def init_task(self):
        self.wine_bottle = Shape('wine_bottle')
        self.register_graspable_objects([self.wine_bottle])

        self.locations = ['middle', 'far side', 'near side']

        self.register_waypoint_ability_start(5, self._is_last)
        self.register_waypoint_ability_start(6, self._is_last)
        self.register_waypoint_ability_start(7, self._is_last)
        self.register_waypoint_ability_start(8, self._is_last)

        self.w0 = Dummy('waypoint0')
        self.w_wine = Dummy('waypoint_wine')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed=None) -> List[str]:
        self._variation_index = index
        location = self.locations[self._variation_index % 3]
        
        if index == 3:
            text = 'move in front of the wine bottle'
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            # set waypoint 0 pose to wine pose
            self.w0.set_pose(self.w_wine.get_pose())
        else:
            text = 'move in front of the %s of the rack' % location
            # gripper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            # set waypoint 3 near middle far
            self._move_to_rack()
            # set waypoint 0 to waypoint 3 pose
            self.w0.set_pose(Dummy('waypoint3').get_pose())

        return [text]
    
    def _move_to_rack(self):
        next1, next2 = Dummy('waypoint3'), Dummy('waypoint4')
        left1, left2 = Dummy('waypoint5'), Dummy('waypoint6')
        right1, right2 = Dummy('waypoint7'), Dummy('waypoint8')
        
        if self._variation_index == 1:
            next1.set_position(left1.get_position())
            next1.set_orientation(left1.get_orientation())

            next2.set_position(left2.get_position())
            next2.set_orientation(left2.get_orientation())
        elif self._variation_index == 2:
            next1.set_position(right1.get_position())
            next1.set_orientation(right1.get_orientation())

            next2.set_position(right2.get_position())
            next2.set_orientation(right2.get_orientation())


    def _is_last(self, waypoint):
        waypoint.skip = True

    def variation_count(self) -> int:
        # 1 move in front of bottle
        # 2, 3, 4 move in front of {left side, right side, middle} of rack
        return 4

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 4.], [0, 0, np.pi / 4.]
