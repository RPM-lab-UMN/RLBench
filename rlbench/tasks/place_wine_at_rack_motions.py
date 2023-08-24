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
        
        text = ['0', '1', '2', '3', '4', '5']
        if index == 3:
            text[0] = 'move in front of the wine bottle'
            text[1] = 'approach the bottle'
            text[2] = 'go in front of the wine'
            text[3] = 'point the gripper at the wine bottle from the front'
            text[4] = 'go in front of the wine bottle'
            text[5] = 'move in front of the neck of the bottle'
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            # set waypoint 0 pose to wine pose
            self.w0.set_pose(self.w_wine.get_pose())
        else:
            if location == 'middle':
                text[0] = 'move in front of the middle of the rack'
                text[1] = 'approach the center of the wine rack'
                text[2] = 'go in front of the middle of the rack'
                text[3] = 'point the bottle at the center of the wine rack'
                text[4] = 'go in front of the center of the rack'
                text[5] = 'align the bottle to the middle of the rack'
            elif location == 'far side':
                text[0] = 'move in front of the far side of the rack'
                text[1] = 'approach the farthest slot of the wine rack'
                text[2] = 'go in front of the far side of the rack'
                text[3] = 'point the bottle at the far slot of the wine rack'
                text[4] = 'go in front of the farthest slot of the rack'
                text[5] = 'align the bottle to the far side of the rack'
            elif location == 'near side':
                text[0] = 'move in front of the near side of the rack'
                text[1] = 'approach the closest slot of the wine rack'
                text[2] = 'go in front of the near side of the rack'
                text[3] = 'point the bottle at the near slot of the wine rack'
                text[4] = 'go in front of the closest slot of the rack'
                text[5] = 'align the bottle to the near side of the rack'
            # gripper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            # set waypoint 3 near middle far
            self._move_to_rack()
            # set waypoint 0 to waypoint 3 pose
            self.w0.set_pose(Dummy('waypoint3').get_pose())

        return text
    
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
