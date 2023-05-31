from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy
from rlbench.const import colors
import numpy as np
import time

class MoveAbove(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        self.boundary = SpawnBoundary([Shape('plane')])
        # blocks
        self.block1 = Shape('block1')
        self.block2 = Shape('block2')
        # waypoints
        self.waypoint0 = Dummy('waypoint0')
        self.tip = Dummy('Panda_tip')
        # bowl
        self.register_success_conditions([
            DetectedCondition(self.tip, self.success_sensor)
        ])

    def init_episode(self, index: int) -> List[str]:
        # move robot to initial position
        j = np.array([1.90242633e-01, -1.82561681e-03, -1.74581066e-01, -2.33221745e+00, -1.09314790e-03,  2.26251936e+00,  8.01950991e-01])
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)

        # spawn objects in the workspace
        self.boundary.clear()
        for ob in [self.block1, self.block2]:
            self.boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # get the position of block1
        block1_pos = self.block1.get_position()

        # set the block colors
        text = []
        valid_idx = np.array([0, 2, 4])
        if index == 0:
            color = 0
            self.block1.set_color(colors[color][1])
            text.append('move above the red object')
            # move the sensor next to the block
        elif index == 1:
            color = 2
            self.block1.set_color(colors[color][1])
            text.append('move above the green object')
        elif index == 2:
            color = 4
            # push magenta block forward
            self.block1.set_color(colors[color][1])
            text.append('move above the blue object')

        # random valid color for distractor block
        color2 = np.random.choice(valid_idx)
        while color2 == color:
            color2 = np.random.choice(valid_idx)
        self.block2.set_color(colors[color2][1])

        return text

    def variation_count(self) -> int:
        return 3
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True