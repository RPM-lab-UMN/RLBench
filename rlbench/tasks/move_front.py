from typing import List, Tuple

from pyrender import Primitive
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy
from rlbench.const import colors
import numpy as np
from pyrep.const import PrimitiveShape
import time

class MoveFront(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        self.plane = Shape('plane')
        # blocks
        # self.block1 = Shape('block1')
        # self.block2 = Shape('block2')
        # waypoints
        self.waypoint0 = Dummy('waypoint0')
        self.tip = Dummy('Panda_tip')
        # bowl
        self.register_success_conditions([
            DetectedCondition(self.tip, self.success_sensor)
        ])
        self.target = Shape('cup')
        self.boundary = SpawnBoundary([self.plane])

    def init_episode(self, index: int, seed = None) -> List[str]:
        # move robot to initial position
        # j = self.robot.arm.solve_ik_via_sampling([0.25, 0, 1.0], [np.pi, 0, np.pi])[0]
        # 0.25, 0, 1.0
        # j = np.array([1.90242633e-01, -1.82561681e-03, -1.74581066e-01, -2.33221745e+00, -1.09314790e-03,  2.26251936e+00,  8.01950991e-01])
        # 0.25, 0, 1.2
        j = np.array([-0.08565323, -1.03495145,  0.12679109, -2.62085414,  0.39900205, 3.08864594,  0.49581817])
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)
        
        # set random seed
        if seed is not None:
            np.random.seed(seed)
        else:
            print('no seed!')

        # get target orientation
        target_orientation = self.target.get_orientation()
        # spawn objects in the workspace
        self.boundary.clear()
        for ob in [self.target]:
            self.boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # step the simulation
        for _ in range(3):
            self.pyrep.step()
        # set target orientation
        self.target.set_orientation(target_orientation)

        # if index is 0, move success in front of the cup
        text = []
        if index == 0:
            self.success_sensor.set_position([-0.05, 0, 0], relative_to=self.target)
            text.append('move in front of the blue cup')
        # if index is 1, move success in front of dispenser
        elif index == 1:
            self.success_sensor.set_position([.23, .335, 1.235])
            text.append('move in front of the dispenser')

        return text

    def variation_count(self) -> int:
        return 2
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True