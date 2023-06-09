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

class MoveAbove(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        self.boundary = SpawnBoundary([Shape('plane')])
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
        self.target = None
        self.distractor = None

    def init_episode(self, index: int, seed = None) -> List[str]:
        # move robot to initial position
        j = np.array([1.90242633e-01, -1.82561681e-03, -1.74581066e-01, -2.33221745e+00, -1.09314790e-03,  2.26251936e+00,  8.01950991e-01])
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)

        # if target or distractor exist, remove them
        try:
            target = Shape('target')
            target.remove()
            distractor = Shape('distractor')
            distractor.remove()
        except:
            pass

        # create a target with random size
        sx = np.random.uniform(0.05, 0.1)
        sy = np.random.uniform(0.05, 0.1)
        sz = np.random.uniform(0.05, 0.1)
        # type is a random choice from PrimitiveShape
        type = np.random.choice(list(PrimitiveShape))
        if type == PrimitiveShape.SPHERE:
            size = [sz]
        else:
            size = [sx, sy, sz]
        target_height = sz
        self.target = Shape.create(type=type, size=size, position=[0, 0, 0.8], color=[1, 0, 0])
        # set the name of the object
        self.target.set_name('target')

        # create distractor object
        sx = np.random.uniform(0.05, 0.1)
        sy = np.random.uniform(0.05, 0.1)
        sz = np.random.uniform(0.05, 0.1)
        # type is a random choice from PrimitiveShape
        type = np.random.choice(list(PrimitiveShape))
        if type == PrimitiveShape.SPHERE:
            size = [sz]
        else:
            size = [sx, sy, sz]
        self.distractor = Shape.create(type=type, size=size, position=[0, 0.15, 0.8], color=[0, 1, 0])
        self.distractor.set_name('distractor')

        # spawn objects in the workspace
        self.boundary.clear()
        for ob in [self.target, self.distractor]:
            self.boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # step the simulation
        for _ in range(10):
            self.pyrep.step()
        # get the position of target
        target_pos = self.target.get_position()

        # set the block colors
        text = []
        valid_idx = np.array([0, 2, 4])
        if index == 0:
            color = 0
            self.target.set_color(colors[color][1])
            text.append('move above the red object')
        elif index == 1:
            color = 2
            self.target.set_color(colors[color][1])
            text.append('move above the green object')
        elif index == 2:
            color = 4
            self.target.set_color(colors[color][1])
            text.append('move above the blue object')

        # move the sensor above the target
        self.waypoint0.set_position([target_pos[0], target_pos[1], target_pos[2] + target_height/2 + 0.025])
        self.success_sensor.set_position([target_pos[0], target_pos[1], target_pos[2] + target_height/2])

        # random valid color for distractor block
        color2 = np.random.choice(valid_idx)
        while color2 == color:
            color2 = np.random.choice(valid_idx)
        self.distractor.set_color(colors[color2][1])

        return text

    def variation_count(self) -> int:
        return 3
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True