from typing import List, Tuple

from pyrender import Primitive
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, GraspedCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy
from rlbench.const import colors
import numpy as np
from pyrep.const import PrimitiveShape
import time

class PickUp(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        # waypoints
        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.tip = Dummy('Panda_tip')
        self.target = Shape('cup')
        self.register_graspable_objects([self.target])
        self.register_success_conditions([
            DetectedCondition(self.tip, self.success_sensor), GraspedCondition(self.robot.gripper, self.target), 
        ])
        self.plane0 = Shape('plane0')
        self.plane1 = Shape('plane1')
        self.planes = [self.plane0, self.plane1]
        self.boundary0 = SpawnBoundary([self.plane0])
        self.boundary1 = SpawnBoundary([self.plane1])
        self.boundaries = [self.boundary0, self.boundary1]

    def init_episode(self, index: int, seed = None, interactive=False) -> List[str]:
        # move robot to initial position
        # j = self.robot.arm.solve_ik_via_sampling([0.25, 0, 1.0], [np.pi, 0, np.pi])[0]
        # 0.25, 0, 1.0
        # j = np.array([1.90242633e-01, -1.82561681e-03, -1.74581066e-01, -2.33221745e+00, -1.09314790e-03,  2.26251936e+00,  8.01950991e-01])
        # 0.1, 0, 1.5 np.pi, np.pi/2, np.pi
        j = np.array([ 0.51543331, -1.09951103, -0.3360858 , -2.50569105, -1.29321265, 2.7861464 ,  1.74406898])
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)
        
        # set random seed
        if seed is not None:
            np.random.seed(seed)
        else:
            print('no seed!')

        # get target orientation
        target_orientation = self.target.get_orientation()
        # randomly select on of the boundaries
        i = np.random.randint(0, len(self.boundaries))
        boundary = self.boundaries[i]
        # set target Z to boundary Z
        target_position = self.target.get_position()
        target_position[2] = self.planes[i].get_position()[2]
        self.target.set_position(target_position)
        # spawn objects in the workspace
        boundary.clear()
        for ob in [self.target]:
            boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # step the simulation
        for _ in range(3):
            self.pyrep.step()
        # set target orientation
        self.target.set_orientation(target_orientation)
        # get target position
        target_position = self.target.get_position()

        if index == 0:
            # move waypoint1 behind the cup
            waypoint1_position = self.waypoint0.get_position()
            waypoint1_position[0] = waypoint1_position[0] + 0.1
            self.waypoint1.set_position(waypoint1_position)
            # gripper should be open at the cup
            self.robot.gripper.actuate(1, velocity=0.1)
            for _ in range(3):
                self.pyrep.step()

        return ['pick up the blue cup']

    def variation_count(self) -> int:
        return 1
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True