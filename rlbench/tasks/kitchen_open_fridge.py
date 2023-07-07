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
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition

class KitchenOpenFridge(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        # waypoints
        self.waypoint0 = Dummy('waypoint0')
        self.tip = Dummy('Panda_tip')
        self.target = Shape('pot')
        self.register_graspable_objects([self.target])
        self.register_success_conditions([JointCondition(Joint('fridge_joint'), 0.2)])
        self.plane0 = Shape('plane0')
        self.plane1 = Shape('plane1')
        self.planes = [self.plane0]
        self.boundary0 = SpawnBoundary([self.plane0])
        self.boundary1 = SpawnBoundary([self.plane1])
        self.boundaries = [self.boundary0]

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

        # spawn pot between left and right burner
        self.boundary1.clear()
        self.boundary1.sample(self.target, ignore_collisions=True, min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        # step the simulation
        for _ in range(3):
            self.pyrep.step()

        if index == 0:
            # gripper open
            self.robot.gripper.actuate(1, velocity=0.1)
            text = ['move above the left edge of the pot']
        elif index == 1:
            # step the simulation until the gripper is closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            text = ['move above the left edge of the left burner']
            # move waypoint0
            self.waypoint0.set_position([0.28, -0.148, 1.416])
        elif index == 2:
            # step the simulation until the gripper is closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            text = ['move above the left edge of the right burner']
            # move waypoint0
            self.waypoint0.set_position([0.28, -0.292, 1.416])

        # set fridge joint to 0
        Joint('fridge_joint').set_joint_position(0, disable_dynamics=True)
        for _ in range(3):
            self.pyrep.step()

        return text

    def variation_count(self) -> int:
        # 1 left edge of pot
        # 2 left edge of left burner
        # 3 left edge of right burner
        return 3
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True