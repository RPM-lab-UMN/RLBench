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
from pyrep.objects.joint import Joint

class MoveFront(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        # waypoints
        try:
            self.end_point = Dummy('waypoint0')
        except:
            self.end_point = Dummy('waypoint1')
        self.approach_point = Dummy.create()
        self.tip = Dummy('Panda_tip')
        self.cup = Shape('cup')
        self.apple = Shape('apple')
        self.register_graspable_objects([self.cup, self.apple])
        self.register_success_conditions([
            DetectedCondition(self.tip, self.success_sensor)
        ])
        self.plane0 = Shape('plane0')
        self.boundary0 = SpawnBoundary([self.plane0])
        self.plane1 = Shape('plane1')
        self.boundary1 = SpawnBoundary([self.plane1])

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
        target_orientation = self.cup.get_orientation()
        # randomly select on of the boundaries
        boundary = self.boundary0
        # spawn objects in the workspace
        boundary.clear()
        for ob in [self.cup]:
            boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # step the simulation
        for _ in range(3):
            self.pyrep.step()
        # set target orientation
        self.cup.set_orientation(target_orientation)
        # get target position
        cup_position = self.cup.get_position()

        # if index is 0, move success in front of the cup
        text = []
        if index == 0:
            # rename approach point
            self.approach_point.set_name('not_used')
            # name end point
            self.end_point.set_name('waypoint0')
            self.success_sensor.set_position([cup_position[0]-0.05, cup_position[1], cup_position[2]+0.025])
            text.append('move in front of the blue cup')
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
        # if index is 1, move success in front of dispenser
        elif index == 1:
            # rename approach point
            self.approach_point.set_name('not_used')
            # name end point
            self.end_point.set_name('waypoint0')
            # set fridge door closed
            Joint('fridge_joint').set_joint_position(0, disable_dynamics=True)
            self.success_sensor.set_position([.17, .34, 1.27])
            text.append('move in front of the dispenser')
            # always close gripper at dispenser
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
        # if index is 2, move success in front of apple in fridge
        elif index == 2:
            # open the gripper
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            # end_point is waypoint 1
            self.end_point.set_name('waypoint1')
            # approach is waypoint 0
            self.approach_point.set_name('waypoint0')
            # set fridge door open
            Joint('fridge_joint').set_joint_position(0.25, disable_dynamics=True)
            # spawn apple in fridge
            self.boundary1.clear()
            self.boundary1.sample(self.apple, ignore_collisions=False, min_distance=0.16,
                                    min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
            # set apple z to boundary z
            apple_pos = self.apple.get_position()
            boundary_z = self.plane1.get_position()[2]
            self.apple.set_position([apple_pos[0], apple_pos[1], boundary_z])
            # step the simulation
            for _ in range(3):
                self.pyrep.step()
            # set success sensor in front of apple
            apple_pos = self.apple.get_position()
            self.success_sensor.set_position([apple_pos[0]-0.05, apple_pos[1], apple_pos[2]+0.025])
            # set waypoint 0 orientation to waypoint 1 orientation
            self.approach_point.set_orientation(self.end_point.get_orientation())
            # set waypoint 0 x clear of the fridge
            self.approach_point.set_position([0.1, apple_pos[1], apple_pos[2]])
            text.append('move in front of the apple')
        # if index is 3, move success in front of the fridge handle
        elif index == 3:
            # rename approach point
            self.approach_point.set_name('not_used')
            # name end point
            self.end_point.set_name('waypoint0')
            # open the gripper
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            # close fridge
            Joint('fridge_joint').set_joint_position(0, disable_dynamics=True)
            # set success sensor in front of fridge handle
            self.success_sensor.set_position([0.12, 0.249, 1.324])
            text.append('move in front of the fridge handle')

        # if interactive move the success sensor out of the way
        if interactive:
            self.success_sensor.set_position([.17, .335, 1.0])

        return text

    def variation_count(self) -> int:
        # 1 move in front of cup
        # 2 move in front of dispenser (fridge door closed)
        # 3 move in front of apple in fridge
        # 4 move in front of the fridge handle
        return 4
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True