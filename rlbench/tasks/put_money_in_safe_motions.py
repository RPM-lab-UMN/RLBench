from tkinter import N
from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

NUM_SHELVES_IN_SAFE = 3


class PutMoneyInSafeMotions(Task):

    def init_task(self) -> None:
        self.index_dic = {0: 'bottom', 1: 'middle', 2: 'top'}
        self.money = Shape('dollar_stack')
        self.money_boundary = Shape('dollar_stack_boundary')
        self.register_graspable_objects([self.money])

        self.w1_rel_pos = [-2.7287 * 10 ** (-4), -2.3246 * 10 ** (-6),
                           +4.5627 * 10 ** (-2)]
        self.w1_rel_ori = [-3.1416, 7.2824 * 10 ** (-1), -2.1265 * 10 ** (-2)]

        self.w0 = Dummy('waypoint0')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])
        self.w_money = Dummy('waypoint_money')
        self.w_shelf = Dummy('waypoint_shelf')

    def init_episode(self, index: int, seed=None) -> List[str]:
        self.target_shelf = index % NUM_SHELVES_IN_SAFE
        w4 = Dummy('waypoint4')
        target_dummy_name = 'dummy_shelf' + str(self.target_shelf)
        target_pos_dummy = Dummy(target_dummy_name)
        target_pos = target_pos_dummy.get_position()
        w4.set_position(target_pos, reset_dynamics=False)

        b = SpawnBoundary([self.money_boundary])
        b.sample(self.money,
                 min_rotation=(0.00, 0.00, 0.00),
                 max_rotation=(0.00, 0.00, +0.5 * np.pi))

        if index == 0:
            text = 'move in front of the money'
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            # set waypoint 0 pose to money pose
            self.w0.set_pose(self.w_money.get_pose())
        else:
            text = 'move in front of the %s shelf' % self.index_dic[self.target_shelf]
            # gipper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            # set waypoint 0 pose to shelf pose
            self.w0.set_pose(self.w_shelf.get_pose())

        return [text]

    def variation_count(self) -> int:
        # 1 move in front of the money
        # 2, 3, 4 move in front of the {top, middle, bottom} shelf
        return 4

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, +0.5 * np.pi]
