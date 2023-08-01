from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary
from pyrep.objects.dummy import Dummy


class StackCupsMotions(Task):

    def init_task(self) -> None:
        self.cup1 = Shape('cup1')
        self.cup2 = Shape('cup2')
        self.cup3 = Shape('cup3')
        self.cup1_visual = Shape('cup1_visual')
        self.cup2_visual = Shape('cup2_visual')
        self.cup3_visaul = Shape('cup3_visual')

        self.boundary = SpawnBoundary([Shape('boundary')])

        self.register_graspable_objects([self.cup1, self.cup2, self.cup3])
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])
        self.w0 = Dummy('waypoint0')

    def init_episode(self, index: int, seed=None) -> List[str]:
        if index >= len(colors):
            alot = True
            index -= len(colors)
        else:
            alot = False

        self.variation_index = index
        target_color_name, target_rgb = colors[index]

        random_idx1 = np.random.choice(len(colors))
        while random_idx1 == index:
            random_idx1 = np.random.choice(len(colors))
        first_color, other1_rgb = colors[random_idx1]

        random_idx2 = np.random.choice(len(colors))
        while random_idx2 == index or random_idx2 == random_idx1:
            random_idx2 = np.random.choice(len(colors))
        _, other2_rgb = colors[random_idx2]

        self.cup1_visual.set_color(target_rgb)
        self.cup2_visual.set_color(other1_rgb)
        self.cup3_visaul.set_color(other2_rgb)

        self.boundary.clear()
        self.boundary.sample(self.cup2, min_distance=0.05,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.cup1, min_distance=0.05,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.cup3, min_distance=0.05,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        if alot:
            text = 'move a lot above the left edge of the %s cup' % target_color_name
            # set waypoint 0 pose to waypoint_alot_above pose
            w_alot_above = Dummy('waypoint_alot_above')
            self.w0.set_pose(w_alot_above.get_pose())
            # gripper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
        else:
            text = 'move above the left edge of the %s cup' % target_color_name
            # set waypoint 0 pose to waypoint_above pose
            w_above = Dummy('waypoint_above')
            self.w0.set_pose(w_above.get_pose())
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()

        return [text]

    def variation_count(self) -> int:
        return len(colors) * 2
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
