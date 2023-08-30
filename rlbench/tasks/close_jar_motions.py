from typing import List, Tuple
from rlbench.backend.task import Task
from typing import List
from rlbench.backend.task import Task
from rlbench.const import colors, alt_colors
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.dummy import Dummy

class CloseJarMotions(Task):
    
    def init_task(self) -> None:
        self.lid = Shape('jar_lid0')
        self.jars = [Shape('jar%d' % i) for i in range(2)]
        self.register_graspable_objects([self.lid])
        self.boundary = Shape('spawn_boundary')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('success')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed = None) -> List[str]:
        # set random seed
        if seed is not None:
            np.random.seed(seed)
        else:
            print('no seed!')
            
        if index >= len(colors):
            index -= len(colors)
            lid = True
        else:
            lid = False

        b = SpawnBoundary([self.boundary])
        for obj in self.jars:
            b.sample(obj, min_distance=0.01)
        w3 = Dummy('waypoint3')
        w3.set_orientation([-np.pi, 0, -np.pi], reset_dynamics=False)
        w3.set_position([0.0, 0.0, 0.125], relative_to=self.jars[index % 2],
                        reset_dynamics=False)
        target_color_name, target_color_rgb = colors[index]
        alt_color_name = alt_colors[index]
        color_choice = np.random.choice(
            list(range(index)) + list(
                range(index + 1, len(colors))),
            size=1, replace=False)[0]
        _, distractor_color_rgb = colors[color_choice]
        self.jars[index % 2].set_color(target_color_rgb)
        other_index = {0: 1, 1: 0}
        self.jars[other_index[index % 2]].set_color(distractor_color_rgb)
        w0 = Dummy('waypoint0')

        text = ['0', '1', '2', '3', '4', '5']
        if lid:
            # move waypoint 0 to lid
            w_lid = Dummy('waypoint_lid')
            w0.set_pose(w_lid.get_pose())
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()
            text[0] = 'move above the lid'
            text[1] = 'approach the lid'
            text[2] = 'go over the lid'
            text[3] = 'point the gripper at the lid from above'
            text[4] = 'go above the lid'
            text[5] = 'prepare to grasp the lid'
        else:
            text[0] = 'move above the %s jar' % target_color_name
            text[1] = 'approach the %s jar from above' % alt_color_name
            text[2] = 'go over the %s jar' % target_color_name
            text[3] = 'align the gripper with the %s jar' % alt_color_name
            text[4] = 'go above the %s jar' % target_color_name
            text[5] = 'align the lid to the %s jar' % target_color_name

            # gripper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
            # move waypoint 0 to jar (w3)
            w0.set_pose(w3.get_pose())
        # set orientation to be consistent
        w0.set_orientation([-np.pi, 0, -np.pi], reset_dynamics=False)
        return text

    def variation_count(self) -> int:
        return len(colors) * 2 # move above jar or lid for every color

    def cleanup(self) -> None:
        self.conditions = [NothingGrasped(self.robot.gripper)]

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        # This is here to stop the issue of gripper rotation joint reaching its
        # limit and not being able to go through the full range of rotation to
        # unscrew, leading to a weird jitery and tilted cap while unscrewing.
        # Issue occured rarely so is only minor
        return (0.0, 0.0, -0.6*np.pi), (0.0, 0.0, +0.6*np.pi)