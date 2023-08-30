from typing import List
import itertools
import math
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import JointCondition, ConditionSet
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition

MAX_TARGET_BUTTONS = 3
MAX_VARIATIONS = 50

# button top plate and wrapper will be be red before task completion
# and be changed to cyan upon success of task, so colors list used to randomly vary colors of
# base block will be redefined, excluding red and green
colors = [
    ('maroon', (0.5, 0.0, 0.0)),
    ('green', (0.0, 0.5, 0.0)),
    ('blue', (0.0, 0.0, 1.0)),
    ('navy', (0.0, 0.0, 0.5)),
    ('yellow', (1.0, 1.0, 0.0)),
    ('cyan', (0.0, 1.0, 1.0)),
    ('magenta', (1.0, 0.0, 1.0)),
    ('silver', (0.75, 0.75, 0.75)),
    ('gray', (0.5, 0.5, 0.5)),
    ('orange', (1.0, 0.5, 0.0)),
    ('olive', (0.5, 0.5, 0.0)),
    ('purple', (0.5, 0.0, 0.5)),
    ('teal', (0, 0.5, 0.5)),
    ('azure', (0.0, 0.5, 1.0)),
    ('violet', (0.5, 0.0, 1.0)),
    ('rose', (1.0, 0.0, 0.5)),
    ('black', (0.0, 0.0, 0.0)),
    ('white', (1.0, 1.0, 1.0)),
]

color_permutations = list(itertools.permutations(colors, 3))


def print_permutations(color_permutations):
    # pretty printing color_permutations for debug
    print('num permutations: ', str(len(color_permutations)))
    print('color_permutations:\n')
    for i in range(len(color_permutations)):
        print(str(color_permutations[i]))
        if ((i + 1) % 16 == 0): print('')


class PushButtonsMotions(Task):

    def init_task(self) -> None:
        self.buttons_pushed = 0
        self.color_variation_index = 0
        self.target_buttons = [Shape('push_buttons_target%d' % i)
                               for i in range(3)]
        self.target_topPlates = [Shape('target_button_topPlate%d' % i)
                                 for i in range(3)]
        self.target_joints = [Joint('target_button_joint%d' % i)
                              for i in range(3)]
        self.target_wraps = [Shape('target_button_wrap%d' % i)
                             for i in range(3)]
        self.boundaries = Shape('push_buttons_boundary')
        # goal_conditions merely state joint conditions for push action for
        # each button regardless of whether the task involves pushing it
        self.goal_conditions = [JointCondition(self.target_joints[n], 0.001)
                                for n in range(3)]

        self.register_waypoint_ability_start(0, self._move_above_next_target)
        self.register_waypoints_should_repeat(self._repeat)
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed=None) -> List[str]:
        # set random seed
        if seed is not None:
            np.random.seed(seed)
        else:
            print('no seed!')
            
        for tp in self.target_topPlates:
            tp.set_color([1.0, 0.0, 0.0])
        for w in self.target_wraps:
            w.set_color([1.0, 0.0, 0.0])
        # For each color permutation, we want to have 1, 2 or 3 buttons pushed
        color_index = index
        self.buttons_to_push = 1 + index % MAX_TARGET_BUTTONS
        button_colors = []
        # first button color is index
        button_colors.append(colors[color_index])
        # second and third are random different colors
        spare_colors = list(set(colors) - set([colors[color_index]]))
        color_choice_indexes = np.random.choice(range(len(spare_colors)),
                                                size=2,
                                                replace=False)
        for i in range(2):
            button_colors.append(spare_colors[color_choice_indexes[i]])

        self.color_names = []
        self.color_rgbs = []
        self.chosen_colors = []
        i = 0
        for b in self.target_buttons:
            color_name, color_rgb = button_colors[i]
            self.color_names.append(color_name)
            self.color_rgbs.append(color_rgb)
            self.chosen_colors.append((color_name, color_rgb))
            b.set_color(color_rgb)
            i += 1

        rtn0 = 'push the %s button' % self.color_names[0]
        rtn1 = 'press the %s button' % self.color_names[0]
        rtn2 = 'push down the button with the %s base' % self.color_names[0]
        for i in range(self.buttons_to_push):
            if i == 0:
                continue
            else:
                rtn0 += ', then push the %s button' % self.color_names[i]
                rtn1 += ', then press the %s button' % self.color_names[i]
                rtn2 += ', then the %s one' % self.color_names[i]

        b = SpawnBoundary([self.boundaries])
        for button in self.target_buttons:
            b.sample(button, min_distance=0.1)

        num_non_targets = 3 - self.buttons_to_push
        spare_colors = list(set(colors)
                            - set(
            [self.chosen_colors[i] for i in range(self.buttons_to_push)]))

        spare_color_rgbs = []
        for i in range(len(spare_colors)):
            _, rgb = spare_colors[i]
            spare_color_rgbs.append(rgb)

        color_choice_indexes = np.random.choice(range(len(spare_colors)),
                                                size=num_non_targets,
                                                replace=False)
        non_target_index = 0
        for i, button in enumerate(self.target_buttons):
            if i in range(self.buttons_to_push):
                pass
            else:
                _, rgb = spare_colors[color_choice_indexes[non_target_index]]
                button.set_color(rgb)
                non_target_index += 1

        # gripper should be closed
        while self.robot.gripper.get_open_amount()[0] > 0.01:
            self.robot.gripper.actuate(0, velocity=0.1)
            self.pyrep.step()

        text = ['0', '1', '2', '3', '4', '5']
        text[0] = 'move above the %s button' % self.color_names[0]
        text[1] = 'approach the %s button' % self.color_names[0]
        text[2] = 'go above the %s button' % self.color_names[0]
        text[3] = 'point the gripper at the %s button from above' % self.color_names[0]
        text[4] = 'go above the %s button' % self.color_names[0]
        text[5] = 'position the gripper above the %s button' % self.color_names[0]

        return text

    def variation_count(self) -> int:
        return len(colors)

    def step(self) -> None:
        for i in range(len(self.target_buttons)):
            if self.goal_conditions[i].condition_met() == (True, True):
                self.target_topPlates[i].set_color([0.0, 1.0, 0.0])
                self.target_wraps[i].set_color([0.0, 1.0, 0.0])

    def cleanup(self) -> None:
        self.buttons_pushed = 0

    def _move_above_next_target(self, waypoint):
        if self.buttons_pushed >= self.buttons_to_push:
            print('buttons_pushed:', self.buttons_pushed, 'buttons_to_push:',
                  self.buttons_to_push)
            raise RuntimeError('Should not be here.')
        w0 = Dummy('waypoint0')
        x, y, z = self.target_buttons[self.buttons_pushed].get_position()
        w0.set_position([x, y, z + 0.083])
        w0.set_orientation([math.pi, 0, math.pi])

    def _repeat(self):
        self.buttons_pushed += 1
        return self.buttons_pushed < self.buttons_to_push
