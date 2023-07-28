from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

GROCERY_NAMES = [
    'crackers',
    'chocolate jello',
    'strawberry jello',
    'soup',
    'tuna',
    'spam',
    'coffee',
    'mustard',
    'sugar',
]


class PutGroceriesInCupboardMotions(Task):

    def init_task(self) -> None:
        self.groceries = [Shape(name.replace(' ', '_'))
                          for name in GROCERY_NAMES]
        self.grasp_points = [Dummy('%s_grasp_point' % name.replace(' ', '_'))
                             for name in GROCERY_NAMES]
        self.waypoint1 = Dummy('waypoint1')
        self.register_graspable_objects(self.groceries)
        self.boundary = SpawnBoundary([Shape('workspace')])
        self.w0 = Dummy('waypoint0')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed=None) -> List[str]:
        if index == 0:
            text = 'move in front of the cupboard'
            cupboard = True
        else:
            cupboard = False
            index -= 1

        self.boundary.clear()
        [self.boundary.sample(g, min_distance=0.1) for g in self.groceries]
        self.waypoint1.set_pose(self.grasp_points[index].get_pose())

        if cupboard:
            # move waypoint 0 to cupboard
            w_cupboard = Dummy('waypoint3')
            self.w0.set_pose(w_cupboard.get_pose())
            # gripper closed
            while self.robot.gripper.get_open_amount()[0] > 0.01:
                self.robot.gripper.actuate(0, velocity=0.1)
                self.pyrep.step()
        else:
            # move waypoint 0 to grocery
            text = 'move above the %s' % GROCERY_NAMES[index]
            w_grocery = Dummy('waypoint_grocery')
            self.w0.set_pose(w_grocery.get_pose())
            # gripper open
            while self.robot.gripper.get_open_amount()[0] < 0.99:
                self.robot.gripper.actuate(1, velocity=0.1)
                self.pyrep.step()

        return [text]

    def variation_count(self) -> int:
        return len(GROCERY_NAMES) + 1 # +1 for move in front of the cupboard

    def boundary_root(self) -> Object:
        return Shape('boundary_root')

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -1.), (0.0, 0.0, 1.)

