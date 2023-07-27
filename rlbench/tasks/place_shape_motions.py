from cgitb import reset
from re import T
from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import DetectedCondition

SHAPE_NAMES = ['cube', 'cylinder', 'prism', 'star', 'moon']


class PlaceShapeMotions(Task):

    def init_task(self) -> None:
        self.shape_sorter = Shape('shape_sorter')
        self.success_sensor = ProximitySensor('success')
        self.shapes = [Shape(ob.replace(' ', '_')) for ob in SHAPE_NAMES]
        self.drop_points = [
            Dummy('%s_drop_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self.grasp_points = [
            Dummy('%s_grasp_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint4 = Dummy('waypoint4')
        self.register_graspable_objects(self.shapes)

        self.boundary = SpawnBoundary([Shape('boundary')])

        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index, seed=None) -> List[str]:
        if index >= len(SHAPE_NAMES):
            hole = True
            index -= len(SHAPE_NAMES)
        else:
            hole = False

        self.variation_index = index
        shape = SHAPE_NAMES[index]

        self.boundary.clear()
        [self.boundary.sample(s, min_distance=0.05) for s in self.shapes]

        # step sim to settle
        for _ in range(3):
            self.pyrep.step()

        # set cylinder orientation
        cylinder = self.shapes[1]
        cylinder.set_orientation([0, 0, 0])

        if hole:
            # move waypoint 1 to the hole
            self._set_drop()
            text = 'move above the %s hole' % shape
        else:
            # move waypoint 1 to the shape
            self._set_grasp()
            text = 'move above the %s' % shape

        return [text]

    def variation_count(self) -> int:
        return len(SHAPE_NAMES) * 2 # above shapes and above holes

    def _set_grasp(self):
        gp = self.grasp_points[self.variation_index]
        self.waypoint1.set_pose(gp.get_pose())

    def _set_drop(self):
        dp = self.drop_points[self.variation_index]
        self.waypoint1.set_pose(dp.get_pose())
