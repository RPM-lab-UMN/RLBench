from typing import List
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import DetectedCondition

OPTIONS = ['left', 'right']

class TurnTapMotions(Task):

    def init_task(self) -> None:
        self.left_start = Dummy('waypoint0')
        self.left_end = Dummy('waypoint1')
        self.right_start = Dummy('waypoint5')
        self.right_end = Dummy('waypoint6')
        self.left_joint = Joint('left_joint')
        self.right_joint = Joint('right_joint')
        self.register_stop_at_waypoint(1)
        self.sensor = ProximitySensor('sensor')
        self.tip = Dummy('Panda_tip')
        self.register_success_conditions([
            DetectedCondition(self.tip, self.sensor)
        ])

    def init_episode(self, index: int, seed = None, interactive=False) -> List[str]:
        option = OPTIONS[index]
        if option == 'right':
            self.left_start.set_position(self.right_start.get_position())
            self.left_start.set_orientation(self.right_start.get_orientation())
            self.left_end.set_position(self.right_end.get_position())
            self.left_end.set_orientation(self.right_end.get_orientation())

        return ['move %s tap' % option]

    def variation_count(self) -> int:
        return 2
