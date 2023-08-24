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

    def init_episode(self, index: int, seed = None) -> List[str]:
        option = OPTIONS[index]
        if option == 'right':
            self.left_start.set_position(self.right_start.get_position())
            self.left_start.set_orientation(self.right_start.get_orientation())
            self.left_end.set_position(self.right_end.get_position())
            self.left_end.set_orientation(self.right_end.get_orientation())

        text = ['0', '1', '2', '3', '4', '5']
        if option == 'left':
            text[0] = 'move to the left tap'
            text[1] = 'approach the left handle'
            text[2] = 'go to the left tap'
            text[3] = 'prepare to turn the left tap'
            text[4] = 'move to the left handle'
            text[5] = 'move to the left valve'
        elif option == 'right':
            text[0] = 'move to the right tap'
            text[1] = 'approach the right handle'
            text[2] = 'go to the right tap'
            text[3] = 'prepare to turn the right tap'
            text[4] = 'move to the right handle'
            text[5] = 'move to the right valve'

        return text

    def variation_count(self) -> int:
        return 2
