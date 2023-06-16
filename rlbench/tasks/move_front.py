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

class MoveFront(Task):

    def init_task(self) -> None:
        self.success_sensor = ProximitySensor('success1')
        self.plane = Shape('plane')
        # blocks
        # self.block1 = Shape('block1')
        # self.block2 = Shape('block2')
        # waypoints
        self.waypoint0 = Dummy('waypoint0')
        self.tip = Dummy('Panda_tip')
        # bowl
        self.register_success_conditions([
            DetectedCondition(self.tip, self.success_sensor)
        ])
        self.target = None
        self.distractor = None

    def init_episode(self, index: int, seed = None) -> List[str]:
        # move robot to initial position
        # 0.25, 0, 1.0
        # j = np.array([1.90242633e-01, -1.82561681e-03, -1.74581066e-01, -2.33221745e+00, -1.09314790e-03,  2.26251936e+00,  8.01950991e-01])
        # 0.25, 0, 1.2
        j = np.array([-0.58046514, -0.16537985,  0.54929668, -1.96424127,  0.08739433, 1.75281847,  0.73243582])
        self.robot.arm.set_joint_positions(j, disable_dynamics=True)

        # if platform, target, or distractor exist, remove them
        try:
            target = Shape('target')
            target.remove()
            distractor = Shape('distractor')
            distractor.remove()
            platform = Shape('platform')
            platform.remove()
        except:
            pass
        
        # set random seed
        if seed is not None:
            # set seed
            np.random.seed(seed)
            # np.random = np.random.default_np.random(seed)
            # np.random.seed(seed)
            # print('epsiode seed: ', seed)
        else:
            print('no seed!')
            # np.random = np.random.default_np.random()

        # create platform of random height
        platform_height = np.random.uniform(0.01, 0.2)
        # create platform
        platform = Shape.create(type=PrimitiveShape.CUBOID, size=[0.5, 0.5, platform_height], position=[0.35, 0, 0.75])
        platform.set_color([0.5, 0.5, 0.5])
        platform.set_name('platform')
        # set the plane above the platform
        self.plane.set_position([0.35, 0, 0.85 + platform_height/2])
        # create spawn boundary
        self.boundary = SpawnBoundary([self.plane])

        # create a target with random size
        sx = np.random.uniform(0.05, 0.1)
        sy = np.random.uniform(0.05, 0.1)
        sz = np.random.uniform(0.05, 0.1)
        # type is a random choice from PrimitiveShape
        type = np.random.choice(list(PrimitiveShape))
        if type == PrimitiveShape.SPHERE:
            size = [sz]
        else:
            size = [sx, sy, sz]
        target_height = sz
        self.target = Shape.create(type=type, size=size, position=[0, 0, 0.8], color=[1, 0, 0])
        # set the name of the object
        self.target.set_name('target')
        # string of target object type
        target_type = str(type).split('.')[1].lower()

        # create distractor object
        sx = np.random.uniform(0.05, 0.1)
        sy = np.random.uniform(0.05, 0.1)
        sz = np.random.uniform(0.05, 0.1)
        # type is a random choice from PrimitiveShape
        type = np.random.choice(list(PrimitiveShape))
        if type == PrimitiveShape.SPHERE:
            size = [sz]
        else:
            size = [sx, sy, sz]
        self.distractor = Shape.create(type=type, size=size, position=[0, 0.15, 0.8], color=[0, 1, 0])
        self.distractor.set_name('distractor')
        distractor_type = str(type).split('.')[1].lower()

        # spawn objects in the workspace
        self.boundary.clear()
        for ob in [self.target, self.distractor]:
            self.boundary.sample(ob, ignore_collisions=False, min_distance=0.16, 
                                 min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        
        # step the simulation
        for _ in range(10):
            self.pyrep.step()
        # get the position of target
        target_pos = self.target.get_position()

        # set the block colors
        text = []
        valid_idx = np.array([0, 2, 4])
        if index == 0:
            color = 0
            self.target.set_color(colors[color][1])
            target_color = 'red'
        elif index == 1:
            color = 2
            self.target.set_color(colors[color][1])
            target_color = 'green'
        elif index == 2:
            color = 4
            self.target.set_color(colors[color][1])
            target_color = 'blue'

        # move the sensor above the target
        self.waypoint0.set_position([target_pos[0], target_pos[1], target_pos[2] + target_height/2 + 0.025])
        self.success_sensor.set_position([target_pos[0], target_pos[1], target_pos[2] + target_height/2])

        # random valid color for distractor block
        color2 = np.random.choice(valid_idx)
        while color2 == color:
            color2 = np.random.choice(valid_idx)
        self.distractor.set_color(colors[color2][1])

        # generate all the text options
        # move above the red object
        # move above the red sphere
        # move above the sphere
        # go above...
        lang_goals = []
        text1 = ['move above the ', 'go above the ']
        text2 = [target_color]
        text3 = ['object', target_type]
        for t1 in text1:
            for t2 in text2:
                for t3 in text3:
                    lang_goals.append(t1 + t2 + ' ' + t3)
        # add non-color commands if distractor is not the same shape
        if distractor_type != target_type:
            for t1 in text1:
                lang_goals.append(t1 + text3[1])
        lang_goal_idx = np.random.choice(len(lang_goals))
        # move lang_goal_idx to the front of the list
        lang_goals.insert(0, lang_goals.pop(lang_goal_idx))

        return lang_goals

    def variation_count(self) -> int:
        return 3
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return ((0, 0, 0), (0, 0, 0))
    
    def is_static_workspace(self) -> bool:
        return True