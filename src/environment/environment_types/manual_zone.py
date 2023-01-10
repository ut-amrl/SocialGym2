from typing import Union, Tuple, List, TYPE_CHECKING
import numpy as np
import random
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from src.environment.ros_social_gym import RosSocialEnv
from src.environment.utils.utils import ROOT_FOLDER
from src.environment.scenarios import Scenario
from src.environment.services import UTMRSResponse

if TYPE_CHECKING:
    from src.environment.rewards import Rewarder
    from src.environment.observations.observer import Observer


class ManualZoneUTMRSResponse(UTMRSResponse):

    agents_priority_order: int
    agents_current_order: int
    in_zone: bool
    entering: bool
    exiting: bool

    number_agents_entering: bool
    number_agents_exiting: bool

    @classmethod
    def process(cls, env_response, env: 'ManualZoneEnv', *args, **kwargs) -> List['UTMRSResponse']:
        observations = []

        for robot_idx, robot_obs in enumerate(env_response.robot_responses):
            inst = cls()
            inst.set_vars(robot_obs, robot_idx, *args, **kwargs)
            observations.append(inst)

        for robot_idx, resp in enumerate(observations):
            resp.set_zone_vars(robot_idx, env, observations)

        zone = env.zone

        # In zone
        z_ur = zone['upper_right']
        z_lr = zone['lower_right']
        z_ul = zone['upper_left']
        z_ll = zone['lower_left']

        radius = 0.5

        idx_to_distances = [(idx, np.linalg.norm(x.robot_poses - ((z_lr + z_ur) / 2))) for idx, x in enumerate(observations) if env.agents_priority_orders[idx] != -1]
        distances = np.array([x[1] for x in idx_to_distances]).argsort().tolist()
        indices = [x[0] for x in idx_to_distances]

        entering_agents = [
            p.robot_poses[1] + radius >= z_ll[1] and p.robot_poses[1] - radius <= z_ul[1] and \
            p.robot_poses[0] + radius >= z_ul[0] and p.robot_poses[0] - radius <= z_ul[0] for p in observations
        ]

        exiting_agents = [
            p.robot_poses[1] + radius >= z_lr[1] and p.robot_poses[1] - radius <= z_ur[1] and \
            p.robot_poses[0] + radius >= z_ur[0] and p.robot_poses[0] - radius <= z_ur[0] for p in observations
        ]

        agents_intersecting_entrance = sum(entering_agents)
        agents_intersecting_exit = sum(exiting_agents)

        for robot_idx, resp in enumerate(observations):
            resp.agents_priority_order = env.agents_priority_orders[robot_idx]
            resp.entering = entering_agents[robot_idx]
            resp.exiting = exiting_agents[robot_idx]
            resp.number_agents_entering = agents_intersecting_entrance
            resp.number_agents_exiting = agents_intersecting_exit

            if robot_idx in indices:
                resp.agents_current_order = distances.index(indices.index(robot_idx))
            else:
                resp.agents_current_order = -1

        return observations

    def set_zone_vars(self, robot_idx: int, env: 'ManualZoneEnv', others: List['ManualZoneUTMRSResponse'], *args, **kwargs):
        zone = env.zone

        # In zone
        z_ul = zone['upper_left']
        z_ur = zone['upper_right']
        z_ll = zone['lower_left']
        z_lr = zone['lower_right']

        radius = 0.5
        r_r = self.robot_poses + np.array([radius, 0, 0])
        r_l = self.robot_poses + np.array([-radius, 0, 0])
        r_t = self.robot_poses + np.array([0, radius, 0])
        r_b = self.robot_poses + np.array([0, -radius, 0])

        self.in_zone = bool(r_r[0] > z_ul[0] and r_l[0] < z_ur[0] and r_t[1] > z_ll[1] and r_b[1] < z_ul[1])

        # We give some padding so that the exit event can be calculated (so you are not out of the ordering until you
        # are 1/2*radius away from the right edge.
        if (z_ur[0] + radius/8) < r_l[0] and env.agents_priority_orders[robot_idx] > -1:
            prev_priority = env.agents_priority_orders[robot_idx]
            env.agents_priority_orders[robot_idx] = -1
            for i in range(len(env.agents_priority_orders)):
                if env.agents_priority_orders[i] > prev_priority:
                    env.agents_priority_orders[i] -= 1



class ManualZoneEnv(RosSocialEnv):

    agents_priority_orders: List[int]

    def __init__(
            self,
            start_point: int,
            end_point: int,
            width: float,
            launch_config: str = f"{ROOT_FOLDER}/config/gym_gen/launch.launch",
            observer: 'Observer' = None,
            rewarder: 'Rewarder' = None,
            scenario: Scenario = None,
            num_humans: Union[int, Tuple[int, int]] = (5, 25),
            num_agents: Union[int, Tuple[int, int]] = (3, 5),
            debug: bool = False
    ):
        super().__init__(
            start_point, end_point, width,
            launch_config=launch_config,
            observer=observer,
            rewarder=rewarder,
            scenario=scenario,
            num_humans=num_humans,
            num_agents=num_agents,
            debug=debug
        )

        # Given two points on the navigation graph; create a box of the specified width where the two points are in the
        # middle of either side of the rectangle.

        start = np.array(self.scenario.robot_positions[start_point])
        end = np.array(self.scenario.robot_positions[end_point])

        line = end - start
        length = np.linalg.norm(line)
        line /= length
        perpendicular_points = 1/2 * width * line

        upper_left = start + np.array([0., width, 0.])
        upper_right = end + np.array([0., width, 0.])
        lower_left = start - np.array([0., width, 0.])
        lower_right = end - np.array([0., width, 0.])

        self.zone = {
            'upper_left': upper_left,
            'upper_right': upper_right,
            'lower_left': lower_left,
            'lower_right': lower_right
        }

        self.env_response_type = ManualZoneUTMRSResponse
        self.vis = RvisZoneVisualization(self.zone)

    def sim_step(self, args):
        env_response = self.env_response_type.process(self.utmrs_service.step(*args), self)
        self.vis.publish()
        return env_response

    def reset(self, seed=None, return_info=False, options=None):
        self.agents_priority_orders = random.sample(range(0, self.curr_num_agents), self.curr_num_agents)
        res = super().reset(seed, return_info, options)
        return res


class RvisZoneVisualization:

    def __init__(self, zone):
        self.zone_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
        self.points_pub = rospy.Publisher("/visualization_marker_points", Marker, queue_size=100)

        self.marker = Marker()
        self.points = Marker()

        self.rect(zone)
        self.point([v for v in zone.values()])

    def point(self, points):
        p = Marker()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        p.type = 8
        p.id = 0

        p.color.r = 0.0
        p.color.g = 0.0
        p.color.b = 1.0
        p.color.a = 0.5

        p.scale.x = 0.2
        p.scale.y = 0.2
        p.scale.z = 1.0

        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0

        pnts = []

        for point in points:
            a = Point()
            a.x = point[0]
            a.y = point[1]
            a.z = 1.0
            pnts.append(a)


        p.points = pnts

        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0

        self.points = p

    def rect(self, zone):
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self. marker.type = 1
        self.marker.id = 0

        # Set the scale of the marker
        self.marker.scale.x = abs(zone['upper_right'][0] - zone['upper_left'][0])
        self.marker.scale.y = abs(zone['upper_right'][1] - zone['lower_right'][1])
        self.marker.scale.z = 1.0

        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5

        # Set the pose of the marker
        self.marker.pose.position.x = (zone['upper_right'][0] + zone['upper_left'][0])/2
        self. marker.pose.position.y = (zone['upper_right'][1] + zone['lower_right'][1])/2
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

    def publish(self):
        self.zone_pub.publish(self.marker)
        self.points_pub.publish(self.points)
