import math
import time
from rlbot.agents.base_agent import SimpleControllerState
from Util import *


def get_state(state):
    switcher = {
        FollowBall: "FollowBall",
        QuickShot: "QuickShot"
    }

    return switcher.get(state, state)




class FollowBall:
    def __init__(self):
        self.expired = False

    def execute(self, agent):

        if abs(angle2D(agent.closest_boost_pad, agent.me) - angle2D(agent.ball, agent.me)) < 0.5 and distance2D(agent.ball, agent.me) > distance2D(agent.closest_boost_pad, agent.me) * agent.me.boost / 25.:
            target_location = agent.closest_boost_pad
        else:
            target_location = agent.ball
        target_speed = velocity2D(agent.ball) + (distance2D(agent.ball, agent.me) / 1.5) + (10000 - cap(distance2D(agent.closest_opponent, agent.me), 0, 10000))
        # print(distance2D(agent.ball, agent.me))

        return agent.controller(agent, target_location, target_speed)


class QuickShot:
    def __init__(self):
        self.expired = False

    def execute(self, agent):
        agent.controller = take_shot_controller

        target_speed = 2300

        add_angle = angle2D(agent.goal_location, agent.ball) - angle2D(agent.ball, agent.me)

        if abs(angle2D(agent.closest_boost_pad, agent.me) - angle2D(agent.ball, agent.me)) < 0.5 and distance2D(agent.ball, agent.me) > distance2D(agent.closest_boost_pad, agent.me) * agent.me.boost / 25.:
            target_location = toLocal(agent.closest_boost_pad, agent.me)
        else:
            target_location = [agent.ball.local_location.data[0], agent.ball.local_location.data[1] + add_angle * -150,
                               agent.ball.local_location.data[2]]
        return agent.controller(agent, target_location, target_speed)


def follow_controller(agent, target_object, target_speed):
    location = toLocal(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1], location.data[0])
    current_speed = velocity2D(agent.me)

    pitch = agent.me.location.data[0] % math.pi
    roll = agent.me.location.data[2] % math.pi

    # Turn dem wheels
    controller_state.steer = cap(angle_to_ball * 5, -1, 1)
    print(velocity2D(agent.me))

    if abs(angle_to_ball) > (math.pi / 3.):
        controller_state.handbrake = True
    else:
        controller_state.handbrake = False

    # throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250 and abs(angle_to_ball) < (math.pi / 3.):
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    # doging
    time_difference = time.time() - agent.start

    if time_difference > 2.2 and distance2D(target_object, agent.me) > 1000 and abs(
            angle_to_ball) < 1.3 and velocity2D(agent.me) > 1200:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = controller_state.steer
        controller_state.pitch = -1

    # print("target: " + str(target_speed) + ", current: " + str(current_speed))
    return controller_state


def take_shot_controller(agent, target_object, target_speed):
    goal_local = toLocation([0, -sign(agent.team) * FIELD_LENGTH / 2, 100])
    goal_angle = math.atan2(goal_local.data[1], goal_local.data[0])

    pitch = agent.me.location.data[0] % math.pi
    roll = agent.me.location.data[2] % math.pi

    location = toLocation(target_object)
    controller_state = SimpleControllerState()
    angle_to_target = math.atan2(location.data[1], location.data[0])

    current_speed = velocity2D(agent.me)

    controller_state.steer = cap(angle_to_target * 5, -1, 1)

    # throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    # doging
    time_difference = time.time() - agent.start

    if time_difference > 2.2 and abs(angle_to_target) < 1.3 and velocity2D(agent.me) > 1200 and distance2D(target_object, agent.me) < 1000:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = cap(math.sin(goal_angle) / math.pi, -1, 1)
        controller_state.pitch = -1

    # print("target: " + str(target_speed) + ", current: " + str(current_speed))
    return controller_state
