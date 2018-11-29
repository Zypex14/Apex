from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from States import *
from Util import *


class Apex(BaseAgent):

    def initialize_agent(self):
        self.me = obj()
        self.ball = obj()
        self.closest_opponent = obj()
        self.start = time.time()
        self.str_state = "QuickShot"

    def choose_state(self):
        if abs(self.allignment) < 1:
            self.state = QuickShot()
            self.str_state = "QuickShot"
            self.controller = take_shot_controller
        else:
            self.state = FollowBall()
            self.str_state = "FollowBall"
            self.controller = follow_controller

    def get_output(self, game: GameTickPacket) -> SimpleControllerState:

        # Fetch the good stuff from the game tick packet

        self.preprocess(game)
        self.choose_state()

        print(self.me.local_velocity.data[0])

        self.renderer.begin_rendering()
        self.renderer.draw_string_2d(2, 2, 2, 2, "Ball/Goal/Car allignment:", self.renderer.black())
        self.renderer.draw_string_2d(2, 27, 2, 2, str(self.allignment), self.renderer.black())
        self.renderer.draw_string_2d(2, 52, 2, 2, "Behavior State: " + self.str_state, self.renderer.black())

        self.renderer.draw_string_3d((0, 0, 0), 2, 2, "Origin: (0,0,0)", self.renderer.black())
        self.renderer.draw_line_3d(
            (self.own_goal_location.data[0], self.own_goal_location.data[1], self.own_goal_location.data[2]),
            (self.ball.location.data[0], self.ball.location.data[1], self.ball.location.data[2]),
            self.renderer.create_color(255, 10, 10, 0))
        
        self.renderer.draw_line_3d((self.goal_location.data[0], self.goal_location.data[1], self.goal_location.data[2]),
                                   (self.ball.location.data[0], self.ball.location.data[1], self.ball.location.data[2]),
                                   self.renderer.create_color(10, 255, 10, 0))
        
        self.renderer.draw_string_3d(
            (self.closest_opponent.location.data[0], self.closest_opponent.location.data[1],
             self.closest_opponent.location.data[2]), 2, 2, "Closest Opponent", self.renderer.black())
        self.renderer.end_rendering()

        self.renderer.draw_string_3d(
            (self.closest_boost_pad.data[0], self.closest_boost_pad.data[1],
             self.closest_boost_pad.data[2]), 2, 2, "Closest Boost Pad", self.renderer.black())
        self.renderer.end_rendering()

        return self.state.execute(self)

    def preprocess(self, game):
        self.me.location.data = [game.game_cars[self.index].physics.location.x,
                                 game.game_cars[self.index].physics.location.y,
                                 game.game_cars[self.index].physics.location.z]

        self.me.velocity.data = [game.game_cars[self.index].physics.velocity.x,
                                 game.game_cars[self.index].physics.velocity.y,
                                 game.game_cars[self.index].physics.velocity.z]

        self.me.rotation.data = [game.game_cars[self.index].physics.rotation.pitch,
                                 game.game_cars[self.index].physics.rotation.yaw,
                                 game.game_cars[self.index].physics.rotation.roll]

        self.me.rvelocity.data = [game.game_cars[self.index].physics.angular_velocity.x,
                                  game.game_cars[self.index].physics.angular_velocity.y,
                                  game.game_cars[self.index].physics.angular_velocity.z]

        self.me.matrix = rotator_to_matrix(self.me)

        self.me.local_velocity = toLocal(self.me.velocity, self.me)

        self.me.boost = game.game_cars[self.index].boost

        self.ball.location.data = [game.game_ball.physics.location.x, game.game_ball.physics.location.y,
                                   game.game_ball.physics.location.z]

        self.ball.velocity.data = [game.game_ball.physics.velocity.x, game.game_ball.physics.velocity.y,
                                   game.game_ball.physics.velocity.z]

        self.ball.rotation.data = [game.game_ball.physics.rotation.pitch, game.game_ball.physics.rotation.yaw,
                                   game.game_ball.physics.rotation.roll]

        self.ball.rvelocity.data = [game.game_ball.physics.angular_velocity.x,
                                    game.game_ball.physics.angular_velocity.y,
                                    game.game_ball.physics.angular_velocity.z]

        self.own_goal_location = Vector3([self.get_field_info().goals[self.team].location.x,
                                          self.get_field_info().goals[self.team].location.y,
                                          self.get_field_info().goals[self.team].location.z])

        self.other_team = 1 if self.team == 0 else 0

        self.goal_location = Vector3([self.get_field_info().goals[self.other_team].location.x,
                                      self.get_field_info().goals[self.other_team].location.y,
                                      self.get_field_info().goals[self.other_team].location.z])

        self.ball.local_location = to_local(self.ball, self.me)

        self.cars = game.game_cars

        self.allignment = angle2D(self.goal_location, self.ball) - angle2D(self.ball, self.me)

        self.closest_opponent = self.find_closest_car()
        
        self.closest_boost_pad = self.find_closest_boost()

    def find_closest_car(self):

        id = 0

        while self.cars[id].team == self.team:
            id += 1

        final_car = self.cars[id]

        for car in self.cars:
            location = [car.physics.location.x,
                        car.physics.location.y,
                        car.physics.location.z]
            final_car_location = [final_car.physics.location.x,
                                  final_car.physics.location.y,
                                  final_car.physics.location.z]

            if car.team == self.other_team and distance2D(self.me, location) < distance2D(self.me, final_car_location):
                final_car = car

        out = obj()

        out.location.data = [final_car.physics.location.x,
                             final_car.physics.location.y,
                             final_car.physics.location.z]

        out.velocity.data = [final_car.physics.velocity.x,
                             final_car.physics.velocity.y,
                             final_car.physics.velocity.z]

        out.rotation.data = [final_car.physics.rotation.pitch,
                             final_car.physics.rotation.yaw,
                             final_car.physics.rotation.roll]

        out.rvelocity.data = [final_car.physics.angular_velocity.x,
                              final_car.physics.angular_velocity.y,
                              final_car.physics.angular_velocity.z]

        return out

    def find_closest_boost(self):
        out = Vector3([self.get_field_info().boost_pads[0].location.x, self.get_field_info().boost_pads[0].location.y,
                       self.get_field_info().boost_pads[0].location.z])

        for boost in self.get_field_info().boost_pads:
            current = Vector3([boost.location.x, boost.location.y, boost.location.z])
            if distance2D(current, self.me) < distance2D(out, self.me):
                out = current
        
        return out